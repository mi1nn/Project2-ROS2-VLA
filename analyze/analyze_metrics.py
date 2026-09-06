#!/usr/bin/env python3
"""MongoDB 기반 ROS2-VLA 초기 MVP 정량 평가 스크립트.

현재 프로젝트 컬렉션
- commands
- component_executions
- kit_executions

선택 평가 컬렉션(정답 데이터가 있을 때)
- evaluation_annotations: 음성 정답

실행 예:
  set -a && source .env && set +a
  python3 analyze/analyze_metrics.py --limit 20
  python3 analyze/analyze_metrics.py --limit 20 --output metrics.json

환경 변수:
  MONGO_HOST, MONGO_PORT, MONGO_DATABASE,
  MONGO_ROOT_USER, MONGO_ROOT_PASSWORD
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

try:
    from pymongo import MongoClient
except ImportError as exc:
    raise SystemExit("pymongo 설치가 필요합니다") from exc


REQUIRED_COMMAND_FIELDS = {"kit_type", "items"}
FINAL_TASK_STATUSES = {"SUCCESS", "FAILED"}


def env_required(name: str) -> str:
    value = os.getenv(name)
    if value is None or not value.strip():
        raise ValueError(f"환경 변수 {name}이 필요합니다.")
    return value


def get_database():
    host = os.getenv("MONGO_HOST", "127.0.0.1")
    port = int(os.getenv("MONGO_PORT", "27017"))
    database = env_required("MONGO_DATABASE")
    user = env_required("MONGO_ROOT_USER")
    password = env_required("MONGO_ROOT_PASSWORD")
    client = MongoClient(
        host=host,
        port=port,
        username=user,
        password=password,
        authSource=os.getenv("MONGO_AUTH_DATABASE", "admin"),
        serverSelectionTimeoutMS=5000,
    )
    client.admin.command("ping")
    return client, client[database]


def dt_value(value: Any) -> datetime:
    if isinstance(value, datetime):
        return value if value.tzinfo else value.replace(tzinfo=timezone.utc)
    return datetime.min.replace(tzinfo=timezone.utc)


def pct(num: int | float, den: int | float) -> float | None:
    return round(100.0 * num / den, 2) if den else None


def normalize_text(text: Any) -> str:
    text = str(text or "").lower().strip()
    return re.sub(r"[^0-9a-zA-Z가-힣]+", "", text)


def valid_command_schema(command: Any) -> tuple[bool, list[str]]:
    errors: list[str] = []
    if not isinstance(command, dict):
        return False, ["command가 JSON object가 아님"]
    missing = REQUIRED_COMMAND_FIELDS - command.keys()
    if missing:
        errors.append("필수 필드 누락: " + ", ".join(sorted(missing)))
    if not isinstance(command.get("kit_type"), str) or not command.get("kit_type", "").strip():
        errors.append("kit_type이 비어 있거나 문자열이 아님")
    items = command.get("items")
    if not isinstance(items, list) or not items:
        errors.append("items가 비어 있거나 배열이 아님")
    else:
        for i, item in enumerate(items):
            if not isinstance(item, dict):
                errors.append(f"items[{i}]가 object가 아님")
                continue
            if not isinstance(item.get("name"), str) or not item.get("name", "").strip():
                errors.append(f"items[{i}].name 오류")
            qty = item.get("qty")
            if isinstance(qty, bool) or not isinstance(qty, int) or qty <= 0:
                errors.append(f"items[{i}].qty는 양의 정수여야 함")
    return not errors, errors


def select_tasks(db, limit: int) -> list[dict]:
    tasks = list(db.kit_executions.find({"status": {"$in": list(FINAL_TASK_STATUSES)}}))
    tasks.sort(key=lambda d: dt_value(d.get("ended_at") or d.get("started_at")))
    return tasks[-limit:]


def command_metrics(db, task_ids: list[str]) -> dict:
    docs = list(db.commands.find({"task_id": {"$in": task_ids}}))
    by_task = {d.get("task_id"): d for d in docs}
    valid = 0
    published_success = 0
    failures = []
    for task_id in task_ids:
        doc = by_task.get(task_id)
        if not doc:
            failures.append({"task_id": task_id, "errors": ["commands 문서 없음"]})
            continue
        ok, errors = valid_command_schema(doc.get("command"))
        if ok:
            valid += 1
        else:
            failures.append({"task_id": task_id, "errors": errors})
        if bool((doc.get("validation") or {}).get("success")):
            published_success += 1
    return {
        "denominator": len(task_ids),
        "valid_json_count": valid,
        "valid_json_rate_pct": pct(valid, len(task_ids)),
        "validation_success_count": published_success,
        "validation_success_rate_pct": pct(published_success, len(task_ids)),
        "invalid_samples": failures,
    }


def voice_metrics(db, task_ids: list[str]) -> dict:
    """정답 annotation과 commands.raw_text를 비교한다.

    evaluation_annotations 예:
    {task_id, expected_text: "지진 키트 만들어줘", keywords: ["지진", "키트"]}
    expected_text 또는 keywords 중 하나 이상을 기록한다.
    """
    commands = {d.get("task_id"): d for d in db.commands.find({"task_id": {"$in": task_ids}})}
    annotations = list(db.evaluation_annotations.find({"task_id": {"$in": task_ids}}))
    exact_total = exact_ok = keyword_total = keyword_ok = 0
    details = []
    for ann in annotations:
        task_id = ann.get("task_id")
        raw = (commands.get(task_id) or {}).get("raw_text", "")
        row = {"task_id": task_id, "raw_text": raw}
        expected = ann.get("expected_text")
        if isinstance(expected, str) and expected.strip():
            exact_total += 1
            row["sentence_match"] = normalize_text(raw) == normalize_text(expected)
            exact_ok += int(row["sentence_match"])
        keywords = ann.get("keywords")
        if isinstance(keywords, list) and keywords:
            keyword_total += 1
            normalized_raw = normalize_text(raw)
            row["keyword_match"] = all(normalize_text(k) in normalized_raw for k in keywords)
            keyword_ok += int(row["keyword_match"])
        details.append(row)
    available = bool(annotations)
    return {
        "available": available,
        "note": None if available else "evaluation_annotations 정답 데이터가 없어 계산하지 않음",
        "sentence_trials": exact_total,
        "sentence_correct": exact_ok,
        "sentence_recognition_rate_pct": pct(exact_ok, exact_total),
        "keyword_trials": keyword_total,
        "keyword_correct": keyword_ok,
        "keyword_recognition_rate_pct": pct(keyword_ok, keyword_total),
        "details": details,
    }


def grasp_metrics(db, task_ids: list[str]) -> dict:
    docs = list(db.component_executions.find({"task_id": {"$in": task_ids}}))
    grouped = defaultdict(lambda: {"components": 0, "success": 0, "attempts": 0, "attempt_success": 0})
    for d in docs:
        cls = d.get("class_name") or "UNKNOWN"
        if d.get("status") == "SKIPPED":
            continue
        row = grouped[cls]
        row["components"] += 1
        row["success"] += int(d.get("status") == "SUCCESS")
        attempts = d.get("attempts") or []
        row["attempts"] += len(attempts)
        row["attempt_success"] += sum(1 for a in attempts if a.get("status") == "SUCCESS")
    result = {}
    for cls, row in sorted(grouped.items()):
        result[cls] = {
            **row,
            "component_grasp_success_rate_pct": pct(row["success"], row["components"]),
            "per_attempt_success_rate_pct": pct(row["attempt_success"], row["attempts"]),
        }
    return {"classes": result}


def system_metrics(tasks: list[dict], requested_limit: int) -> dict:
    success = sum(1 for d in tasks if d.get("status") == "SUCCESS")
    ordered_status = [d.get("status") for d in tasks]
    longest = current = 0
    for status in ordered_status:
        current = current + 1 if status == "SUCCESS" else 0
        longest = max(longest, current)
    trailing = 0
    for status in reversed(ordered_status):
        if status != "SUCCESS":
            break
        trailing += 1
    return {
        "requested_trials": requested_limit,
        "actual_completed_trials": len(tasks),
        "success_count": success,
        "failed_count": len(tasks) - success,
        "end_to_end_success_rate_pct": pct(success, len(tasks)),
        "stability": {
            "longest_consecutive_successes": longest,
            "current_trailing_successes": trailing,
        },
        "warning": None if len(tasks) == requested_limit else f"완료 작업이 {len(tasks)}건뿐이므로 {requested_limit}회 기준을 충족하지 못함",
    }


def build_report(db, limit: int) -> dict:
    tasks = select_tasks(db, limit)
    task_ids = [d.get("task_id") for d in tasks if d.get("task_id")]
    return {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "scope": {"latest_completed_tasks": limit, "task_ids": task_ids},
        "voice": voice_metrics(db, task_ids),
        "command_conversion": command_metrics(db, task_ids),
        "grasp": grasp_metrics(db, task_ids),
        "system": system_metrics(tasks, limit),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="ROS2-VLA MVP MongoDB 정량 평가")
    parser.add_argument("--limit", type=int, default=20, help="최근 완료 작업 수(기본 20)")
    parser.add_argument("--output", help="JSON 결과 저장 경로")
    args = parser.parse_args()
    if args.limit <= 0:
        parser.error("--limit는 양수여야 합니다.")
    client = None
    try:
        client, db = get_database()
        report = build_report(db, args.limit)
        text = json.dumps(report, ensure_ascii=False, indent=2, default=str)
        print(text)
        if args.output:
            Path(args.output).write_text(text + "\n", encoding="utf-8")
            print(f"\n저장 완료: {args.output}", file=sys.stderr)
        return 0
    except Exception as exc:
        print(f"평가 실패: {exc}", file=sys.stderr)
        return 1
    finally:
        if client is not None:
            client.close()


if __name__ == "__main__":
    raise SystemExit(main())
