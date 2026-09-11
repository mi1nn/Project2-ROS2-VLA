import json
from dataclasses import dataclass, field
from datetime import datetime


@dataclass
class Attempt:
    attempt_no: int
    started_at: datetime
    ended_at: datetime | None = None
    status: str = "RUNNING"
    failed_stage: str = ""
    error_code: str = ""
    detail: str = ""


@dataclass
class Component:
    name: str
    index: int
    slot: str
    status: str = "PENDING"
    attempts: list[Attempt] = field(default_factory=list)
    error_code: str = ""
    detail: str = ""
    started_at: datetime | None = None
    ended_at: datetime | None = None

    @property
    def attempt_count(self) -> int:
        return len(self.attempts)


def validate_command(command_json: str, supported_names: set[str]) -> dict:
    try:
        command = json.loads(command_json)
    except (TypeError, ValueError) as error:
        raise ValueError("올바른 JSON 명령이 아닙니다.") from error

    if not isinstance(command, dict):
        raise ValueError("명령은 JSON 객체여야 합니다.")

    kit_type = command.get("kit_type")
    if not isinstance(kit_type, str) or not kit_type.strip():
        raise ValueError("kit_type은 비어 있지 않은 문자열이어야 합니다.")

    items = command.get("items")
    if not isinstance(items, list) or not items:
        raise ValueError("items는 비어 있지 않은 목록이어야 합니다.")

    validated_items = []

    for item in items:
        if not isinstance(item, dict):
            raise ValueError("각 품목은 JSON 객체여야 합니다.")

        name = item.get("name")
        quantity = item.get("qty")

        if not isinstance(name, str) or name not in supported_names:
            raise ValueError(f"지원하지 않는 품목: {name!r}")

        if type(quantity) is not int or quantity < 1:
            raise ValueError(f"{name}: 수량은 1 이상 정수여야 합니다.")

        validated_items.append({"name": name, "qty": quantity})

    return {
        "kit_type": kit_type.strip(),
        "items": validated_items,
    }


def build_components(
    command: dict,
    slot_names: list[str],
) -> tuple[list[Component], dict[str, int]]:
    if any(not isinstance(slot, str) or not slot.strip() for slot in slot_names):
        raise ValueError("슬롯 이름은 비어 있지 않은 문자열이어야 합니다.")

    if len(slot_names) != len(set(slot_names)):
        raise ValueError("슬롯 이름이 중복되었습니다.")

    expected_counts: dict[str, int] = {}
    for item in command["items"]:
        name = item["name"]
        expected_counts[name] = expected_counts.get(name, 0) + item["qty"]

    total = sum(expected_counts.values())
    if total > len(slot_names):
        raise ValueError(f"배치 공간 부족: 요청 {total}개, 슬롯 {len(slot_names)}개")

    components = []
    for name, quantity in expected_counts.items():
        for _ in range(quantity):
            index = len(components)
            components.append(
                Component(
                    name=name,
                    index=index,
                    slot=slot_names[index],
                )
            )

    return components, expected_counts