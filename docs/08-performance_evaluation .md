# 1차 성능평가 결과

# 1. 평가 개요

음성 명령 입력부터 키트 최종 검사까지의 공정을 반복 수행하고 MongoDB 원본 데이터를 Notebook([anlyze_metrics.ipynb](../analyze/anlyze_metrics.ipynb))에서 분석했다.

전체 End-to-End 성공 사례는 없었으므로 최종 결과는 0/20, 0.00%로 명시한다. 그리퍼의 DB 판정 로직이 초기 단계에서 부정확했기 때문에 파지 관련 세부 지표는 DB의 `status`가 아니라 영상·현장 확인 결과를 `actual_grasp_success`에 수동 기록하여 계산한다.

## 분석 원칙

- End-to-End 성공과 단계별·품목별 부분 성공을 구분한다.
- 분모와 제외 조건을 각 지표에 명시한다.
- 세 컬렉션은 `task_id`를 기준으로 연결한다.
- wakeword timeout, invalid command 등 실제 키팅이 시작되지 않은 작업은 파지·키팅 지표에서 제외한다.
- `SKIPPED` Component는 실제 파지 시도가 없으므로 제외한다.
- DB의 그리퍼 성공 판정은 참고용이며, 실제 파지 결과는 `actual_grasp_success`를 사용한다.

# 2. 평가 환경 및 조건

| 항목 | 내용 |
| --- | --- |
| 평가 일자 | 2026-09-08 |
| 계획 시행 횟수 | 20회 |
| DB 후보 작업 | 22건 (키워드 오인식 2건 포함) |
| 대상 시스템 | 음성·LLM·Vision AI 기반 재난 구조키트 자동 구성 시스템 |
| 로봇 및 장비 | Doosan M0609, RG2 Gripper, RGB-D Camera |
| 분석 도구 | Python, pandas, PyMongo, Jupyter Notebook |
| 분석 대상 DB | `commands`, `component_executions`, `kit_executions` |

# 3. 데이터 구성 및 전처리

## 3.1 사용 데이터

| 컬렉션 | 주요 필드 | 분석 목적 |
| --- | --- | --- |
| commands | `task_id`, `raw_text`, `command`, `validation`, `created_at` | 음성 명령 및 구조화 JSON |
| component_executions | `task_id`, `class_name`, `status`, `attempts`, `started_at`, `ended_at` | 품목별 파지·재시도 원본 |
| kit_executions | `task_id`, `status`, `final_inspection`, `started_at`, `ended_at` | 공정 상태와 최종 검사 |

## 3.2 전처리 및 수동 라벨

- `validation.success == True`이고 실제 Component가 존재하는 작업을 실행 후보로 분류한다.
- wakeword timeout과 invalid command는 실행 후보에서 제외한다.
- `grasp_review.csv`의 `actual_grasp_success`는 실제 물체를 목표 위치까지 옮겼으면 `True`, 그렇지 않으면 `False`로 입력한다.
- 파지 결과를 확인하지 못한 빈 값은 성공·실패 분모에서 제외한다.
- 현재 22개 실행 후보에서 116개 Component를 분석했다.

# 4. 평가 지표 및 산식

| 구분 | 지표 | 산식 또는 판정 기준 |
| --- | --- | --- |
| 전체 | End-to-End 성공률 | 실제 키팅 완료이면서 최종 검사 `PASS`인 작업 ÷ 분석 시행 수 |
| 키팅 | 키팅 완료율 | 모든 필수 Component가 실제 성공한 작업 ÷ 분석 시행 수 |
| 조작 | 클래스별 파지 성공률 | 해당 클래스 `actual_grasp_success=True` ÷ 해당 클래스 라벨 입력 시도 |
| 조작 | 품목 작업 성공률 | 실제 이송 성공 Component ÷ 실제 파지 결과가 라벨된 Component |
| 검사 | 누락률 | `expected_counts - actual_counts`의 양수 합 ÷ 전체 기대 수량 |
| 검사 | 검사 수행률 | `final_inspection` 기록 작업 ÷ 분석 시행 수 |
| 검사 | 검사 판정률 | `PASS` 또는 `FAIL` 판정 작업 ÷ 검사 수행 작업 |
| 구성 | 구성 완료율 | 검사에서 확인된 실제 수량 ÷ 전체 기대 수량 |

현재 파지 라벨은 최종 이송 여부를 기준으로 하므로, 파지 후 떨어뜨린 경우 `False`로 처리한다.

# 5. 분석 결과

## 5.1 핵심 결과 요약

현재 Notebook 저장 출력 기준:

| 지표 | 성공 또는 충족 건수 | 분모 | 결과 |
| --- | ---: | ---: | ---: |
| End-to-End 성공률 | 0 | 22 | 0.00% |
| 키팅 완료율 | 3 | 22 | 13.64% |
| 품목 작업 성공률 | 84 | 116 | 72.41% |
| 누락률 | 38 | 112 | 33.93% |
| 검사 수행률 | 19 | 22 | 86.36% |
| 검사 판정률 | 18 | 19 | 94.74% |
| 구성 완료율 | 78 | 112 | 69.64% |

## 5.2 명령 키워드 및 JSON 변환

| 항목 | 결과 |
| --- | ---: |
| 키워드 추출 | 20/22 (90.91%) |
| 응급키트 1번 | 5건 |
| 응급키트 2번 | 5건 |
| 취사키트 1번 | 5건 |
| 취사키트 2번 | 5건 |
| JSON 변환 성공 항목 중 유효 JSON | 22/22 (100.00%) |

키워드 미검출 사례는 `응급기트 2번`, `시사 킬투 2번`이다. 두 작업은 명령 JSON은 생성되었지만 `raw_text`에 기준 키워드가 정확히 포함되지 않았다.

## 5.3 클래스별 실제 파지 성공률

| 품목 | 성공 | 라벨 입력 시도 | 실제 성공률 |
| --- | ---: | ---: | ---: |
| 마스크 | 1 | 6 | 16.67% |
| 분유 | 11 | 14 | 78.57% |
| 샴푸리필 | 6 | 7 | 85.71% |
| 수세미 | 13 | 16 | 81.25% |
| 양갱 | 14 | 20 | 70.00% |
| 여행용티슈 | 6 | 9 | 66.67% |
| 일회용숟가락 | 12 | 14 | 85.71% |
| 컵라면 | 8 | 15 | 53.33% |
| 햄 | 13 | 15 | 86.67% |

이 표는 DB의 `status`가 아닌 `grasp_review.csv`의 실제 관찰 라벨을 기준으로 한다.

# 6. 결과 해석

- End-to-End 성공은 0건이다.
- 키팅 완료 3건과 End-to-End 0건은 모순되지 않는다. 키팅 완료 후 최종 검사 `PASS`까지 통과한 작업이 없으면 End-to-End는 0건이다.
- 현재 Notebook 기준 검사 수행률 19/22는 검사 미수행 3건을 뜻한다.
- 검사 판정률 18/19는 검사 수행 19건 중 `PASS` 또는 `FAIL` 판정이 18건이라는 뜻이며, 검사 실패가 1건이라는 의미가 아니다.
- `inspection_mismatch`, `inspection_response_invalid`, `recovery_failed`는 Task 실패 원인이고, `final_inspection.result`의 `FAIL`과는 별도 필드로 구분해야 한다.
- 클래스별로는 마스크와 컵라면의 실제 성공률이 낮게 나타났다.
- 파지 후 낙하·배치 실패는 최종 품목 작업 성공률에는 실패로 반영된다.

# 7. 결론 및 개선 계획

## 7.1 결론

- End-to-End 성공률: 0/22, 0.00%
- 키팅 완료율: 3/22, 13.64%
- 품목 작업 성공률: 84/116, 72.41%
- 누락률: 38/112, 33.93%
- 검사 수행률: 19/22, 86.36%
- 검사 판정률: 18/19, 94.74%
- 구성 완료율: 78/112, 69.64%
- 주요 병목 후보: 실제 파지·이송, 최종 검사 일치성
- 평가 신뢰성 이슈: DB 그리퍼 판정 부정확

## 7.2 개선 과제

- [ ] 그리퍼의 파지·배치 판정을 실제 물체 이동 여부와 일치하도록 수정
- [ ] `final_inspection.result`와 Task 실패 원인을 별도 집계
- [ ] 키팅 완료와 최종 검사 PASS를 End-to-End 판정에 일관되게 적용

# 8. 산출물 및 재현 방법

- 분석 Notebook: [anlyze_metrics.ipynb](../analyze/anlyze_metrics.ipynb)
- 수동 파지 검토: [grasp_review.csv](../analyze/grasp_review.csv)
- MongoDB 컬렉션: `commands`, `component_executions`, `kit_executions`

Notebook 실행 순서:

1. MongoDB 연결
2. 평가 대상 `selected` 20건 확정
3. `grasp_review.csv`의 `actual_grasp_success` 입력
4. 명령·키워드 지표 계산
5. 셀 33~36 실행
6. 클래스별 파지 안정성 셀 실행
7. 본 문서의 표에 결과 반영

# 9. 분석 한계

- 전체 성공 사례가 없어 End-to-End 개선 정도를 직접 비교할 수 없다.
- 그리퍼 DB 판정 로직이 부정확하여 DB `status`를 실제 파지 성공의 근거로 사용할 수 없다.
- 수동 라벨이 입력된 Component만 실제 파지 지표의 분모에 포함된다.
- 최종 검사 결과가 없는 작업은 검사 실패가 아니라 미수행·판정 불가로 구분한다.

# 10. 참고

- 분석 기준 및 원본 스키마: [docs/05-database.md](05-database.md)
- 테스트 시나리오: [docs/07-test-scenario.md](07-test-scenario.md)
