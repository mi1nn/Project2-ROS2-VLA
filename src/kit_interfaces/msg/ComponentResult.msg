# Component 하나의 최종 작업 결과와 그 과정에서 발생한 재시도를 저장

string task_id

int32 component_index
int32 component_total
string component
string slot

string status
int32 attempt_count
string attempts_json

string error_code
string detail

builtin_interfaces/Time started_at
builtin_interfaces/Time ended_at