# 07. 전체 동작 시연 테스트 시나리오

## 1. 목적과 범위

- 고정 관찰 자세에서 실제 객체 탐지·좌표 변환을 확인한다.
- MotionDemo로 파지·배치 호출 로그와 상태 전이를 확인한다.
- 명령·Component·Task 결과의 실제 DB 저장을 확인한다.
- 로봇은 움직이지 않으며 물체도 이동하지 않는다.
- 최종 검사는 관찰 화면 기준이므로 완성 키트 검사로 평가하지 않는다.
- 데모 Component SUCCESS도 실제 재고 차감 대상이다.


## 2. 테스트 방법

### 0. 공통 준비
각 호스트 터미널에서 다음을 실행한다.
```bash
cd ~/Project2-ROS2-VLA
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### 1. docker 실행
```bash
# DB만 먼저 실행. vision은 카메라 연결 후 실행
docker compose up -d postgres mongodb
docker compose ps postgres mongodb
```

### 2. db 노드 실행
```bash
set -a
source .env
set +a
ros2 run kit_db db_node
```

### 3. 카메라 연결
```bash
realsense
```

### 4. object_detection 노드 실행
ultralytics 의존성으로 인해 docker 내부에서 실행한다.
```bash
# 카메라 실행 후 비전을 켠다.
docker compose up -d vision
docker compose logs -f vision
# 내부에서 ros2 run kit_vision object_detection 명령이 지정
```

### 5. position estimation 노드 실행
```bash
ros2 run kit_robot position_estimation
```

### 6. command 노드 실행
```bash
ros2 run kit_voice get_command
```

### 7. Controller 노드 실행
```bash
ros2 run kit_robot controller --ros-args \
--params-file src/kit_robot/resource/controller.yaml \
-p restart_delay_sec:=60.0
```


### 추가. 검사용 : 결과 토픽 구독
각 별도 터미널에서 실행
```bash
ros2 topic echo /kit/command_result
ros2 topic echo /kit/component_result
    ros2 topic echo /kit/task_status
```

### DB 검사

#### 추적 항목 확인
```bash
# mongodb 접속
docker compose exec mongodb sh -c \
'mongosh --username "$MONGO_INITDB_ROOT_USERNAME" \
--password "$MONGO_INITDB_ROOT_PASSWORD" \
--authenticationDatabase admin "$MONGO_INITDB_DATABASE"'

# 최근 작업 확인
db.kit_executions.find().sort({ended_at: -1}).limit(5).pretty()

# task id 기반 확인
const taskId = "TASK-실제_작업_ID";
const taskId = "TASK-20260906T020107916126Z";

db.commands.find({task_id: taskId}).pretty()
db.component_executions.find({task_id: taskId}).pretty()
db.kit_executions.find({task_id: taskId}).pretty()

# 종료
exit
```

#### 재고 관리 확인
```bash
# postgres 접속
docker compose exec postgres sh -c \
'psql -U "$POSTGRES_USER" -d "$POSTGRES_DB"'

# 전체 재고 확인
SELECT i.item_code, v.quantity, v.updated_at
FROM item AS i
JOIN inventory AS v USING (item_id)
ORDER BY i.item_id;

# 종료
\q
```