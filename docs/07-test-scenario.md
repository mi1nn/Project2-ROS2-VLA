# 07. 전체 동작 시연 테스트 시나리오

## 1. 목적과 범위

- `object_detection`의 실제 객체 탐지와 `position_estimation`의 좌표 변환을 확인한다.
- 실제 `Motion`으로 M0609·RG2의 관찰 이동, 파지, 배치, 검사 이동과 복귀를 확인한다.
- 명령·Component·Task 결과의 DB 저장과 `SUCCESS` Component의 재고 차감을 확인한다.
- 이 시나리오는 로봇 없는 데모가 아니며 로봇과 그리퍼가 실제로 움직이고 물체가 이동한다.
- 최종 검사는 `inspection_pose`에서 보이는 검출 전체를 계산한다. 현재 ROI 필터가 없으므로 검사 화면에는 완성 트레이의 검사 대상 물체만 보여야 한다.
- 실행 전에 작업영역, 실제 슬롯 좌표, RG2 연결, 비상 정지 상태와 저속 운전을 반드시 확인한다.


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

최초 실행이면 [05 데이터베이스](05-database.md) 8절에 따라 루트 `.env`를 만들고 DB 접속값을 설정한다. 음성 노드의 `OPENAI_API_KEY`는 [06 Controller 가이드](06-controller-guide.md) 4절에 따라 설치 리소스에 반영한다.

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

### 3. RealSense 카메라 드라이버 실행

```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

`object_detection`은 정렬된 depth 토픽(`/camera/aligned_depth_to_color/image_raw`)을 구독하므로 `align_depth.enable:=true`를 반드시 적용한다.

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

### 7. Doosan 로봇 드라이버 실행

아래의 로봇 IP와 DSR 워크스페이스 경로가 실기 환경과 다르면 실제 값으로 바꿔 실행한다.

```bash
source ~/ws_cobot_pjt/ws_dsr/install/setup.bash
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
  name:=dsr01 \
  host:=192.168.1.100 \
  mode:=real \
  model:=m0609 \
  gui:=false
```

Controller를 실행하기 전에 DSR 서비스가 준비됐는지 확인한다.

```bash
ros2 service list | grep '/dsr01/dsr_controller2/'
```

실제 `Motion`은 RG2에도 직접 접속한다. `motion.py`에 설정된 RG2 접속값(현재 `192.168.1.1:502`)과 `src/kit_robot/config/motion.yaml`의 로봇 자세·슬롯 좌표가 실기 환경에 맞는지 확인한다.

### 8. 검사용 결과 토픽 구독

각 명령은 별도 터미널에서 실행한다.

```bash
ros2 topic echo /kit/command_result
```

```bash
ros2 topic echo /kit/component_result
```

```bash
ros2 topic echo /kit/task_status
```

### 9. Controller 노드 실행

아래 명령은 실제 `Motion`을 초기화하며, 작업이 시작되면 로봇과 그리퍼가 실제로 움직인다. 작업영역과 비상 정지 상태를 확인한 뒤 실행한다.

```bash
source ~/ws_cobot_pjt/ws_dsr/install/setup.bash
source install/setup.bash
ros2 run kit_robot controller --ros-args \
  --params-file src/kit_robot/resource/controller.yaml \
  -p restart_delay_sec:=60.0
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
