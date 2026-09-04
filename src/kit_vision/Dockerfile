# 호스트 우분투에서 ultralytics가 깨져서 kit_vision(YOLO 추론)만 격리 실행하는 이미지.
# realsense2_camera 드라이버는 호스트에 그대로 둔다 — USB 패스스루가 필요 없고,
# --network host 로 호스트의 카메라 토픽을 그대로 구독한다.
FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-colcon-common-extensions \
        ros-jazzy-cv-bridge \
        ros-jazzy-rosidl-default-generators \
        ros-jazzy-launch-ros \
    && rm -rf /var/lib/apt/lists/*

# Ubuntu 24.04(Noble) 이미지는 PEP 668 externally-managed 환경이라 시스템 pip 설치엔
# --break-system-packages 가 필요하다. 컨테이너 전용 이미지라 venv 로 나눌 이유가 없다.
RUN pip install --no-cache-dir --break-system-packages ultralytics opencv-python

WORKDIR /ws
COPY src/kit_interfaces src/kit_interfaces
COPY src/kit_vision src/kit_vision

RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install --packages-select kit_interfaces kit_vision

COPY src/kit_vision/docker-entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "kit_vision", "vision.launch.py"]
