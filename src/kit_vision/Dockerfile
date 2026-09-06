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
#
# torch를 먼저 CPU 전용 wheel로 깔아둔다. 안 그러면 ultralytics가 끌어오는 기본 torch가
# CUDA용이라 nvidia-cudnn/cublas/nccl 등으로 이미지가 수 GB 불어난다 — 이 컨테이너엔
# GPU 패스스루(--gpus, nvidia-container-toolkit) 설정이 없어서 그 무게가 전부 낭비다.
RUN pip install --no-cache-dir --break-system-packages \
        torch --index-url https://download.pytorch.org/whl/cpu
# ros-jazzy-cv-bridge가 apt(python3-numpy)로 numpy를 이미 깔아놓는데, dpkg 설치본은
# pip 메타데이터(RECORD)가 없어 pip가 업그레이드하려다 "Cannot uninstall"로 죽는다.
# numpy만 --ignore-installed로 따로 갈아치운다 — 이 플래그를 ultralytics 설치에 같이 쓰면
# 방금 깐 CPU torch까지 "설치 안 된 것" 취급해서 CUDA torch를 통째로 재다운로드해버린다.
RUN pip install --no-cache-dir --break-system-packages --ignore-installed numpy
# cv_bridge_boost.so는 apt 빌드 시점의 numpy 1.x ABI로 컴파일돼 있어서, numpy 2.x로 올리면
# import는 되지만 실제 이미지 변환(imgmsg_to_cv2) 호출 시 세그폴트(exit -11)로 죽는다.
# ultralytics/opencv-python 최신판은 numpy2를 끌어오므로, 이 pip 호출에 numpy<2를 같이 줘서
# 한 번의 resolve로 numpy1과 맞는 opencv-python(4.x)이 선택되게 한다 — 따로 pip install을
# 나누면 뒤 단계가 numpy를 다시 2.x로 끌어올려버린다(직접 검증함).
RUN pip install --no-cache-dir --break-system-packages "numpy<2" ultralytics opencv-python

WORKDIR /ws
COPY src/kit_interfaces src/kit_interfaces
COPY src/kit_vision src/kit_vision

RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install --packages-select kit_interfaces kit_vision

COPY src/kit_vision/docker-entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "kit_vision", "vision.launch.py"]
