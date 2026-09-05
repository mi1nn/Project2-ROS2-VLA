#!/bin/bash
# debug_view 팝업창(X11)을 컨테이너에서 띄우려면 호스트 X 서버 접근을 열어줘야 한다.
# xhost는 호스트 X 서버 제어 명령이라 컨테이너 안(Dockerfile/entrypoint)에서는 실행할 수 없다.
set -e
xhost +local:docker
# vision은 이름으로 같이 넘기지 않는다 — depends_on으로 필요시에만 딸려 뜨게 해서,
# 이 스크립트를 Ctrl+C로 끝내도 이미 떠 있던 vision(object_detection)까지 죽지 않는다.
exec docker compose up "$@" vision-debug
