#!/bin/bash
# debug_view 팝업창(X11)을 컨테이너에서 띄우려면 호스트 X 서버 접근을 열어줘야 한다.
# xhost는 호스트 X 서버 제어 명령이라 컨테이너 안(Dockerfile/entrypoint)에서는 실행할 수 없다.
set -e
xhost +local:docker
exec docker compose up "$@" vision
