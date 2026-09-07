import time

import cv2
import rclpy

from kit_vision.realsense import ImgNode
from kit_vision.yolo_model import YoloModel

# 디버그 전용 시각화 노드. object_detection.py는 헤드리스로 유지하고(로봇에서 디스플레이
# 없이 돌아야 하므로) 박스/마스크 확인은 이 노드로 따로 띄운다.
# 어노테이션은 ultralytics Results.plot()이 이미 해주므로 직접 그리지 않는다.

# object_detection.py와 같은 스로틀(3.3Hz). 없으면 프레임마다 CPU 추론을 쉬지 않고 돌려서
# (GPU 패스스루 없음 — Dockerfile 참고) 호스트 CPU를 갈아버리고, host의 realsense USB 드라이버
# 스레드가 스케줄링을 못 받아 "Incomplete video frame/Frame Corrupted"로 이어진다.
INFER_PERIOD_SEC = 0.3


def main(args=None):
    rclpy.init(args=args)
    img_node = ImgNode()
    model = YoloModel()
    next_infer_at = 0.0
    last_stamp = None
    try:
        while rclpy.ok():
            img_node.spin_once(timeout_sec=0.1)
            frame = img_node.get_color_frame()
            header = img_node.get_color_frame_header()
            if frame is None or header is None:
                continue

            # 새 프레임이 아니면(콜백이 안 갱신했으면) 같은 프레임을 다시 추론하지 않는다.
            # 카메라가 멈춘/드랍 중일 때 안 그러면 오래된 프레임에 계속 YOLO를 돌려 CPU를
            # 낭비하고, 하필 그 타이밍에 USB 드라이버 스레드를 더 굶긴다.
            stamp = (header.stamp.sec, header.stamp.nanosec)
            if stamp == last_stamp:
                continue

            now = time.monotonic()
            if now < next_infer_at:
                continue
            next_infer_at = now + INFER_PERIOD_SEC
            last_stamp = stamp

            results = model.model(frame, verbose=False)[0]
            cv2.imshow("kit_vision debug (q to quit)", results.plot())
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cv2.destroyAllWindows()
        img_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
