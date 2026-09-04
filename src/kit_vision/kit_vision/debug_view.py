import cv2
import rclpy

from kit_vision.realsense import ImgNode
from kit_vision.yolo_model import YoloModel

# 디버그 전용 시각화 노드. object_detection.py는 헤드리스로 유지하고(로봇에서 디스플레이
# 없이 돌아야 하므로) 박스/마스크 확인은 이 노드로 따로 띄운다.
# 어노테이션은 ultralytics Results.plot()이 이미 해주므로 직접 그리지 않는다.


def main(args=None):
    rclpy.init(args=args)
    img_node = ImgNode()
    model = YoloModel()
    try:
        while rclpy.ok():
            img_node.spin_once(timeout_sec=0.1)
            frame = img_node.get_color_frame()
            if frame is None:
                continue

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
