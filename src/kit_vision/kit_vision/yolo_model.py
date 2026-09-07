import glob
import json
import os

import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO

PACKAGE_NAME = "kit_vision"
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)
RESOURCE_DIR = os.path.join(PACKAGE_PATH, "resource")

CLASS_NAMES_FILENAME = "class_names.json"
DEFAULT_CONF_THRESHOLD = 0.5


def _find_model_path():
    # 가중치 파일명은 팀원 학습 산출물이라 고정하지 않는다. resource/ 안의 .pt 하나를 찾는다.
    candidates = glob.glob(os.path.join(RESOURCE_DIR, "*.pt"))
    if len(candidates) != 1:
        raise FileNotFoundError(
            f"resource/ 에 .pt 가중치가 정확히 1개 있어야 한다. 발견: {candidates}"
        )
    return candidates[0]


def _load_class_names():
    path = os.path.join(RESOURCE_DIR, CLASS_NAMES_FILENAME)
    with open(path, "r", encoding="utf-8") as f:
        raw = json.load(f)
    return {int(k): v for k, v in raw.items()}


def polygon_to_mask(polygon, shape):
    """polygon: (N,2) 픽셀 좌표. shape: (h, w). 반환: uint8 마스크 (0/1)."""
    mask = np.zeros(shape, dtype=np.uint8)
    if len(polygon) < 3:
        return mask
    cv2.fillPoly(mask, [np.asarray(polygon, dtype=np.int32)], 1)
    return mask


def mask_centroid(mask):
    """mask 무게중심 픽셀 (x, y). 빈 마스크면 None."""
    m = cv2.moments(mask, binaryImage=True)
    if m["m00"] == 0:
        return None
    return (int(round(m["m10"] / m["m00"])), int(round(m["m01"] / m["m00"])))


class YoloModel:
    def __init__(self):
        self.model = YOLO(_find_model_path())
        self.class_names = _load_class_names()

    def infer(self, frame, conf_threshold=DEFAULT_CONF_THRESHOLD):
        """단일 프레임 seg 추론. 컨트롤러가 레시피로 고르므로 전 클래스를 담는다.

        반환: [{"class_name": str, "score": float, "mask": np.ndarray(h,w),
                "polygon": list[float] (x1,y1,x2,y2,...), "centroid_px": (x, y)}, ...]
        """
        results = self.model(frame, verbose=False)[0]
        if results.masks is None:
            return []

        h, w = frame.shape[:2]
        instances = []
        for polygon, score, label in zip(
            results.masks.xy,
            results.boxes.conf.tolist(),
            results.boxes.cls.tolist(),
        ):
            if score < conf_threshold:
                continue   # score 낮을 때 
            class_name = self.class_names.get(int(label))
            if class_name is None:   # class이름 없을 떄 
                continue

            mask = polygon_to_mask(polygon, (h, w))
            centroid = mask_centroid(mask)
            if centroid is None:
                continue

            instances.append({
                "class_name": class_name,
                "score": float(score),
                "mask": mask,
                "polygon": np.asarray(polygon, dtype=float).flatten().tolist(),
                "centroid_px": centroid,
            })
        return instances


def _demo():
    # 모델/가중치 없이도 도는 순수 기하 self-check.
    square = [(10, 10), (10, 30), (30, 30), (30, 10)]
    mask = polygon_to_mask(square, (40, 40))
    assert mask.sum() > 0
    cx, cy = mask_centroid(mask)
    assert abs(cx - 20) <= 1 and abs(cy - 20) <= 1, (cx, cy)

    empty = np.zeros((40, 40), dtype=np.uint8)
    assert mask_centroid(empty) is None
    print("ok")


if __name__ == "__main__":
    _demo()
