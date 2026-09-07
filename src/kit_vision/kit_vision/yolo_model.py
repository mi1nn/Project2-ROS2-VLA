import glob
import json
import os

import cv2
import numpy as np
import torch
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO

# 추론 본체는 GPU로 돌리지만 전처리/NMS 등 CPU 쪽 연산도 있다. torch가 기본으로 이런
# 연산 하나에도 호스트 논리 코어 수만큼 스레드를 다 끌어써서(실측 10~16개) realsense USB
# 드라이버 스레드 등 다른 프로세스가 스케줄링을 못 받는 문제가 있었다. 코어 몇 개로 묶어둔다.
# ponytail: 2는 임의값 — 부족하면 3~4로.
torch.set_num_threads(2)

PACKAGE_NAME = "kit_vision"
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)
RESOURCE_DIR = os.path.join(PACKAGE_PATH, "resource")
CLASS_NAMES_FILENAME = "class_names.json"

DEFAULT_CONF_THRESHOLD = 0.5
DEFAULT_IMGSZ = 640


def _find_model_path():
    candidates = glob.glob(os.path.join(RESOURCE_DIR, "*.pt"))

    if len(candidates) != 1:
        raise FileNotFoundError(
            "resource/ 에 .pt 가중치가 정확히 1개 있어야 한다. "
            f"발견: {candidates}"
        )

    return candidates[0]


def _load_class_names():
    path = os.path.join(RESOURCE_DIR, CLASS_NAMES_FILENAME)

    with open(path, "r", encoding="utf-8") as f:
        raw = json.load(f)

    return {int(k): v for k, v in raw.items()}


def polygon_centroid(polygon):
    """
    polygon 전체 화면 mask를 만들지 않고 contour moments로 중심점을 계산한다.
    """
    contour = np.asarray(
        polygon,
        dtype=np.float32,
    ).reshape(-1, 1, 2)

    if len(contour) < 3:
        return None

    m = cv2.moments(contour)

    if m["m00"] == 0:
        return None

    return (
        int(round(m["m10"] / m["m00"])),
        int(round(m["m01"] / m["m00"])),
    )


class YoloModel:
    def __init__(self, imgsz=DEFAULT_IMGSZ):
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.use_half = self.device == "cuda"
        self.imgsz = int(imgsz)

        if self.device == "cuda":
            torch.backends.cudnn.benchmark = True

        model_path = _find_model_path()

        self.model = YOLO(model_path)
        self.model.to(self.device)
        self.class_names = _load_class_names()

        print(
            "[kit_vision] "
            f"YOLO model={model_path}, "
            f"device={self.device}, "
            f"imgsz={self.imgsz}, "
            f"fp16={self.use_half}"
        )

        # 첫 추론에서 CUDA context/kernel 초기화가 몰리는 현상을 줄인다.
        dummy = np.zeros(
            (self.imgsz, self.imgsz, 3),
            dtype=np.uint8,
        )

        try:
            with torch.inference_mode():
                self.model.predict(
                    source=dummy,
                    imgsz=self.imgsz,
                    conf=DEFAULT_CONF_THRESHOLD,
                    device=self.device,
                    half=self.use_half,
                    verbose=False,
                )
        except Exception as exc:
            print(
                "[kit_vision] "
                f"YOLO warm-up skipped: {exc}"
            )

    def infer(
        self,
        frame,
        conf_threshold=DEFAULT_CONF_THRESHOLD,
    ):
        """
        단일 프레임 YOLO segmentation 추론.

        반환:
        [
            {
                "class_name": str,
                "score": float,
                "polygon": list[float],
                "centroid_px": (x, y),
            },
            ...
        ]
        """
        with torch.inference_mode():
            result = self.model.predict(
                source=frame,
                imgsz=self.imgsz,
                conf=float(conf_threshold),
                device=self.device,
                half=self.use_half,
                verbose=False,
            )[0]

        if result.masks is None or result.boxes is None:
            return []

        instances = []

        for polygon, score, label in zip(
            result.masks.xy,
            result.boxes.conf.tolist(),
            result.boxes.cls.tolist(),
        ):
            class_name = self.class_names.get(int(label))

            if class_name is None:
                continue

            polygon_np = np.asarray(
                polygon,
                dtype=np.float32,
            )

            if polygon_np.shape[0] < 3:
                continue

            centroid = polygon_centroid(polygon_np)

            if centroid is None:
                continue

            instances.append(
                {
                    "class_name": class_name,
                    "score": float(score),
                    "polygon": (
                        polygon_np
                        .astype(float)
                        .flatten()
                        .tolist()
                    ),
                    "centroid_px": centroid,
                }
            )

        return instances
