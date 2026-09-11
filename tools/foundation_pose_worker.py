#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
FoundationPose socket worker.

Run INSIDE the existing FoundationPose Docker container / conda env.
This file does NOT use ROS2 and does NOT open RealSense.

Input from ROS2 host over TCP:
  1) request JSON
  2) BGR image (uint8 HxWx3)
  3) aligned depth in meters (float32 HxW)
  4) color camera K (float64 3x3)

Processing:
  YOLO segmentation -> target mask -> FoundationPose register()

Output:
  success/failure JSON
  on success: 4x4 T_camera_object matrix (float64)
"""

import os
import math
import json
import socket
import struct
import shutil
import logging
import traceback

import cv2
import numpy as np
import torch
import trimesh

from ultralytics import YOLO

from estimater import *
from datareader import *
import nvdiffrast.torch as dr


# ============================================================
# Configuration copied from the already-working demo
# ============================================================

YOLO_MODEL_PATH = "/home/rokey/FoundationPose/models/best.pt"

TARGET_CLASS_ID = 7

MESH_FILES = {
    4: "/home/rokey/FoundationPose/models/YANGGANG.ply",
    7: "/home/rokey/FoundationPose/models/RAMYEON.ply",
    8: "/home/rokey/FoundationPose/models/RICHAM.ply",
}

MESH_SCALE = 1.0

CONF = 0.5
IMGSZ = 640
YOLO_DEVICE = 0

EST_REFINE_ITER = 5
MIN_MASK_PIXELS = 100
MIN_VALID_DEPTH_PIXELS = 100

DEBUG = 1
DEBUG_DIR = "/home/rokey/FoundationPose/debug_socket"

SOCKET_HOST = "0.0.0.0"
SOCKET_PORT = 5555
SOCKET_BACKLOG = 4
SOCKET_TIMEOUT_SEC = 30.0

# Complete object point cloud sent to GraspGenX.
# Sample once in object/mesh coordinates and transform by each FP pose.
COMPLETE_OBJECT_PC_POINTS = 8192


# ============================================================
# Socket protocol helpers
# ============================================================

def recv_exact(sock, nbytes):
    chunks = []
    remaining = nbytes

    while remaining > 0:
        chunk = sock.recv(remaining)
        if not chunk:
            raise ConnectionError(
                f"Socket closed while receiving {nbytes} bytes."
            )

        chunks.append(chunk)
        remaining -= len(chunk)

    return b"".join(chunks)


def send_json(sock, obj):
    payload = json.dumps(
        obj,
        ensure_ascii=False,
        separators=(",", ":"),
    ).encode("utf-8")

    sock.sendall(struct.pack("!I", len(payload)))
    sock.sendall(payload)


def recv_json(sock):
    length = struct.unpack("!I", recv_exact(sock, 4))[0]
    payload = recv_exact(sock, length)
    return json.loads(payload.decode("utf-8"))


def send_array(sock, array):
    arr = np.ascontiguousarray(array)

    send_json(
        sock,
        {
            "dtype": arr.dtype.str,
            "shape": list(arr.shape),
            "nbytes": int(arr.nbytes),
        },
    )

    sock.sendall(arr.tobytes(order="C"))


def recv_array(sock):
    meta = recv_json(sock)

    dtype = np.dtype(meta["dtype"])
    shape = tuple(int(v) for v in meta["shape"])
    nbytes = int(meta["nbytes"])

    raw = recv_exact(sock, nbytes)

    arr = np.frombuffer(raw, dtype=dtype)

    expected_elements = int(np.prod(shape))
    if arr.size != expected_elements:
        raise ValueError(
            f"Array size mismatch: expected {expected_elements}, got {arr.size}"
        )

    return arr.reshape(shape).copy()


# ============================================================
# Existing demo helper logic
# ============================================================

def rotation_matrix_to_rpy_deg(R):
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6

    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0

    return tuple(np.degrees([roll, pitch, yaw]))


def polygon_to_mask(polygon, height, width):
    mask = np.zeros((height, width), dtype=np.uint8)
    polygon = np.asarray(polygon, dtype=np.float32)

    if polygon.ndim != 2 or polygon.shape[0] < 3:
        return mask.astype(bool)

    polygon = np.round(polygon).astype(np.int32)
    polygon[:, 0] = np.clip(polygon[:, 0], 0, width - 1)
    polygon[:, 1] = np.clip(polygon[:, 1], 0, height - 1)

    cv2.fillPoly(mask, [polygon], 1)
    return mask.astype(bool)


def load_mesh(mesh_file, mesh_scale):
    if not os.path.isfile(mesh_file):
        raise FileNotFoundError(f"Mesh file not found: {mesh_file}")

    loaded = trimesh.load(mesh_file)

    if isinstance(loaded, trimesh.Scene):
        try:
            mesh = loaded.dump(concatenate=True)
        except Exception as e:
            raise RuntimeError(
                "Mesh가 Scene으로 로드되었습니다. OBJ/PLY 단일 mesh 사용을 권장합니다."
            ) from e
    else:
        mesh = loaded

    if not hasattr(mesh, "vertices") or len(mesh.vertices) == 0:
        raise RuntimeError("유효한 mesh vertex를 읽지 못했습니다.")

    if mesh_scale != 1.0:
        mesh.apply_scale(mesh_scale)

    return mesh


def init_foundationpose(mesh):
    if os.path.isdir(DEBUG_DIR):
        shutil.rmtree(DEBUG_DIR)

    os.makedirs(os.path.join(DEBUG_DIR, "track_vis"), exist_ok=True)
    os.makedirs(os.path.join(DEBUG_DIR, "ob_in_cam"), exist_ok=True)

    to_origin, extents = trimesh.bounds.oriented_bounds(mesh)
    bbox = np.stack(
        [-extents / 2.0, extents / 2.0],
        axis=0,
    ).reshape(2, 3)

    scorer = ScorePredictor()
    refiner = PoseRefinePredictor()
    glctx = dr.RasterizeCudaContext()

    estimator = FoundationPose(
        model_pts=mesh.vertices,
        model_normals=mesh.vertex_normals,
        mesh=mesh,
        scorer=scorer,
        refiner=refiner,
        debug_dir=DEBUG_DIR,
        debug=DEBUG,
        glctx=glctx,
    )

    return estimator, to_origin, bbox


def get_best_target_detection(result, target_class_id):
    if result.boxes is None or len(result.boxes) == 0:
        return None

    if result.masks is None:
        return None

    candidates = []

    for i, box in enumerate(result.boxes):
        class_id = int(box.cls[0])

        if class_id != target_class_id:
            continue

        if i >= len(result.masks.xy):
            continue

        polygon = result.masks.xy[i]

        if polygon is None or len(polygon) < 3:
            continue

        confidence = float(box.conf[0])
        x1, y1, x2, y2 = (
            box.xyxy[0]
            .detach()
            .cpu()
            .numpy()
            .tolist()
        )

        candidates.append(
            {
                "class_id": class_id,
                "class_name": result.names[class_id],
                "confidence": confidence,
                "polygon": polygon,
                "bbox": (x1, y1, x2, y2),
            }
        )

    if not candidates:
        return None

    return max(
        candidates,
        key=lambda x: x["confidence"],
    )


# ============================================================
# FoundationPose worker
# ============================================================

class FoundationPoseWorker:

    def __init__(self):
        set_logging_format()
        set_seed(0)

        print("=" * 60)
        print(" FoundationPose socket worker")
        print("=" * 60)
        print(f"YOLO model   : {YOLO_MODEL_PATH}")
        print(f"Target class : {TARGET_CLASS_ID}")
        print(f"Socket       : {SOCKET_HOST}:{SOCKET_PORT}")

        if not os.path.isfile(YOLO_MODEL_PATH):
            raise FileNotFoundError(
                f"YOLO model not found: {YOLO_MODEL_PATH}"
            )

        mesh_file = MESH_FILES.get(TARGET_CLASS_ID)
        if mesh_file is None:
            raise ValueError(
                f"No mesh configured for class {TARGET_CLASS_ID}"
            )

        # Load ONCE and keep in GPU/RAM.
        self.yolo = YOLO(YOLO_MODEL_PATH)

        if TARGET_CLASS_ID not in self.yolo.names:
            raise ValueError(
                f"TARGET_CLASS_ID={TARGET_CLASS_ID} not in "
                f"YOLO classes: {self.yolo.names}"
            )

        self.target_class_name = self.yolo.names[TARGET_CLASS_ID]

        print(
            f"Target       : id={TARGET_CLASS_ID}, "
            f"name={self.target_class_name}"
        )
        print(f"Mesh         : {mesh_file}")

        self.mesh = load_mesh(
            mesh_file,
            MESH_SCALE,
        )

        self.estimator, self.to_origin, self.bbox = (
            init_foundationpose(self.mesh)
        )

        # Sample the mesh ONCE in original object coordinates.
        # On each request we only transform these points by T_camera_object.
        sampled_points, _ = trimesh.sample.sample_surface(
            self.mesh,
            int(COMPLETE_OBJECT_PC_POINTS),
        )
        self.object_pc_mesh = np.asarray(
            sampled_points,
            dtype=np.float32,
        ).reshape(-1, 3)

        print(
            f"Complete object PC: {len(self.object_pc_mesh)} mesh points"
        )
        print("FoundationPose initialized.")
        print("=" * 60)

    def estimate(self, color_bgr, depth_m, K):
        color_bgr = np.asarray(
            color_bgr,
            dtype=np.uint8,
        )

        depth_m = np.asarray(
            depth_m,
            dtype=np.float32,
        )

        K = np.asarray(
            K,
            dtype=np.float64,
        ).reshape(3, 3)

        if color_bgr.ndim != 3 or color_bgr.shape[2] != 3:
            raise ValueError(
                f"Invalid RGB image shape: {color_bgr.shape}"
            )

        if depth_m.ndim != 2:
            raise ValueError(
                f"Invalid depth shape: {depth_m.shape}"
            )

        if color_bgr.shape[:2] != depth_m.shape:
            raise ValueError(
                "RGB/depth size mismatch: "
                f"rgb={color_bgr.shape[:2]}, "
                f"depth={depth_m.shape}"
            )

        depth_m = depth_m.copy()
        depth_m[
            (~np.isfinite(depth_m)) |
            (depth_m < 0.001)
        ] = 0.0

        # --------------------------------------------
        # YOLO one-shot segmentation
        # --------------------------------------------
        results = self.yolo.predict(
            source=color_bgr,
            imgsz=IMGSZ,
            conf=CONF,
            device=YOLO_DEVICE,
            verbose=False,
        )

        result = results[0]

        detection_count = (
            len(result.boxes)
            if result.boxes is not None
            else 0
        )

        target = get_best_target_detection(
            result,
            TARGET_CLASS_ID,
        )

        if target is None:
            self._save_detection_debug(result, None, None, None)

            # Release temporary YOLO inference tensors.
            del results
            del result
            if torch.cuda.is_available():
                torch.cuda.empty_cache()

            return {
                "success": False,
                "error": "TARGET_NOT_DETECTED",
                "message": (
                    f"Target class {TARGET_CLASS_ID} "
                    f"({self.target_class_name}) not detected."
                ),
                "detection_count": detection_count,
            }, None, None

        # Keep a CPU copy only for later visualization, then release
        # YOLO's temporary GPU tensors before FoundationPose register().
        result_for_debug = result.cpu()

        del results
        del result

        if torch.cuda.is_available():
            torch.cuda.empty_cache()

        h, w = color_bgr.shape[:2]

        mask = polygon_to_mask(
            target["polygon"],
            h,
            w,
        )

        mask_pixels = int(np.count_nonzero(mask))
        valid_depth_pixels = int(
            np.count_nonzero(
                mask & (depth_m > 0)
            )
        )

        if mask_pixels < MIN_MASK_PIXELS:
            return {
                "success": False,
                "error": "MASK_TOO_SMALL",
                "message": (
                    f"Mask too small: {mask_pixels} px"
                ),
                "mask_pixels": mask_pixels,
                "valid_depth_pixels": valid_depth_pixels,
            }, None, None

        if valid_depth_pixels < MIN_VALID_DEPTH_PIXELS:
            return {
                "success": False,
                "error": "NOT_ENOUGH_DEPTH",
                "message": (
                    "Not enough valid depth in mask: "
                    f"{valid_depth_pixels} px"
                ),
                "mask_pixels": mask_pixels,
                "valid_depth_pixels": valid_depth_pixels,
            }, None, None

        print(
            "[REGISTER] "
            f"{target['class_name']} "
            f"conf={target['confidence']:.3f} "
            f"mask={mask_pixels}px "
            f"valid_depth={valid_depth_pixels}px"
        )

        color_rgb = cv2.cvtColor(
            color_bgr,
            cv2.COLOR_BGR2RGB,
        )

        # --------------------------------------------
        # Existing FoundationPose register() logic
        # --------------------------------------------
        current_pose = self.estimator.register(
            K=K,
            rgb=color_rgb,
            depth=depth_m,
            ob_mask=mask,
            iteration=EST_REFINE_ITER,
        )

        if torch.is_tensor(current_pose):
            current_pose = (
                current_pose
                .detach()
                .cpu()
                .numpy()
            )

        current_pose = np.asarray(
            current_pose,
            dtype=np.float64,
        ).reshape(4, 4)

        if not np.all(np.isfinite(current_pose)):
            raise RuntimeError(
                "FoundationPose returned NaN/Inf."
            )

        tx, ty, tz = current_pose[:3, 3]

        roll, pitch, yaw = (
            rotation_matrix_to_rpy_deg(
                current_pose[:3, :3]
            )
        )

        os.makedirs(
            DEBUG_DIR,
            exist_ok=True,
        )

        np.savetxt(
            os.path.join(
                DEBUG_DIR,
                "latest_pose.txt",
            ),
            current_pose,
            fmt="%.9f",
        )

        self._save_detection_debug(
            result_for_debug,
            current_pose,
            K,
            color_bgr,
        )

        # Release temporary CUDA allocations from this request while
        # keeping the loaded YOLO/FoundationPose models resident.
        if torch.cuda.is_available():
            torch.cuda.empty_cache()

        # Complete object point cloud in camera optical frame.
        # FoundationPose current_pose is T_camera_object for the ORIGINAL
        # mesh coordinates, so applying it to the sampled mesh points puts
        # the complete object cloud directly in camera coordinates.
        R_cam_obj = current_pose[:3, :3]
        t_cam_obj = current_pose[:3, 3]

        complete_object_pc = (
            self.object_pc_mesh.astype(np.float64)
            @ R_cam_obj.T
            + t_cam_obj.reshape(1, 3)
        ).astype(np.float32)

        metadata = {
            "success": True,
            "class_id": int(target["class_id"]),
            "class_name": str(target["class_name"]),
            "confidence": float(target["confidence"]),
            "detection_count": int(detection_count),
            "mask_pixels": int(mask_pixels),
            "valid_depth_pixels": int(valid_depth_pixels),
            "xyz_m": [
                float(tx),
                float(ty),
                float(tz),
            ],
            "rpy_deg": [
                float(roll),
                float(pitch),
                float(yaw),
            ],

            # RViz visualization metadata.
            # `to_origin` and `bbox_extents_m` are the same oriented-bounds
            # information used by the original FoundationPose demo:
            #   center_pose = T_camera_object @ inv(to_origin)
            "to_origin": np.asarray(
                self.to_origin,
                dtype=np.float64,
            ).reshape(4, 4).tolist(),
            "bbox_extents_m": (
                np.asarray(
                    self.bbox[1] - self.bbox[0],
                    dtype=np.float64,
                ).reshape(3).tolist()
            ),

            "complete_object_pc_points": int(
                complete_object_pc.shape[0]
            ),
            "message": "FoundationPose register succeeded.",
        }

        return metadata, current_pose, complete_object_pc

    def _save_detection_debug(
        self,
        result,
        current_pose,
        K,
        color_bgr,
    ):
        try:
            os.makedirs(
                DEBUG_DIR,
                exist_ok=True,
            )

            annotated_bgr = result.plot()

            if (
                current_pose is not None
                and K is not None
            ):
                vis_rgb = cv2.cvtColor(
                    annotated_bgr,
                    cv2.COLOR_BGR2RGB,
                )

                center_pose = (
                    current_pose
                    @ np.linalg.inv(self.to_origin)
                )

                vis_rgb = draw_posed_3d_box(
                    K,
                    img=vis_rgb,
                    ob_in_cam=center_pose,
                    bbox=self.bbox,
                )

                vis_rgb = draw_xyz_axis(
                    vis_rgb,
                    ob_in_cam=center_pose,
                    scale=0.08,
                    K=K,
                    thickness=3,
                    transparency=0,
                    is_input_rgb=True,
                )

                annotated_bgr = cv2.cvtColor(
                    vis_rgb,
                    cv2.COLOR_RGB2BGR,
                )

            cv2.imwrite(
                os.path.join(
                    DEBUG_DIR,
                    "latest_socket_debug.png",
                ),
                annotated_bgr,
            )

        except Exception as e:
            logging.warning(
                f"Debug visualization failed: {e}"
            )

    def handle_connection(self, conn, addr):
        print(f"[SOCKET] connected: {addr}")

        conn.settimeout(SOCKET_TIMEOUT_SEC)

        request = recv_json(conn)
        print(f"[SOCKET] header received: {request}")

        op = request.get("op")

        if op != "estimate":
            send_json(
                conn,
                {
                    "success": False,
                    "error": "INVALID_OPERATION",
                    "message": f"Unknown operation: {op}",
                },
            )
            return

        color_bgr = recv_array(conn)
        print(f"[SOCKET] RGB received: {color_bgr.shape} {color_bgr.dtype}")

        depth_m = recv_array(conn)
        print(f"[SOCKET] depth received: {depth_m.shape} {depth_m.dtype}")

        K = recv_array(conn)
        print(f"[SOCKET] K received: {K.shape} {K.dtype}")

        print(
            "[SOCKET] request "
            f"rgb={color_bgr.shape} "
            f"depth={depth_m.shape}"
        )

        metadata, pose, complete_object_pc = self.estimate(
            color_bgr,
            depth_m,
            K,
        )

        send_json(
            conn,
            metadata,
        )

        if metadata["success"]:
            # 1) FoundationPose T_camera_object
            send_array(
                conn,
                pose,
            )

            # 2) Complete mesh point cloud transformed into
            #    camera_color_optical_frame.
            send_array(
                conn,
                complete_object_pc,
            )

    def serve_forever(self):
        server = socket.socket(
            socket.AF_INET,
            socket.SOCK_STREAM,
        )

        server.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_REUSEADDR,
            1,
        )

        server.bind(
            (
                SOCKET_HOST,
                SOCKET_PORT,
            )
        )

        server.listen(
            SOCKET_BACKLOG
        )

        print(
            f"[SOCKET] listening on "
            f"{SOCKET_HOST}:{SOCKET_PORT}"
        )

        try:
            while True:
                conn, addr = server.accept()

                try:
                    self.handle_connection(
                        conn,
                        addr,
                    )

                except Exception as e:
                    print(
                        "[REQUEST ERROR]",
                        repr(e),
                    )
                    traceback.print_exc()

                    if isinstance(e, torch.cuda.OutOfMemoryError):
                        print("[CUDA] Clearing PyTorch CUDA cache after OOM.")
                        torch.cuda.empty_cache()

                    # IMPORTANT:
                    # Send the actual worker error while the connection
                    # is still open. The previous version used
                    # `with conn:` and attempted this after __exit__()
                    # had already closed the socket.
                    try:
                        send_json(
                            conn,
                            {
                                "success": False,
                                "error": "WORKER_EXCEPTION",
                                "message": (
                                    f"{type(e).__name__}: {e}"
                                ),
                            },
                        )
                    except Exception as send_error:
                        print(
                            "[ERROR RESPONSE FAILED]",
                            repr(send_error),
                        )

                finally:
                    try:
                        conn.shutdown(
                            socket.SHUT_RDWR
                        )
                    except Exception:
                        pass

                    conn.close()

        except KeyboardInterrupt:
            print("\nStopping FoundationPose worker.")

        finally:
            server.close()


def main():
    worker = FoundationPoseWorker()
    worker.serve_forever()


if __name__ == "__main__":
    main()
