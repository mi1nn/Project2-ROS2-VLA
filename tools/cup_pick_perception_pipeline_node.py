#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Cup-ramen perception pipeline:
ROS2 RealSense -> FoundationPose TCP worker -> official GraspGenX ZMQ server.

This node intentionally stops BEFORE MoveIt.
It publishes the raw camera-frame object pose and grasp poses so they can be
validated in RViz before robot motion is enabled.

Required processes:
  1) realsense2_camera                (ROS2 Jazzy host)
  2) foundation_pose_worker.py :5555  (FoundationPose Docker)
  3) graspgenx_server.py       :5556  (GraspGenX environment)
  4) this node                         (ROS2 Jazzy host)

Service:
  /cup_pick/perception   std_srvs/srv/Trigger

Outputs:
  /foundation_pose/pose
  /foundation_pose/bbox_marker
  TF camera_color_optical_frame -> foundation_pose_object

  /grasp/best_pose
  /grasp/candidate_markers
  /grasp/status
  TF camera_color_optical_frame -> graspgenx_best_grasp
"""

import json
import os
import socket
import struct
import threading
import time
import traceback

import cv2
import msgpack
import msgpack_numpy
import numpy as np
import zmq

msgpack_numpy.patch()

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from scipy.spatial.transform import Rotation
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from rclpy.time import Time
from rclpy.duration import Duration


# ============================================================
# ROS / camera configuration
# ============================================================

COLOR_TOPIC = "/camera/color/image_raw"
DEPTH_TOPIC = "/camera/aligned_depth_to_color/image_raw"
CAMERA_INFO_TOPIC = "/camera/color/camera_info"

PIPELINE_SERVICE = "/cup_pick/perception"

FOUNDATION_POSE_TOPIC = "/foundation_pose/pose"
FOUNDATION_BBOX_TOPIC = "/foundation_pose/bbox_marker"

GRASP_BEST_POSE_TOPIC = "/grasp/best_pose"
GRASP_CANDIDATE_MARKERS_TOPIC = "/grasp/candidate_markers"
GRASP_STATUS_TOPIC = "/grasp/status"

FOUNDATION_TF_CHILD = "foundation_pose_object"
GRASP_TF_CHILD = "graspgenx_best_grasp"

SYNC_QUEUE_SIZE = 10
SYNC_SLOP_SEC = 0.08
MAX_FRAME_AGE_SEC = 1.0

# A service request must use a NEW synchronized RGB-D pair captured after the
# request arrives.  If the camera drops a frame momentarily, wait/retry locally
# instead of failing the whole cup-pick attempt immediately.
FRAME_CAPTURE_MAX_ATTEMPTS = 3
FRAME_CAPTURE_WAIT_SEC = 0.7

# Keep the normal ROS camera at its current resolution.
# Only the FoundationPose request is resized.
FOUNDATIONPOSE_MAX_WIDTH = 640


# ============================================================
# FoundationPose TCP worker
# ============================================================

FOUNDATIONPOSE_HOST = "127.0.0.1"
FOUNDATIONPOSE_PORT = 5555
FOUNDATIONPOSE_TIMEOUT_SEC = 30.0


# ============================================================
# Official GraspGenX ZMQ server
# ============================================================

GRASPGENX_HOST = "127.0.0.1"
GRASPGENX_PORT = 5556
GRASPGENX_TIMEOUT_MS = 120_000

# Official released GraspGenX assets include onrobot_RG2.
GRIPPER_NAME = "onrobot_RG2"

NUM_GRASPS = 200
TOPK_NUM_GRASPS = 100
GRASP_THRESHOLD = -1.0

# Number of candidate arrows to render in RViz.
RVIZ_GRASP_COUNT = 20

# Exact live inference snapshot for the standalone GraspGenX viser process.
# The viewer reads this atomically and renders the SAME candidates that are
# handed to the robot pipeline; it does not run a second inference.
LIVE_VIS_SNAPSHOT = "/tmp/graspgenx_live/latest.npz"

# ============================================================
# Grasp selection policy
# ============================================================
# GraspGenX canonical +Z is the approach direction:
#   PREGRASP = GRASP translated by -Z in the grasp frame.
#
# Only accept grasps whose approach direction is within this angle
# from base_link -Z (vertical downward).
GRASP_MAX_TILT_DEG = 30.0
BASE_FRAME = "base_link"
GRASP_SELECTION_TF_TIMEOUT_SEC = 2.0


# ============================================================
# FoundationPose raw TCP protocol helpers
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
    length = struct.unpack(
        "!I",
        recv_exact(sock, 4),
    )[0]

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

    expected = int(np.prod(shape))
    if arr.size != expected:
        raise ValueError(
            f"Array size mismatch: expected {expected}, got {arr.size}"
        )

    return arr.reshape(shape).copy()


# ============================================================
# Camera helpers
# ============================================================

def depth_to_meters(depth_raw, encoding):
    arr = np.asarray(depth_raw)
    enc = str(encoding).upper()

    if enc in ("16UC1", "MONO16"):
        depth_m = arr.astype(np.float32) * 0.001
    elif enc == "32FC1":
        depth_m = arr.astype(np.float32)
    else:
        raise ValueError(
            f"Unsupported depth encoding: {encoding}"
        )

    invalid = (~np.isfinite(depth_m)) | (depth_m < 0.001)
    depth_m[invalid] = 0.0
    return depth_m


def scale_camera_matrix(
    K,
    info_width,
    info_height,
    image_width,
    image_height,
):
    K = np.asarray(K, dtype=np.float64).reshape(3, 3).copy()

    if (
        info_width <= 0
        or info_height <= 0
        or (
            info_width == image_width
            and info_height == image_height
        )
    ):
        return K

    sx = float(image_width) / float(info_width)
    sy = float(image_height) / float(info_height)

    K[0, 0] *= sx
    K[0, 2] *= sx
    K[1, 1] *= sy
    K[1, 2] *= sy

    return K


def resize_for_foundationpose(color_bgr, depth_m, K):
    h, w = color_bgr.shape[:2]

    if w <= FOUNDATIONPOSE_MAX_WIDTH:
        return color_bgr, depth_m, K

    scale = float(FOUNDATIONPOSE_MAX_WIDTH) / float(w)

    new_w = int(round(w * scale))
    new_h = int(round(h * scale))

    color_small = cv2.resize(
        color_bgr,
        (new_w, new_h),
        interpolation=cv2.INTER_AREA,
    )

    depth_small = cv2.resize(
        depth_m,
        (new_w, new_h),
        interpolation=cv2.INTER_NEAREST,
    ).astype(np.float32, copy=False)

    K_small = np.asarray(
        K,
        dtype=np.float64,
    ).reshape(3, 3).copy()

    sx = float(new_w) / float(w)
    sy = float(new_h) / float(h)

    K_small[0, 0] *= sx
    K_small[0, 2] *= sx
    K_small[1, 1] *= sy
    K_small[1, 2] *= sy

    return color_small, depth_small, K_small


# ============================================================
# Pose / RViz helpers
# ============================================================

def rotation_to_quaternion(R):
    return Rotation.from_matrix(
        np.asarray(R, dtype=np.float64).reshape(3, 3)
    ).as_quat()  # x, y, z, w


def matrix_to_pose_stamped(T, frame_id, stamp):
    T = np.asarray(T, dtype=np.float64).reshape(4, 4)

    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp

    msg.pose.position.x = float(T[0, 3])
    msg.pose.position.y = float(T[1, 3])
    msg.pose.position.z = float(T[2, 3])

    qx, qy, qz, qw = rotation_to_quaternion(T[:3, :3])

    msg.pose.orientation.x = float(qx)
    msg.pose.orientation.y = float(qy)
    msg.pose.orientation.z = float(qz)
    msg.pose.orientation.w = float(qw)

    return msg


def matrix_to_transform_stamped(
    T,
    parent_frame,
    child_frame,
    stamp,
):
    T = np.asarray(T, dtype=np.float64).reshape(4, 4)

    msg = TransformStamped()
    msg.header.frame_id = parent_frame
    msg.header.stamp = stamp
    msg.child_frame_id = child_frame

    msg.transform.translation.x = float(T[0, 3])
    msg.transform.translation.y = float(T[1, 3])
    msg.transform.translation.z = float(T[2, 3])

    qx, qy, qz, qw = rotation_to_quaternion(T[:3, :3])

    msg.transform.rotation.x = float(qx)
    msg.transform.rotation.y = float(qy)
    msg.transform.rotation.z = float(qz)
    msg.transform.rotation.w = float(qw)

    return msg


def make_foundation_bbox_marker(
    T_camera_object,
    to_origin,
    bbox_extents_m,
    frame_id,
    stamp,
):
    T_camera_object = np.asarray(
        T_camera_object,
        dtype=np.float64,
    ).reshape(4, 4)

    to_origin = np.asarray(
        to_origin,
        dtype=np.float64,
    ).reshape(4, 4)

    extents = np.asarray(
        bbox_extents_m,
        dtype=np.float64,
    ).reshape(3)

    # Same convention as the original working FoundationPose demo.
    T_camera_bbox = (
        T_camera_object
        @ np.linalg.inv(to_origin)
    )

    hx, hy, hz = extents / 2.0

    corners_local = np.array(
        [
            [-hx, -hy, -hz],
            [ hx, -hy, -hz],
            [ hx,  hy, -hz],
            [-hx,  hy, -hz],
            [-hx, -hy,  hz],
            [ hx, -hy,  hz],
            [ hx,  hy,  hz],
            [-hx,  hy,  hz],
        ],
        dtype=np.float64,
    )

    corners_h = np.concatenate(
        [
            corners_local,
            np.ones((8, 1), dtype=np.float64),
        ],
        axis=1,
    )

    corners_camera = (
        T_camera_bbox
        @ corners_h.T
    ).T[:, :3]

    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    ]

    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = "foundation_pose_bbox"
    marker.id = 0
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.004

    marker.color.r = 0.1
    marker.color.g = 1.0
    marker.color.b = 0.1
    marker.color.a = 1.0

    for i, j in edges:
        for idx in (i, j):
            p = Point()
            p.x = float(corners_camera[idx, 0])
            p.y = float(corners_camera[idx, 1])
            p.z = float(corners_camera[idx, 2])
            marker.points.append(p)

    return marker


def make_grasp_markers(
    grasps,
    confidences,
    frame_id,
    stamp,
    max_count=20,
):
    grasps = np.asarray(
        grasps,
        dtype=np.float64,
    ).reshape(-1, 4, 4)

    confidences = np.asarray(
        confidences,
        dtype=np.float32,
    ).reshape(-1)

    order = np.argsort(-confidences)
    order = order[: min(max_count, len(order))]

    array = MarkerArray()

    # Delete old markers first.
    clear = Marker()
    clear.header.frame_id = frame_id
    clear.header.stamp = stamp
    clear.action = Marker.DELETEALL
    array.markers.append(clear)

    if len(order) == 0:
        return array

    cmin = float(np.min(confidences[order]))
    cmax = float(np.max(confidences[order]))
    denom = max(cmax - cmin, 1e-6)

    for marker_id, idx in enumerate(order):
        T = grasps[idx]
        score = float(confidences[idx])
        norm = (score - cmin) / denom

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = "graspgenx_candidates"
        marker.id = int(marker_id)
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.pose.position.x = float(T[0, 3])
        marker.pose.position.y = float(T[1, 3])
        marker.pose.position.z = float(T[2, 3])

        qx, qy, qz, qw = rotation_to_quaternion(T[:3, :3])

        marker.pose.orientation.x = float(qx)
        marker.pose.orientation.y = float(qy)
        marker.pose.orientation.z = float(qz)
        marker.pose.orientation.w = float(qw)

        # Arrow points along the grasp frame's +X axis.
        marker.scale.x = 0.07
        marker.scale.y = 0.008
        marker.scale.z = 0.012

        # Higher score -> more green, lower -> more red.
        marker.color.r = float(1.0 - norm)
        marker.color.g = float(norm)
        marker.color.b = 0.1
        marker.color.a = 0.85

        array.markers.append(marker)

    return array



def save_live_vis_snapshot(
    point_cloud,
    grasps,
    confidences,
    selected_best_index=None,
    tilt_angles_deg=None,
    valid_indices=None,
):
    """Atomically save the exact current GraspGenX result for the web viewer.

    In addition to every raw candidate/score, save the candidate that the
    robot pipeline ACTUALLY selected after the base-frame tilt filter.
    """
    path = os.path.abspath(LIVE_VIS_SNAPSHOT)
    os.makedirs(os.path.dirname(path), exist_ok=True)

    pc = np.asarray(point_cloud, dtype=np.float32).reshape(-1, 3)
    g = np.asarray(grasps, dtype=np.float32).reshape(-1, 4, 4)
    c = np.asarray(confidences, dtype=np.float32).reshape(-1)

    if tilt_angles_deg is None:
        tilt = np.full(len(g), np.nan, dtype=np.float32)
    else:
        tilt = np.asarray(tilt_angles_deg, dtype=np.float32).reshape(-1)

    if valid_indices is None:
        valid = np.arange(len(g), dtype=np.int32)
    else:
        valid = np.asarray(valid_indices, dtype=np.int32).reshape(-1)

    selected = -1 if selected_best_index is None else int(selected_best_index)

    tmp = path + ".tmp"
    with open(tmp, "wb") as f:
        np.savez_compressed(
            f,
            point_cloud=pc,
            grasps=g,
            confidences=c,
            selected_best_index=np.asarray(selected, dtype=np.int32),
            tilt_angles_deg=tilt,
            valid_indices=valid,
            max_tilt_deg=np.asarray(GRASP_MAX_TILT_DEG, dtype=np.float32),
            gripper_name=np.asarray(GRIPPER_NAME),
            created_unix=np.asarray(time.time(), dtype=np.float64),
        )
        f.flush()
        os.fsync(f.fileno())

    os.replace(tmp, path)
    return path



def transform_rotation_matrix(transform_stamped):
    """Return R_target_source from a geometry_msgs/TransformStamped."""
    q = transform_stamped.transform.rotation
    return Rotation.from_quat(
        [q.x, q.y, q.z, q.w]
    ).as_matrix()


def select_grasp_by_base_tilt(
    grasps,
    confidences,
    R_base_camera,
    max_tilt_deg=GRASP_MAX_TILT_DEG,
):
    """Select the highest-score grasp inside a base-frame downward cone.

    GraspGenX convention used by this project:
        grasp local +Z = approach direction

    For each candidate:
        approach_base = R_base_camera @ R_camera_grasp[:, 2]

    The candidate is accepted when the angle between approach_base and
    base_link -Z = [0, 0, -1] is <= max_tilt_deg.

    Returns:
        best_index: int
        valid_indices: ndarray[int]
        tilt_angles_deg: ndarray[float]
        approach_vectors_base: ndarray[N,3]

    No unsafe fallback is used. If no grasp satisfies the cone, this raises
    instead of silently choosing a sideways grasp.
    """
    g = np.asarray(grasps, dtype=np.float64).reshape(-1, 4, 4)
    scores = np.asarray(confidences, dtype=np.float64).reshape(-1)
    R_bc = np.asarray(R_base_camera, dtype=np.float64).reshape(3, 3)

    if len(g) != len(scores):
        raise ValueError(
            f"grasps/confidences length mismatch: {len(g)} vs {len(scores)}"
        )
    if len(g) == 0:
        raise RuntimeError("No GraspGenX candidates to filter.")

    # Candidate approach axis in camera frame = grasp local +Z.
    approach_camera = g[:, :3, 2]

    # camera -> base rotation.
    approach_base = (R_bc @ approach_camera.T).T

    norms = np.linalg.norm(approach_base, axis=1)
    finite = np.isfinite(approach_base).all(axis=1) & (norms > 1e-9)

    approach_unit = np.zeros_like(approach_base)
    approach_unit[finite] = approach_base[finite] / norms[finite, None]

    # angle against base downward axis [0, 0, -1].
    # dot(v, -Z) = -v_z
    cos_angle = np.clip(-approach_unit[:, 2], -1.0, 1.0)
    tilt_deg = np.full(len(g), np.inf, dtype=np.float64)
    tilt_deg[finite] = np.degrees(np.arccos(cos_angle[finite]))

    valid = np.flatnonzero(
        finite
        & np.isfinite(scores)
        & (tilt_deg <= float(max_tilt_deg))
    )

    if len(valid) == 0:
        finite_tilts = tilt_deg[np.isfinite(tilt_deg)]
        closest = float(np.min(finite_tilts)) if len(finite_tilts) else float("inf")
        raise RuntimeError(
            "No GraspGenX candidate satisfies the base-frame downward "
            f"tilt limit <= {float(max_tilt_deg):.1f} deg. "
            f"Closest candidate tilt={closest:.1f} deg."
        )

    best_index = int(valid[np.argmax(scores[valid])])

    return (
        best_index,
        valid.astype(np.int32),
        tilt_deg,
        approach_unit,
    )


# ============================================================
# Official GraspGenX wire client
# ============================================================

class GraspGenXWireClient:

    def __init__(
        self,
        host=GRASPGENX_HOST,
        port=GRASPGENX_PORT,
        timeout_ms=GRASPGENX_TIMEOUT_MS,
    ):
        self.address = f"tcp://{host}:{port}"
        self.timeout_ms = int(timeout_ms)

    def request(self, payload):
        ctx = zmq.Context.instance()
        sock = ctx.socket(zmq.REQ)

        sock.setsockopt(
            zmq.RCVTIMEO,
            self.timeout_ms,
        )

        sock.setsockopt(
            zmq.SNDTIMEO,
            self.timeout_ms,
        )

        sock.setsockopt(
            zmq.LINGER,
            0,
        )

        sock.connect(self.address)

        try:
            packed = msgpack.packb(
                payload,
                use_bin_type=True,
            )
            sock.send(packed)

            raw = sock.recv()

            response = msgpack.unpackb(
                raw,
                raw=False,
            )

        except zmq.error.Again as exc:
            raise TimeoutError(
                f"GraspGenX server timeout at {self.address}"
            ) from exc

        finally:
            sock.close(linger=0)

        if (
            isinstance(response, dict)
            and "error" in response
        ):
            raise RuntimeError(
                f"GraspGenX server error: "
                f"{response['error']}"
            )

        return response

    def health(self):
        return self.request(
            {"action": "health"}
        )

    def infer(self, point_cloud):
        pc = np.asarray(
            point_cloud,
            dtype=np.float32,
        ).reshape(-1, 3)

        response = self.request(
            {
                "action": "infer",
                "point_cloud": pc,
                "gripper_name": GRIPPER_NAME,
                "num_grasps": int(NUM_GRASPS),
                "grasp_threshold": float(
                    GRASP_THRESHOLD
                ),
                "topk_num_grasps": int(
                    TOPK_NUM_GRASPS
                ),
            }
        )

        grasps = np.asarray(
            response["grasps"],
            dtype=np.float32,
        ).reshape(-1, 4, 4)

        confidences = np.asarray(
            response["confidences"],
            dtype=np.float32,
        ).reshape(-1)

        return grasps, confidences, response


# ============================================================
# ROS2 pipeline node
# ============================================================

class CupPickPerceptionPipeline(Node):

    def __init__(self):
        super().__init__(
            "cup_pick_perception_pipeline"
        )

        self.cv_bridge = CvBridge()
        self.frame_lock = threading.Lock()
        self.frame_condition = threading.Condition(self.frame_lock)
        self.request_lock = threading.Lock()

        self.latest_color_bgr = None
        self.latest_depth_m = None
        self.latest_stamp = None
        self.latest_frame_id = None
        self.latest_receive_time = None
        self.latest_frame_seq = 0

        self.camera_K = None
        self.camera_info_width = 0
        self.camera_info_height = 0

        self.callback_group = ReentrantCallbackGroup()

        # TF used for base-frame grasp selection.  The robot remains stationary
        # throughout perception, so this observation-time camera orientation is
        # valid for selecting the grasp before any MoveIt execution begins.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=False,
        )

        # Publishers
        self.foundation_pose_pub = self.create_publisher(
            PoseStamped,
            FOUNDATION_POSE_TOPIC,
            10,
        )

        self.foundation_bbox_pub = self.create_publisher(
            Marker,
            FOUNDATION_BBOX_TOPIC,
            10,
        )

        self.grasp_best_pub = self.create_publisher(
            PoseStamped,
            GRASP_BEST_POSE_TOPIC,
            10,
        )

        self.grasp_marker_pub = self.create_publisher(
            MarkerArray,
            GRASP_CANDIDATE_MARKERS_TOPIC,
            10,
        )

        self.grasp_status_pub = self.create_publisher(
            String,
            GRASP_STATUS_TOPIC,
            10,
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        # Service
        self.pipeline_srv = self.create_service(
            Trigger,
            PIPELINE_SERVICE,
            self.pipeline_callback,
            callback_group=self.callback_group,
        )

        # CameraInfo
        self.info_sub = self.create_subscription(
            CameraInfo,
            CAMERA_INFO_TOPIC,
            self.camera_info_callback,
            qos_profile_sensor_data,
            callback_group=self.callback_group,
        )

        # RGB + aligned depth sync
        self.color_sub = Subscriber(
            self,
            Image,
            COLOR_TOPIC,
            qos_profile=qos_profile_sensor_data,
            callback_group=self.callback_group,
        )

        self.depth_sub = Subscriber(
            self,
            Image,
            DEPTH_TOPIC,
            qos_profile=qos_profile_sensor_data,
            callback_group=self.callback_group,
        )

        self.sync = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub],
            queue_size=SYNC_QUEUE_SIZE,
            slop=SYNC_SLOP_SEC,
        )

        self.sync.registerCallback(
            self.rgbd_callback
        )

        self.grasp_client = GraspGenXWireClient()

        self.get_logger().info(
            "================================================"
        )
        self.get_logger().info(
            " Cup pick perception pipeline ready"
        )
        self.get_logger().info(
            "================================================"
        )
        self.get_logger().info(
            f"FoundationPose: tcp://"
            f"{FOUNDATIONPOSE_HOST}:{FOUNDATIONPOSE_PORT}"
        )
        self.get_logger().info(
            f"GraspGenX     : tcp://"
            f"{GRASPGENX_HOST}:{GRASPGENX_PORT}"
        )
        self.get_logger().info(
            f"Gripper       : {GRIPPER_NAME}"
        )
        self.get_logger().info(
            f"Service       : {PIPELINE_SERVICE}"
        )
        self.get_logger().info(
            "Frame retry   : "
            f"{FRAME_CAPTURE_MAX_ATTEMPTS} attempts x "
            f"{FRAME_CAPTURE_WAIT_SEC:.2f}s, fresh frame required"
        )

    def publish_grasp_status(self, text):
        msg = String()
        msg.data = str(text)
        self.grasp_status_pub.publish(msg)

    def camera_info_callback(self, msg):
        K = np.asarray(
            msg.k,
            dtype=np.float64,
        ).reshape(3, 3)

        with self.frame_condition:
            self.camera_K = K
            self.camera_info_width = int(msg.width)
            self.camera_info_height = int(msg.height)
            # Wake a pending service callback in case RGB-D is already fresh and
            # CameraInfo was the last missing piece.
            self.frame_condition.notify_all()

    def rgbd_callback(self, color_msg, depth_msg):
        try:
            color_bgr = self.cv_bridge.imgmsg_to_cv2(
                color_msg,
                desired_encoding="bgr8",
            )

            depth_raw = self.cv_bridge.imgmsg_to_cv2(
                depth_msg,
                desired_encoding="passthrough",
            )

            depth_m = depth_to_meters(
                depth_raw,
                depth_msg.encoding,
            )

            if color_bgr.shape[:2] != depth_m.shape:
                self.get_logger().warning(
                    "RGB/depth shape mismatch: "
                    f"{color_bgr.shape[:2]} vs {depth_m.shape}"
                )
                return

            with self.frame_condition:
                self.latest_color_bgr = (
                    np.asarray(
                        color_bgr,
                        dtype=np.uint8,
                    ).copy()
                )

                self.latest_depth_m = depth_m.copy()
                self.latest_stamp = color_msg.header.stamp
                self.latest_frame_id = (
                    color_msg.header.frame_id
                    or "camera_color_optical_frame"
                )
                self.latest_receive_time = time.monotonic()
                self.latest_frame_seq += 1
                self.frame_condition.notify_all()

        except Exception as e:
            self.get_logger().error(
                f"RGB-D callback failed: {e}"
            )

    def get_snapshot(self):
        with self.frame_lock:
            if (
                self.latest_color_bgr is None
                or self.latest_depth_m is None
                or self.camera_K is None
                or self.latest_receive_time is None
            ):
                raise RuntimeError(
                    "RGB/depth/CameraInfo is not ready."
                )

            age = (
                time.monotonic()
                - self.latest_receive_time
            )

            if age > MAX_FRAME_AGE_SEC:
                raise RuntimeError(
                    f"Camera frame is stale: {age:.2f}s"
                )

            color = self.latest_color_bgr.copy()
            depth = self.latest_depth_m.copy()
            K = self.camera_K.copy()
            stamp = self.latest_stamp
            frame_id = self.latest_frame_id
            info_width = self.camera_info_width
            info_height = self.camera_info_height

        h, w = color.shape[:2]

        K = scale_camera_matrix(
            K,
            info_width,
            info_height,
            w,
            h,
        )

        return (
            color,
            depth,
            K,
            stamp,
            frame_id,
        )

    def wait_for_fresh_snapshot(
        self,
        after_frame_seq,
        timeout_sec=FRAME_CAPTURE_WAIT_SEC,
    ):
        """Wait for a synchronized RGB-D pair newer than *after_frame_seq*.

        ApproximateTimeSynchronizer already guarantees the RGB/depth pair is
        within SYNC_SLOP_SEC.  The service callback additionally requires the
        pair to arrive AFTER the service request, so a stale cached frame is
        never sent to FoundationPose.
        """
        deadline = time.monotonic() + float(timeout_sec)

        with self.frame_condition:
            while rclpy.ok():
                now = time.monotonic()

                ready = (
                    self.latest_frame_seq > int(after_frame_seq)
                    and self.latest_color_bgr is not None
                    and self.latest_depth_m is not None
                    and self.camera_K is not None
                    and self.latest_receive_time is not None
                )

                if ready:
                    age = now - self.latest_receive_time
                    if age <= MAX_FRAME_AGE_SEC:
                        color = self.latest_color_bgr.copy()
                        depth = self.latest_depth_m.copy()
                        K = self.camera_K.copy()
                        stamp = self.latest_stamp
                        frame_id = self.latest_frame_id
                        info_width = self.camera_info_width
                        info_height = self.camera_info_height
                        frame_seq = int(self.latest_frame_seq)
                        break

                remaining = deadline - now
                if remaining <= 0.0:
                    raise TimeoutError(
                        "No fresh synchronized RGB-D frame arrived "
                        f"within {float(timeout_sec):.2f}s "
                        f"(last_seq={self.latest_frame_seq}, "
                        f"required_seq>{int(after_frame_seq)})."
                    )

                self.frame_condition.wait(timeout=remaining)
            else:
                raise RuntimeError("ROS shutdown while waiting for camera frame.")

        h, w = color.shape[:2]
        K = scale_camera_matrix(
            K,
            info_width,
            info_height,
            w,
            h,
        )

        return (
            color,
            depth,
            K,
            stamp,
            frame_id,
            frame_seq,
        )

    def acquire_fresh_snapshot_with_retry(self):
        """Acquire one post-request RGB-D snapshot, retrying up to 3 times."""
        with self.frame_lock:
            request_start_seq = int(self.latest_frame_seq)

        last_error = None

        self.get_logger().info(
            "[FRAME] Waiting for a fresh synchronized RGB-D pair: "
            f"max_attempts={FRAME_CAPTURE_MAX_ATTEMPTS}, "
            f"wait_per_attempt={FRAME_CAPTURE_WAIT_SEC:.2f}s, "
            f"sync_slop={SYNC_SLOP_SEC:.3f}s, "
            f"start_seq={request_start_seq}"
        )

        # Every attempt still requires a frame newer than the service request.
        # If a frame arrives, wait_for_fresh_snapshot returns immediately.
        for attempt in range(1, FRAME_CAPTURE_MAX_ATTEMPTS + 1):
            try:
                snapshot = self.wait_for_fresh_snapshot(
                    after_frame_seq=request_start_seq,
                    timeout_sec=FRAME_CAPTURE_WAIT_SEC,
                )

                frame_seq = snapshot[-1]
                self.get_logger().info(
                    f"[FRAME] Fresh RGB-D acquired "
                    f"(attempt={attempt}/{FRAME_CAPTURE_MAX_ATTEMPTS}, "
                    f"seq={frame_seq})"
                )
                return snapshot[:-1]

            except Exception as error:
                last_error = error
                self.get_logger().warning(
                    f"[FRAME] Capture attempt {attempt}/"
                    f"{FRAME_CAPTURE_MAX_ATTEMPTS} failed: {error}"
                )

        raise RuntimeError(
            "CAMERA_FRAME_UNAVAILABLE: failed to acquire a fresh "
            f"synchronized RGB-D frame after {FRAME_CAPTURE_MAX_ATTEMPTS} "
            f"attempts. Last error: {last_error}"
        )

    def request_foundationpose(
        self,
        color_bgr,
        depth_m,
        K,
    ):
        original_h, original_w = color_bgr.shape[:2]

        color_small, depth_small, K_small = (
            resize_for_foundationpose(
                color_bgr,
                depth_m,
                K,
            )
        )

        h, w = color_small.shape[:2]

        self.get_logger().info(
            "[FP] Sending snapshot: "
            f"{original_w}x{original_h} -> {w}x{h}"
        )

        with socket.create_connection(
            (
                FOUNDATIONPOSE_HOST,
                FOUNDATIONPOSE_PORT,
            ),
            timeout=FOUNDATIONPOSE_TIMEOUT_SEC,
        ) as sock:

            sock.settimeout(
                FOUNDATIONPOSE_TIMEOUT_SEC
            )

            send_json(
                sock,
                {"op": "estimate"},
            )

            send_array(
                sock,
                color_small,
            )

            send_array(
                sock,
                depth_small,
            )

            send_array(
                sock,
                K_small,
            )

            metadata = recv_json(sock)

            if not metadata.get(
                "success",
                False,
            ):
                raise RuntimeError(
                    "FoundationPose failed: "
                    f"{metadata.get('error')}: "
                    f"{metadata.get('message')}"
                )

            T_camera_object = recv_array(
                sock
            )

            complete_object_pc = recv_array(
                sock
            )

        return (
            metadata,
            np.asarray(
                T_camera_object,
                dtype=np.float64,
            ).reshape(4, 4),
            np.asarray(
                complete_object_pc,
                dtype=np.float32,
            ).reshape(-1, 3),
        )

    def pipeline_callback(
        self,
        request,
        response,
    ):
        del request

        if not self.request_lock.acquire(
            blocking=False
        ):
            response.success = False
            response.message = (
                "Perception pipeline is already running."
            )
            return response

        try:
            # The Trigger request itself is the capture event.  Do not reuse a
            # cached pre-request frame: wait for a NEW synchronized RGB-D pair.
            # Momentary RealSense drops are retried locally up to three times.
            self.publish_grasp_status(
                "CAMERA_FRAME_WAIT"
            )
            (
                color_bgr,
                depth_m,
                K,
                stamp,
                frame_id,
            ) = self.acquire_fresh_snapshot_with_retry()

            # ====================================================
            # 1. FoundationPose
            # ====================================================
            self.publish_grasp_status(
                "FOUNDATIONPOSE_RUNNING"
            )

            fp_meta, T_camera_object, object_pc = (
                self.request_foundationpose(
                    color_bgr,
                    depth_m,
                    K,
                )
            )

            fp_pose_msg = matrix_to_pose_stamped(
                T_camera_object,
                frame_id,
                stamp,
            )

            self.foundation_pose_pub.publish(
                fp_pose_msg
            )

            self.tf_broadcaster.sendTransform(
                matrix_to_transform_stamped(
                    T_camera_object,
                    frame_id,
                    FOUNDATION_TF_CHILD,
                    stamp,
                )
            )

            if (
                fp_meta.get("to_origin") is not None
                and fp_meta.get("bbox_extents_m")
                is not None
            ):
                self.foundation_bbox_pub.publish(
                    make_foundation_bbox_marker(
                        T_camera_object,
                        fp_meta["to_origin"],
                        fp_meta["bbox_extents_m"],
                        frame_id,
                        stamp,
                    )
                )

            xyz = T_camera_object[:3, 3]

            self.get_logger().info(
                "[FP SUCCESS] "
                f"XYZ=({xyz[0]:.4f}, "
                f"{xyz[1]:.4f}, "
                f"{xyz[2]:.4f}) m | "
                f"complete_pc={len(object_pc)}"
            )

            # ====================================================
            # 2. GraspGenX
            # ====================================================
            self.publish_grasp_status(
                "GRASPGENX_RUNNING"
            )

            # Lightweight official wire protocol.
            # The complete object PC is already in camera optical frame,
            # therefore all returned grasp transforms are in that same frame.
            grasps, confidences, grasp_meta = (
                self.grasp_client.infer(
                    object_pc
                )
            )

            if len(grasps) == 0:
                raise RuntimeError(
                    "GraspGenX returned no grasp candidates."
                )

            # ----------------------------------------------------
            # Base-frame approach-angle filter
            # ----------------------------------------------------
            # GraspGenX +Z is the approach axis.  Transform that axis
            # camera -> base and keep only candidates within 30 deg of
            # base_link -Z.  The arm has NOT moved yet, so the current
            # observation-time TF is the correct one to use.
            try:
                tf_base_camera = self.tf_buffer.lookup_transform(
                    BASE_FRAME,
                    frame_id,
                    Time(),
                    timeout=Duration(
                        seconds=GRASP_SELECTION_TF_TIMEOUT_SEC
                    ),
                )
            except Exception as tf_error:
                raise RuntimeError(
                    "Cannot apply grasp tilt filter because TF is unavailable: "
                    f"{BASE_FRAME} <- {frame_id}: {tf_error}"
                ) from tf_error

            R_base_camera = transform_rotation_matrix(
                tf_base_camera
            )

            (
                best_index,
                valid_indices,
                tilt_angles_deg,
                approach_vectors_base,
            ) = select_grasp_by_base_tilt(
                grasps,
                confidences,
                R_base_camera,
                GRASP_MAX_TILT_DEG,
            )

            T_camera_grasp = np.asarray(
                grasps[best_index],
                dtype=np.float64,
            ).reshape(4, 4)

            best_conf = float(
                confidences[best_index]
            )
            best_tilt_deg = float(
                tilt_angles_deg[best_index]
            )
            best_approach_base = (
                approach_vectors_base[best_index]
            )

            raw_score_best_index = int(
                np.argmax(confidences)
            )
            raw_score_best_tilt = float(
                tilt_angles_deg[raw_score_best_index]
            )

            self.get_logger().info(
                "[GRASP FILTER] "
                f"base -Z cone <= {GRASP_MAX_TILT_DEG:.1f} deg | "
                f"accepted={len(valid_indices)}/{len(grasps)} | "
                f"selected_index={best_index} "
                f"score={best_conf:.4f} "
                f"tilt={best_tilt_deg:.2f} deg | "
                "approach_base=("
                f"{best_approach_base[0]:+.3f}, "
                f"{best_approach_base[1]:+.3f}, "
                f"{best_approach_base[2]:+.3f})"
            )

            if raw_score_best_index != best_index:
                self.get_logger().info(
                    "[GRASP FILTER] Raw score-best rejected/overridden: "
                    f"raw_index={raw_score_best_index} "
                    f"raw_score={float(confidences[raw_score_best_index]):.4f} "
                    f"raw_tilt={raw_score_best_tilt:.2f} deg"
                )

            # Save every raw candidate, plus the exact index selected after
            # the base-frame angle filter, for the live web viewer.
            live_vis_path = save_live_vis_snapshot(
                object_pc,
                grasps,
                confidences,
                selected_best_index=best_index,
                tilt_angles_deg=tilt_angles_deg,
                valid_indices=valid_indices,
            )
            self.get_logger().info(
                f"[WEB VIS] exact live GraspGenX snapshot -> {live_vis_path}"
            )

            # ====================================================
            # 3. ROS / RViz output
            # ====================================================
            best_pose_msg = matrix_to_pose_stamped(
                T_camera_grasp,
                frame_id,
                stamp,
            )

            self.grasp_best_pub.publish(
                best_pose_msg
            )

            self.tf_broadcaster.sendTransform(
                matrix_to_transform_stamped(
                    T_camera_grasp,
                    frame_id,
                    GRASP_TF_CHILD,
                    stamp,
                )
            )

            self.grasp_marker_pub.publish(
                make_grasp_markers(
                    grasps,
                    confidences,
                    frame_id,
                    stamp,
                    RVIZ_GRASP_COUNT,
                )
            )

            gx = T_camera_grasp[:3, 3]

            self.get_logger().info(
                "[GRASP SUCCESS] "
                f"candidates={len(grasps)} "
                f"best_score={best_conf:.4f} "
                f"tilt={best_tilt_deg:.2f}deg "
                f"accepted={len(valid_indices)}/{len(grasps)} "
                f"XYZ=({gx[0]:.4f}, "
                f"{gx[1]:.4f}, "
                f"{gx[2]:.4f}) m"
            )

            self.publish_grasp_status(
                "SUCCESS"
            )

            response.success = True
            response.message = (
                "FoundationPose + GraspGenX succeeded. "
                f"object_xyz=({xyz[0]:.5f},"
                f"{xyz[1]:.5f},"
                f"{xyz[2]:.5f}), "
                f"grasp_xyz=({gx[0]:.5f},"
                f"{gx[1]:.5f},"
                f"{gx[2]:.5f}), "
                f"grasp_score={best_conf:.4f}, "
                f"grasp_tilt_deg={best_tilt_deg:.2f}, "
                f"valid_candidates={len(valid_indices)}, "
                f"candidates={len(grasps)}"
            )

            return response

        except Exception as e:
            self.get_logger().error(
                "Pipeline failed:\n"
                + traceback.format_exc()
            )

            self.publish_grasp_status(
                "ERROR"
            )

            response.success = False
            response.message = (
                f"{type(e).__name__}: {e}"
            )

            return response

        finally:
            self.request_lock.release()


def main(args=None):
    rclpy.init(args=args)

    node = CupPickPerceptionPipeline()

    executor = MultiThreadedExecutor(
        num_threads=4
    )

    executor.add_node(node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
