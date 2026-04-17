#!/usr/bin/env python3

import collections
import importlib
import json
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Sequence

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from homeplus_interfaces.msg import GestureRecognition
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32, Int32, String

os.environ.setdefault("MEDIAPIPE_DISABLE_TF", "1")


EPS = 1e-6
POSE_VIS_THRESH = 0.40
HAND_MIN_VALID = 6
CLIP_ABS = 50.0

_HAND_TRIPLETS = [
    (0, 1, 2), (1, 2, 3), (2, 3, 4),
    (0, 5, 6), (5, 6, 7), (6, 7, 8),
    (0, 9, 10), (9, 10, 11), (10, 11, 12),
    (0, 13, 14), (13, 14, 15), (14, 15, 16),
    (0, 17, 18), (17, 18, 19), (18, 19, 20),
]
_ARM_TRIPLETS = [(0, 1, 2)]


@dataclass
class PinholeIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


def stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def make_velocity(x: np.ndarray) -> np.ndarray:
    return np.diff(x, axis=0, prepend=x[0:1])


def normalize_velocity_per_clip(dx: np.ndarray) -> np.ndarray:
    mag = np.sqrt((dx * dx).sum(axis=1) + EPS)
    mean_mag = float(mag.mean())
    return dx if mean_mag < EPS else (dx / mean_mag)


def metric_hand_scale(hand_xyz_wrist_rel: np.ndarray, joint_valid: np.ndarray) -> float:
    dists = []
    for joint_idx in [5, 9, 17]:
        if joint_valid[joint_idx]:
            dist = float(np.linalg.norm(hand_xyz_wrist_rel[joint_idx]))
            if np.isfinite(dist) and dist > EPS:
                dists.append(dist)
    return float(max(np.mean(dists) if dists else EPS, EPS))


def deproject(intr: PinholeIntrinsics, u: float, v: float, z_m: float) -> np.ndarray:
    x = (float(u) - intr.cx) * float(z_m) / max(intr.fx, EPS)
    y = (float(v) - intr.cy) * float(z_m) / max(intr.fy, EPS)
    return np.array([x, y, float(z_m)], dtype=np.float32)


def _value_to_meters(value, dtype, depth_unit_scale: float) -> Optional[float]:
    if not np.isfinite(value) or value <= 0:
        return None
    if np.issubdtype(dtype, np.floating):
        return float(value)
    return float(value) * depth_unit_scale


def get_depth_m(depth_img: np.ndarray, x: int, y: int, depth_unit_scale: float) -> Optional[float]:
    height, width = depth_img.shape[:2]
    if not (0 <= x < width and 0 <= y < height):
        return None

    value = _value_to_meters(depth_img[y, x], depth_img.dtype, depth_unit_scale)
    if value is not None:
        return value

    y0, y1 = max(0, y - 2), min(height, y + 3)
    x0, x1 = max(0, x - 2), min(width, x + 3)
    window = depth_img[y0:y1, x0:x1]
    valid = window[np.isfinite(window) & (window > 0)]
    if valid.size == 0:
        return None
    return _value_to_meters(np.median(valid), valid.dtype, depth_unit_scale)


def insert_angles(x_raw: np.ndarray) -> np.ndarray:
    t = x_raw.shape[0]
    hand_xyz = x_raw[:, 0:63].reshape(t, 21, 3)
    arm_xyz = x_raw[:, 63:72].reshape(t, 3, 3)
    hand_joint_valid = x_raw[:, 149:170].astype(bool)
    arm_joint_valid = x_raw[:, 170:173].astype(bool)

    angles = np.zeros((t, 16), dtype=np.float32)
    angle_valid = np.zeros((t, 16), dtype=np.float32)

    for angle_idx, (parent, joint, child) in enumerate(_HAND_TRIPLETS):
        valid = hand_joint_valid[:, parent] & hand_joint_valid[:, joint] & hand_joint_valid[:, child]
        if valid.any():
            v1 = hand_xyz[valid, parent] - hand_xyz[valid, joint]
            v2 = hand_xyz[valid, child] - hand_xyz[valid, joint]
            n1 = np.linalg.norm(v1, axis=1, keepdims=True).clip(EPS)
            n2 = np.linalg.norm(v2, axis=1, keepdims=True).clip(EPS)
            cos_a = ((v1 / n1) * (v2 / n2)).sum(axis=1).clip(-1.0, 1.0)
            angles[valid, angle_idx] = np.arccos(cos_a)
            angle_valid[valid, angle_idx] = 1.0

    for angle_idx, (parent, joint, child) in enumerate(_ARM_TRIPLETS):
        valid = arm_joint_valid[:, parent] & arm_joint_valid[:, joint] & arm_joint_valid[:, child]
        if valid.any():
            v1 = arm_xyz[valid, parent] - arm_xyz[valid, joint]
            v2 = arm_xyz[valid, child] - arm_xyz[valid, joint]
            n1 = np.linalg.norm(v1, axis=1, keepdims=True).clip(EPS)
            n2 = np.linalg.norm(v2, axis=1, keepdims=True).clip(EPS)
            cos_a = ((v1 / n1) * (v2 / n2)).sum(axis=1).clip(-1.0, 1.0)
            angles[valid, 15 + angle_idx] = np.arccos(cos_a)
            angle_valid[valid, 15 + angle_idx] = 1.0

    return np.concatenate(
        [x_raw[:, :148], angles, x_raw[:, 148:], angle_valid],
        axis=1,
    ).astype(np.float32)


def build_model_from_artifact(artifact: Dict):
    torch = importlib.import_module("torch")
    nn = torch.nn

    model_type = str(artifact.get("type", ""))
    if "attention_bilstm" not in model_type and not model_type.startswith("bilstm"):
        raise ValueError(
            f"Unsupported checkpoint type '{model_type}'. "
            "The ROS node currently supports Attention-BiLSTM XYZ checkpoints."
        )

    class TemporalAttentionPool(nn.Module):
        def __init__(self, embed_dim: int, num_heads: int = 4):
            super().__init__()
            self.query = nn.Parameter(torch.randn(1, 1, embed_dim) * 0.02)
            self.attn = nn.MultiheadAttention(embed_dim, num_heads, batch_first=True)

        def forward(self, x, key_padding_mask=None):
            batch = x.size(0)
            query = self.query.expand(batch, -1, -1)
            out, _ = self.attn(query, x, x, key_padding_mask=key_padding_mask)
            return out.squeeze(1)

    class AttentionBiLSTM(nn.Module):
        def __init__(
            self,
            in_dim: int,
            n_classes: int,
            hidden: int = 128,
            lstm_layers: int = 2,
            attn_heads: int = 4,
            proj_dim: int = 128,
            dropout: float = 0.4,
        ):
            super().__init__()
            self.input_proj = nn.Sequential(
                nn.Linear(in_dim, proj_dim),
                nn.LayerNorm(proj_dim),
                nn.GELU(),
            )
            self.lstm = nn.LSTM(
                proj_dim,
                hidden,
                lstm_layers,
                dropout=dropout if lstm_layers > 1 else 0.0,
                batch_first=True,
                bidirectional=True,
            )
            lstm_out_dim = hidden * 2
            self.res_proj = nn.Linear(lstm_out_dim, proj_dim) if lstm_out_dim != proj_dim else nn.Identity()
            self.res_norm = nn.LayerNorm(proj_dim)
            self.attn_pool = TemporalAttentionPool(proj_dim, num_heads=attn_heads)
            self.head = nn.Sequential(
                nn.LayerNorm(proj_dim),
                nn.Dropout(dropout),
                nn.Linear(proj_dim, n_classes),
            )

        def forward(self, x, mask):
            h_proj = self.input_proj(x)
            lengths = mask.sum(dim=1).clamp(min=1).to(torch.int64)
            packed = nn.utils.rnn.pack_padded_sequence(
                h_proj,
                lengths.cpu(),
                batch_first=True,
                enforce_sorted=False,
            )
            lstm_out, _ = self.lstm(packed)
            lstm_out, _ = nn.utils.rnn.pad_packed_sequence(lstm_out, batch_first=True)
            target_len = h_proj.size(1)
            if lstm_out.size(1) < target_len:
                pad = torch.zeros(
                    lstm_out.size(0),
                    target_len - lstm_out.size(1),
                    lstm_out.size(2),
                    device=lstm_out.device,
                )
                lstm_out = torch.cat([lstm_out, pad], dim=1)
            h = self.res_norm(self.res_proj(lstm_out) + h_proj)
            key_padding_mask = ~(mask[:, :target_len].bool())
            return self.head(self.attn_pool(h, key_padding_mask=key_padding_mask))

    class GroupLinearDownsampler(nn.Module):
        def __init__(self, group_dims):
            super().__init__()
            feature_groups = [
                ("pose", 0, 72),
                ("vel_pose", 72, 144),
                ("vel_wrist", 144, 147),
                ("scale", 147, 148),
                ("angles", 148, 164),
                ("validity", 164, 207),
            ]
            self.projections = nn.ModuleList()
            for (_, start, end), out_dim in zip(feature_groups, group_dims):
                self.projections.append(
                    nn.Sequential(
                        nn.Linear(end - start, out_dim),
                        nn.LayerNorm(out_dim),
                        nn.GELU(),
                    )
                )
            self.out_dim = int(sum(group_dims))

        def forward(self, x):
            feature_groups = [
                (0, 72),
                (72, 144),
                (144, 147),
                (147, 148),
                (148, 164),
                (164, 207),
            ]
            parts = []
            for (start, end), proj in zip(feature_groups, self.projections):
                parts.append(proj(x[:, :, start:end]))
            if x.size(-1) > 207:
                parts.append(x[:, :, 207:])
            return torch.cat(parts, dim=-1)

    class FeatureDownsampleWrapper(nn.Module):
        def __init__(self, downsampler, temporal_model):
            super().__init__()
            self.downsampler = downsampler
            self.temporal_model = temporal_model

        def forward(self, x, mask):
            return self.temporal_model(self.downsampler(x), mask)

    state_dict = artifact["state_dict"]
    has_group_downsampler = any(key.startswith("downsampler.projections.") for key in state_dict.keys())

    if has_group_downsampler:
        group_dims = []
        idx = 0
        while True:
            weight_key = f"downsampler.projections.{idx}.0.weight"
            if weight_key not in state_dict:
                break
            group_dims.append(int(state_dict[weight_key].shape[0]))
            idx += 1
        if not group_dims:
            raise RuntimeError("Checkpoint advertises a feature downsampler, but no projection weights were found.")

        temporal_model = AttentionBiLSTM(
            int(sum(group_dims)),
            int(artifact["n_classes"]),
            hidden=int(artifact.get("hidden", 128)),
            lstm_layers=int(artifact.get("lstm_layers", 2)),
            attn_heads=int(artifact.get("attn_heads", 4)),
            proj_dim=int(artifact.get("proj_dim", 128)),
            dropout=float(artifact.get("dropout", 0.4)),
        )
        model = FeatureDownsampleWrapper(GroupLinearDownsampler(group_dims), temporal_model)
    else:
        model = AttentionBiLSTM(
            int(artifact["in_dim"]),
            int(artifact["n_classes"]),
            hidden=int(artifact.get("hidden", 128)),
            lstm_layers=int(artifact.get("lstm_layers", 2)),
            attn_heads=int(artifact.get("attn_heads", 4)),
            proj_dim=int(artifact.get("proj_dim", 128)),
            dropout=float(artifact.get("dropout", 0.4)),
        )

    model.load_state_dict(state_dict)
    return model


def load_checkpoint(path: str) -> Dict:
    torch = importlib.import_module("torch")
    artifact = torch.load(path, map_location="cpu", weights_only=False)
    clip_norm = artifact.get("clip_norm")
    return {
        "artifact": artifact,
        "model": build_model_from_artifact(artifact),
        "mu": np.array(artifact["mu"], dtype=np.float32),
        "sd": np.array(artifact["sd"], dtype=np.float32),
        "n_continuous": int(artifact.get("n_continuous", 148)),
        "n_classes": int(artifact["n_classes"]),
        "in_dim": int(artifact["in_dim"]),
        "has_angles": bool("angles" in str(artifact.get("type", "")) or int(artifact["in_dim"]) >= 207),
        "scale_mean": float(artifact["scale_mean"]) if artifact.get("scale_mean") is not None else None,
        "clip_norm": True if clip_norm is None else bool(clip_norm),
    }


def load_label_map(path: Optional[str], label_names: Sequence[str]) -> Dict[int, str]:
    if label_names:
        return {idx: str(name) for idx, name in enumerate(label_names)}
    if not path:
        return {}
    label_path = Path(path)
    if not label_path.is_file():
        raise FileNotFoundError(f"Label map file not found: {label_path}")
    with label_path.open("r", encoding="utf-8") as handle:
        raw = json.load(handle)
    return {int(key): str(value) for key, value in raw.items()}


class FeatureBufferXYZ:
    def __init__(self, intr: PinholeIntrinsics, t_target: int, buffer_size: int, depth_unit_scale: float):
        self.intr = intr
        self.t_target = t_target
        self.depth_unit_scale = depth_unit_scale
        self.buf = collections.deque(maxlen=max(buffer_size, t_target))

    def append_frame(self, bgr: np.ndarray, depth: np.ndarray, holistic_result) -> None:
        height, width = bgr.shape[:2]
        arm_uvd = np.full((3, 3), np.nan, dtype=np.float32)
        arm_joint_valid = np.zeros(3, dtype=np.uint8)

        if holistic_result.pose_landmarks:
            pose_landmarks = holistic_result.pose_landmarks.landmark
            pose_module = importlib.import_module("mediapipe").solutions.holistic.PoseLandmark
            for joint_idx, landmark_idx in enumerate(
                [
                    pose_module.RIGHT_SHOULDER.value,
                    pose_module.RIGHT_ELBOW.value,
                    pose_module.RIGHT_WRIST.value,
                ]
            ):
                px = int(np.clip(pose_landmarks[landmark_idx].x * width, 0, width - 1))
                py = int(np.clip(pose_landmarks[landmark_idx].y * height, 0, height - 1))
                if pose_landmarks[landmark_idx].visibility > POSE_VIS_THRESH:
                    depth_m = get_depth_m(depth, px, py, self.depth_unit_scale)
                    if depth_m is not None:
                        arm_uvd[joint_idx] = [px, py, depth_m]
                        arm_joint_valid[joint_idx] = 1

        chosen_hand = getattr(holistic_result, "right_hand_landmarks", None)

        hand_xyz_norm = np.zeros((21, 3), dtype=np.float32)
        arm_xyz_norm = np.zeros((3, 3), dtype=np.float32)
        wrist_xyz = np.full(3, np.nan, dtype=np.float32)
        scale_m = np.nan
        joint_valid = np.zeros(21, dtype=np.uint8)
        frame_valid = False

        if chosen_hand is not None:
            hand_uvd = np.full((21, 3), np.nan, dtype=np.float32)
            for joint_idx, joint_lm in enumerate(chosen_hand.landmark):
                u = joint_lm.x * width
                v = joint_lm.y * height
                u_px = int(np.clip(round(u), 0, width - 1))
                v_px = int(np.clip(round(v), 0, height - 1))
                depth_m = get_depth_m(depth, u_px, v_px, self.depth_unit_scale)
                hand_uvd[joint_idx, 0] = u
                hand_uvd[joint_idx, 1] = v
                if depth_m is not None:
                    hand_uvd[joint_idx, 2] = depth_m
                    joint_valid[joint_idx] = 1

            wrist_depth = hand_uvd[0, 2]
            if np.isfinite(wrist_depth) and wrist_depth > 0 and joint_valid.sum() >= HAND_MIN_VALID:
                wrist_xyz[:] = deproject(
                    self.intr,
                    float(np.clip(hand_uvd[0, 0], 0, width - 1)),
                    float(np.clip(hand_uvd[0, 1], 0, height - 1)),
                    float(wrist_depth),
                )

                hand_xyz_m = np.zeros((21, 3), dtype=np.float32)
                for joint_idx in range(21):
                    if joint_valid[joint_idx]:
                        joint_xyz = deproject(
                            self.intr,
                            float(np.clip(hand_uvd[joint_idx, 0], 0, width - 1)),
                            float(np.clip(hand_uvd[joint_idx, 1], 0, height - 1)),
                            float(hand_uvd[joint_idx, 2]),
                        )
                        hand_xyz_m[joint_idx] = joint_xyz - wrist_xyz

                if any(joint_valid[joint_idx] for joint_idx in [5, 9, 17]):
                    scale_m = metric_hand_scale(hand_xyz_m, joint_valid)
                elif joint_valid[9]:
                    scale_m = float(max(np.linalg.norm(hand_xyz_m[9]), EPS))
                elif joint_valid[5]:
                    scale_m = float(max(np.linalg.norm(hand_xyz_m[5]), EPS))

                if np.isfinite(scale_m) and scale_m > EPS:
                    hand_xyz_norm = hand_xyz_m / scale_m
                    hand_xyz_norm[joint_valid == 0] = 0.0

                    arm_xyz_m = np.zeros((3, 3), dtype=np.float32)
                    for joint_idx in range(3):
                        if arm_joint_valid[joint_idx]:
                            arm_xyz_m[joint_idx] = deproject(
                                self.intr,
                                float(np.clip(arm_uvd[joint_idx, 0], 0, width - 1)),
                                float(np.clip(arm_uvd[joint_idx, 1], 0, height - 1)),
                                float(arm_uvd[joint_idx, 2]),
                            ) - wrist_xyz
                    arm_xyz_norm = arm_xyz_m / scale_m
                    arm_xyz_norm[arm_joint_valid == 0] = 0.0
                    frame_valid = True

        self.buf.append(
            (
                hand_xyz_norm,
                arm_xyz_norm,
                wrist_xyz,
                scale_m,
                joint_valid,
                arm_joint_valid,
                frame_valid,
            )
        )

    def _build_features_from_slice(self, frames) -> tuple[np.ndarray, np.ndarray]:
        t = self.t_target
        hand_xyz_norm_seq = np.stack([frame[0] for frame in frames], axis=0).astype(np.float32)
        arm_xyz_norm_seq = np.stack([frame[1] for frame in frames], axis=0).astype(np.float32)
        wrist_xyz_seq = np.nan_to_num(np.stack([frame[2] for frame in frames], axis=0), nan=0.0)
        scale_seq = np.nan_to_num(np.array([frame[3] for frame in frames], dtype=np.float32), nan=0.0)
        valid_joint_t = np.stack([frame[4] for frame in frames], axis=0).astype(np.float32)
        valid_arm_joint_t = np.stack([frame[5] for frame in frames], axis=0).astype(np.float32)
        valid_t = np.array([frame[6] for frame in frames], dtype=bool)

        pose = np.concatenate([hand_xyz_norm_seq, arm_xyz_norm_seq], axis=1).reshape(t, 72)
        d_pose = make_velocity(pose)
        d_wrist = make_velocity(wrist_xyz_seq)

        bad_now = ~valid_t
        bad_prev = np.concatenate([[bad_now[0]], bad_now[:-1]])
        bad = (bad_now | bad_prev)[:, None]
        d_pose[bad.repeat(72, axis=1)] = 0.0
        d_wrist[bad.repeat(3, axis=1)] = 0.0
        d_pose = normalize_velocity_per_clip(d_pose)
        d_wrist = normalize_velocity_per_clip(d_wrist)

        x = np.concatenate(
            [
                pose,
                d_pose,
                d_wrist,
                scale_seq[:, None],
                valid_t.astype(np.float32)[:, None],
                valid_joint_t,
                valid_arm_joint_t,
                valid_joint_t.mean(axis=1, keepdims=True),
                valid_arm_joint_t.mean(axis=1, keepdims=True),
            ],
            axis=1,
        ).astype(np.float32)

        x[:, :148] = np.clip(x[:, :148], -CLIP_ABS, CLIP_ABS)
        x = np.nan_to_num(x, nan=0.0, posinf=0.0, neginf=0.0)
        return x, valid_t

    def assemble_windows(self, window_stride: int, recent_window_count: int = 3):
        buf_list = list(self.buf)
        total_frames = len(buf_list)
        if total_frames < self.t_target:
            return None

        recent_frames = buf_list[-self.t_target:]
        hand_xyz_full = np.stack([frame[0] for frame in recent_frames], axis=0)
        valid_full = np.array([frame[6] for frame in recent_frames], dtype=bool)
        hand_valid = hand_xyz_full[valid_full, 1:, :]
        hand_motion = float(hand_valid.std(axis=0).mean()) if len(hand_valid) > 1 else 0.0

        windows = []
        for start in range(0, total_frames - self.t_target + 1, window_stride):
            windows.append(self._build_features_from_slice(buf_list[start:start + self.t_target]))

        if recent_window_count > 0 and len(windows) > recent_window_count:
            windows = windows[-recent_window_count:]

        return windows, hand_motion


class HolisticProcessor:
    def __init__(self, draw: bool = False):
        self.draw = draw
        self.mp = importlib.import_module("mediapipe")
        self.mp_holistic = self.mp.solutions.holistic
        self.holistic = self.mp_holistic.Holistic(
            static_image_mode=False,
            model_complexity=1,
            smooth_landmarks=True,
            enable_segmentation=False,
            refine_face_landmarks=False,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7,
        )
        if draw:
            self.mp_drawing = self.mp.solutions.drawing_utils
            self.mp_styles = self.mp.solutions.drawing_styles

    def process(self, bgr: np.ndarray):
        return self.holistic.process(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))

    def draw_landmarks(self, bgr: np.ndarray, result) -> None:
        if not self.draw:
            return
        if result.right_hand_landmarks:
            self.mp_drawing.draw_landmarks(
                bgr,
                result.right_hand_landmarks,
                self.mp_holistic.HAND_CONNECTIONS,
                self.mp_styles.get_default_hand_landmarks_style(),
                self.mp_styles.get_default_hand_connections_style(),
            )
        if result.pose_landmarks:
            self.mp_drawing.draw_landmarks(
                bgr,
                result.pose_landmarks,
                self.mp_holistic.POSE_CONNECTIONS,
            )

    def close(self) -> None:
        self.holistic.close()


class ProbAccumulator:
    def __init__(
        self,
        n_classes: int,
        alpha: float = 0.2,
        switch_margin: float = 0.15,
        min_hold_frames: int = 15,
        confidence_thresh: float = 0.45,
    ):
        self.n_classes = n_classes
        self.alpha = alpha
        self.switch_margin = switch_margin
        self.min_hold_frames = min_hold_frames
        self.confidence_thresh = confidence_thresh
        self.reset()

    def reset(self) -> None:
        self.ema = np.ones(self.n_classes, dtype=np.float32) / self.n_classes
        self.public_label = None
        self.hold_count = 0

    def update(self, probs: np.ndarray):
        self.ema = (1 - self.alpha) * self.ema + self.alpha * probs
        candidate = int(np.argmax(self.ema))
        max_prob = float(self.ema[candidate])
        self.hold_count += 1

        if max_prob < self.confidence_thresh:
            return None, max_prob

        if self.public_label is None:
            self.public_label = candidate
            self.hold_count = 0
        elif (
            candidate != self.public_label
            and self.hold_count >= self.min_hold_frames
            and self.ema[candidate] > self.ema[self.public_label] + self.switch_margin
        ):
            self.public_label = candidate
            self.hold_count = 0

        return self.public_label, max_prob


class GestureRecognizerNode(Node):
    def __init__(self):
        super().__init__("gesture_recognizer")

        self.declare_parameter("checkpoint_path", "")
        self.declare_parameter("label_map_path", "")
        self.declare_parameter("label_names", [])
        self.declare_parameter("device", "auto")
        self.declare_parameter("recognition_topic", "/gesture_recognition/state")
        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("depth_unit_scale", 0.001)
        self.declare_parameter("max_sync_delta_sec", 0.05)
        self.declare_parameter("t_target", 64)
        self.declare_parameter("stride_frames", 2)
        self.declare_parameter("ema_alpha", 0.4)
        self.declare_parameter("switch_margin", 0.05)
        self.declare_parameter("min_hold_frames", 6)
        self.declare_parameter("inference_stride", 12)
        self.declare_parameter("buffer_size", 128)
        self.declare_parameter("window_stride", 8)
        self.declare_parameter("recent_window_count", 3)
        self.declare_parameter("min_valid_ratio", 0.25)
        self.declare_parameter("confidence_thresh", 0.45)
        self.declare_parameter("hand_motion_thresh", 0.06)
        self.declare_parameter("idle_reset_frames", 20)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("draw_landmarks", True)
        self.declare_parameter("idle_label", "idle")

        checkpoint_path = self.get_parameter("checkpoint_path").value
        if not checkpoint_path:
            raise ValueError("Parameter 'checkpoint_path' is required.")
        if not Path(checkpoint_path).is_file():
            raise FileNotFoundError(f"Checkpoint not found: {checkpoint_path}")

        try:
            label_names = list(self.get_parameter("label_names").value or [])
        except ParameterUninitializedException:
            label_names = []
        label_map_path = self.get_parameter("label_map_path").value
        self.label_map = load_label_map(label_map_path, label_names)
        self.idle_label = str(self.get_parameter("idle_label").value)

        self.depth_unit_scale = float(self.get_parameter("depth_unit_scale").value)
        self.max_sync_delta_ns = int(float(self.get_parameter("max_sync_delta_sec").value) * 1e9)
        self.t_target = int(self.get_parameter("t_target").value)
        self.stride_frames = max(1, int(self.get_parameter("stride_frames").value))
        self.inference_stride = max(1, int(self.get_parameter("inference_stride").value))
        self.window_stride = max(1, int(self.get_parameter("window_stride").value))
        self.recent_window_count = max(1, int(self.get_parameter("recent_window_count").value))
        self.min_valid_ratio = float(self.get_parameter("min_valid_ratio").value)
        self.hand_motion_thresh = float(self.get_parameter("hand_motion_thresh").value)
        self.idle_reset_frames = max(1, int(self.get_parameter("idle_reset_frames").value))
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.draw_landmarks = bool(self.get_parameter("draw_landmarks").value)

        checkpoint = load_checkpoint(checkpoint_path)
        self.torch = importlib.import_module("torch")
        requested_device = str(self.get_parameter("device").value)
        self.device = self._resolve_device(requested_device)
        self.model = checkpoint["model"].to(self.device)
        self.model.eval()
        self.mu = checkpoint["mu"]
        self.sd = checkpoint["sd"]
        self.n_continuous = checkpoint["n_continuous"]
        self.has_angles = checkpoint["has_angles"]
        self.scale_mean = checkpoint["scale_mean"]
        self.clip_norm = checkpoint["clip_norm"]
        if self.label_map and len(self.label_map) != checkpoint["n_classes"]:
            self.get_logger().warn(
                "Label map size does not match checkpoint classes: "
                f"{len(self.label_map)} names for {checkpoint['n_classes']} classes."
            )

        self.bridge = CvBridge()
        self.holistic = HolisticProcessor(draw=self.draw_landmarks)
        self.accumulator = ProbAccumulator(
            n_classes=checkpoint["n_classes"],
            alpha=float(self.get_parameter("ema_alpha").value),
            switch_margin=float(self.get_parameter("switch_margin").value),
            min_hold_frames=int(self.get_parameter("min_hold_frames").value),
            confidence_thresh=float(self.get_parameter("confidence_thresh").value),
        )

        self.intrinsics: Optional[PinholeIntrinsics] = None
        self.feature_buffer: Optional[FeatureBufferXYZ] = None
        self.latest_depth: Optional[np.ndarray] = None
        self.latest_depth_stamp_ns: Optional[int] = None
        self.frame_index = 0
        self.inference_index = 0
        self.idle_frames = 0
        self.last_warn_ns: Dict[str, int] = {}
        self.last_logged_label = None
        self.last_logged_active = None

        self.label_pub = self.create_publisher(String, "/gesture_recognition/label", 10)
        self.class_id_pub = self.create_publisher(Int32, "/gesture_recognition/class_id", 10)
        self.confidence_pub = self.create_publisher(Float32, "/gesture_recognition/confidence", 10)
        self.active_pub = self.create_publisher(Bool, "/gesture_recognition/active", 10)
        self.debug_pub = self.create_publisher(Image, "/gesture_recognition/debug_image", 10)
        self.recognition_pub = self.create_publisher(
            GestureRecognition,
            str(self.get_parameter("recognition_topic").value),
            10,
        )

        color_topic = str(self.get_parameter("color_topic").value)
        depth_topic = str(self.get_parameter("depth_topic").value)
        camera_info_topic = str(self.get_parameter("camera_info_topic").value)

        self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, qos_profile_sensor_data)
        self.create_subscription(Image, depth_topic, self.depth_callback, qos_profile_sensor_data)
        self.create_subscription(Image, color_topic, self.color_callback, qos_profile_sensor_data)

        self.get_logger().info(
            "Gesture recognizer ready: "
            f"checkpoint={checkpoint_path}, device={self.device}, "
            f"angles={self.has_angles}, n_continuous={self.n_continuous}"
        )

    def _resolve_device(self, requested: str) -> str:
        if requested in ("", "auto"):
            return "cuda" if self.torch.cuda.is_available() else "cpu"
        if requested.startswith("cuda") and not self.torch.cuda.is_available():
            self.get_logger().warn("CUDA requested but unavailable; falling back to CPU.")
            return "cpu"
        return requested

    def _warn_throttled(self, key: str, message: str, period_sec: float = 5.0) -> None:
        now_ns = self.get_clock().now().nanoseconds
        last_ns = self.last_warn_ns.get(key, 0)
        if now_ns - last_ns >= int(period_sec * 1e9):
            self.get_logger().warn(message)
            self.last_warn_ns[key] = now_ns

    def _label_name(self, label_idx: Optional[int]) -> str:
        if label_idx is None or label_idx < 0:
            return self.idle_label
        return self.label_map.get(label_idx, str(label_idx))

    def camera_info_callback(self, msg: CameraInfo) -> None:
        if self.intrinsics is not None:
            return
        self.intrinsics = PinholeIntrinsics(
            fx=float(msg.k[0]),
            fy=float(msg.k[4]),
            cx=float(msg.k[2]),
            cy=float(msg.k[5]),
        )
        self.feature_buffer = FeatureBufferXYZ(
            intr=self.intrinsics,
            t_target=self.t_target,
            buffer_size=int(self.get_parameter("buffer_size").value),
            depth_unit_scale=self.depth_unit_scale,
        )
        self.get_logger().info(
            f"Camera intrinsics received: fx={self.intrinsics.fx:.2f}, fy={self.intrinsics.fy:.2f}, "
            f"cx={self.intrinsics.cx:.2f}, cy={self.intrinsics.cy:.2f}"
        )

    def depth_callback(self, msg: Image) -> None:
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.latest_depth_stamp_ns = stamp_to_ns(msg.header.stamp)
        except Exception as exc:
            self._warn_throttled("depth_convert", f"Failed to decode depth image: {exc}")

    def color_callback(self, msg: Image) -> None:
        if self.intrinsics is None or self.feature_buffer is None:
            return
        if self.latest_depth is None or self.latest_depth_stamp_ns is None:
            self._warn_throttled("waiting_depth", "Waiting for aligned depth frames.")
            return

        color_stamp_ns = stamp_to_ns(msg.header.stamp)
        if abs(color_stamp_ns - self.latest_depth_stamp_ns) > self.max_sync_delta_ns:
            self._warn_throttled("sync", "Color/depth timestamps are too far apart; skipping frame.")
            return

        self.frame_index += 1
        if (self.frame_index % self.stride_frames) != 0:
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self._warn_throttled("color_convert", f"Failed to decode color image: {exc}")
            return

        debug_frame = bgr.copy() if self.publish_debug_image else None
        result = self.holistic.process(bgr)
        self.feature_buffer.append_frame(bgr, self.latest_depth, result)
        if debug_frame is not None and self.draw_landmarks:
            self.holistic.draw_landmarks(debug_frame, result)

        assembled = self.feature_buffer.assemble_windows(
            self.window_stride,
            recent_window_count=self.recent_window_count,
        )
        if assembled is None:
            if debug_frame is not None:
                self._draw_overlay(debug_frame, "warming_up", self.idle_label, 0.0, False)
                self._publish_debug_image(msg, debug_frame)
            return

        windows, hand_motion = assembled
        is_active = hand_motion >= self.hand_motion_thresh
        public_label = self.accumulator.public_label
        ema_conf = float(self.accumulator.ema.max())

        if not is_active:
            self.idle_frames += 1
            if self.idle_frames >= self.idle_reset_frames:
                self.accumulator.reset()
                public_label = None
        else:
            self.idle_frames = 0

        self.inference_index += 1
        if is_active and self.inference_index % self.inference_stride == 0:
            window_probs = []
            for x_window, valid_t in windows:
                if float(valid_t.mean()) < self.min_valid_ratio:
                    continue
                x_input = x_window.copy()
                if self.has_angles:
                    x_input = insert_angles(x_input)
                if self.scale_mean is not None and self.scale_mean > 1e-8:
                    x_input[:, 147] /= self.scale_mean
                x_input[:, :self.n_continuous] = (x_input[:, :self.n_continuous] - self.mu) / (self.sd + 1e-8)
                if self.clip_norm:
                    clip_mu = x_input[:, :self.n_continuous].mean(axis=0)
                    clip_sd = x_input[:, :self.n_continuous].std(axis=0) + 1e-6
                    x_input[:, :self.n_continuous] = (x_input[:, :self.n_continuous] - clip_mu) / clip_sd

                x_tensor = self.torch.from_numpy(x_input[None]).float().to(self.device)
                m_tensor = self.torch.from_numpy(valid_t[None].astype(np.float32)).to(self.device)
                with self.torch.no_grad():
                    probs = self.torch.softmax(self.model(x_tensor, m_tensor), dim=1).cpu().numpy()[0]
                window_probs.append(probs)

            if window_probs:
                public_label, ema_conf = self.accumulator.update(np.mean(window_probs, axis=0))

        valid_ratio = float(np.mean([window[1].mean() for window in windows]))
        self._publish_result(public_label, ema_conf, is_active)

        if debug_frame is not None:
            state = "idle" if not is_active else f"motion={hand_motion:.3f} valid={valid_ratio:.2f}"
            self._draw_overlay(debug_frame, state, self._label_name(public_label), ema_conf, is_active)
            self._publish_debug_image(msg, debug_frame)

    def _publish_result(self, public_label: Optional[int], confidence: float, is_active: bool) -> None:
        label_name = self._label_name(public_label)
        class_id = -1 if public_label is None else int(public_label)
        recognition_msg = GestureRecognition()
        recognition_msg.header.stamp = self.get_clock().now().to_msg()
        recognition_msg.class_id = class_id
        recognition_msg.label = label_name
        recognition_msg.confidence = float(confidence)
        recognition_msg.active = bool(is_active)

        self.recognition_pub.publish(recognition_msg)
        self.label_pub.publish(String(data=label_name))
        self.class_id_pub.publish(Int32(data=class_id))
        self.confidence_pub.publish(Float32(data=float(confidence)))
        self.active_pub.publish(Bool(data=bool(is_active)))

        if self.last_logged_label != class_id or self.last_logged_active != is_active:
            self.get_logger().info(
                f"gesture={label_name} class_id={class_id} conf={confidence:.2f} active={is_active}"
            )
            self.last_logged_label = class_id
            self.last_logged_active = is_active

    def _draw_overlay(
        self,
        image: np.ndarray,
        state_text: str,
        gesture_text: str,
        confidence: float,
        is_active: bool,
    ) -> None:
        cv2.rectangle(image, (0, 0), (image.shape[1], 64), (0, 0, 0), -1)
        cv2.putText(
            image,
            state_text,
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            f"gesture={gesture_text} conf={confidence:.2f}",
            (10, 52),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0) if is_active else (160, 160, 160),
            2,
            cv2.LINE_AA,
        )

    def _publish_debug_image(self, src_msg: Image, image: np.ndarray) -> None:
        if not self.publish_debug_image:
            return
        debug_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        debug_msg.header = src_msg.header
        self.debug_pub.publish(debug_msg)

    def destroy_node(self):
        try:
            self.holistic.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GestureRecognizerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
