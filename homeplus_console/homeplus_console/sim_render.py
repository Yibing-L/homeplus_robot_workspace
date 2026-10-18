"""Synthetic camera frames for sim mode. Every frame is labelled SIM."""
from __future__ import annotations

import math
import time
from typing import Optional, Tuple

import numpy as np
from PIL import Image, ImageDraw, ImageFont

W, H = 424, 240


def _font(size: int):
    try:
        return ImageFont.load_default(size=size)
    except TypeError:  # Pillow without FreeType size support
        return ImageFont.load_default()


F_SMALL = _font(11)
F_BANNER = _font(13)


def _stamp(draw: ImageDraw.ImageDraw, label: str, t_wall: float, dark: bool = True) -> None:
    ts = time.strftime("%H:%M:%S", time.localtime(t_wall)) + f".{int((t_wall % 1) * 10)}"
    fill = (255, 255, 255) if dark else (20, 32, 43)
    draw.text((6, H - 16), f"SIM  {label}  {ts}", fill=fill, font=F_SMALL)


def cup_projection(base_x: float, cup_x: float) -> Tuple[float, float, float]:
    """Where the cup sits in the hand camera image, given distance from base to cup."""
    dist = max(0.18, cup_x - base_x)
    scale = 0.55 / dist
    cx = W * 0.5
    cy = H * 0.58 - 10 * scale
    return cx, cy, scale


def render_color(base_x: float, cup_x: float, t_wall: float, second_object: bool) -> Image.Image:
    img = Image.new("RGB", (W, H), (96, 104, 100))
    d = ImageDraw.Draw(img)
    # wall and table
    d.rectangle([0, 0, W, int(H * 0.45)], fill=(150, 156, 150))
    d.rectangle([0, int(H * 0.45), W, H], fill=(118, 96, 74))
    cx, cy, s = cup_projection(base_x, cup_x)
    # coffee machine behind the cup
    mw, mh = 120 * s, 150 * s
    d.rectangle([cx - mw / 2, cy - mh * 0.9, cx + mw / 2, cy + mh * 0.25], fill=(42, 44, 48))
    d.rectangle([cx - mw * 0.3, cy - mh * 0.75, cx + mw * 0.3, cy - mh * 0.55], fill=(70, 74, 80))
    # cup
    cw, ch = 34 * s, 44 * s
    d.rectangle([cx - cw / 2, cy - ch / 2, cx + cw / 2, cy + ch / 2], fill=(182, 52, 44))
    d.ellipse([cx - cw / 2, cy - ch / 2 - 4 * s, cx + cw / 2, cy - ch / 2 + 4 * s], fill=(150, 40, 34))
    if second_object:
        ox = cx + 150 * s
        d.ellipse([ox - 20 * s, cy - 6 * s, ox + 20 * s, cy + 22 * s], fill=(196, 40, 40))
    # slight sensor noise so consecutive frames differ like a real camera
    jitter = int((math.sin(t_wall * 7.3) + 1) * 3)
    d.line([0, jitter, W, jitter], fill=(150, 156, 151))
    _stamp(d, "d405 color", t_wall)
    return img


_TURBO = None


def _colormap(v: np.ndarray) -> np.ndarray:
    """Approximate turbo colormap, v in [0,1] (0 = near)."""
    r = np.clip(1.5 - np.abs(4 * v - 3), 0, 1)
    g = np.clip(1.5 - np.abs(4 * v - 2), 0, 1)
    b = np.clip(1.5 - np.abs(4 * v - 1), 0, 1)
    return (np.stack([r, g, b], axis=-1) * 255).astype(np.uint8)


DEPTH_RANGE = (0.15, 1.50)


def render_depth(base_x: float, cup_x: float, t_wall: float) -> Image.Image:
    yy, xx = np.mgrid[0:H, 0:W].astype(np.float32)
    dist = max(0.18, cup_x - base_x)
    depth = np.where(yy > H * 0.45, 0.35 + (H - yy) / H * (dist + 0.3), dist + 0.45)
    cx, cy, s = cup_projection(base_x, cup_x)
    cw, ch = 34 * s, 44 * s
    cup = (np.abs(xx - cx) < cw / 2) & (np.abs(yy - cy) < ch / 2)
    depth = np.where(cup, dist, depth)
    depth += np.random.default_rng(int(t_wall * 10)).normal(0, 0.004, depth.shape)
    lo, hi = DEPTH_RANGE
    v = np.clip((depth - lo) / (hi - lo), 0, 1)
    img = Image.fromarray(_colormap(v))
    _stamp(ImageDraw.Draw(img), "d405 depth", t_wall)
    return img


def render_overlay(color: Image.Image, stage: str, reason: str, point: Optional[Tuple[float, float]],
                   base_x: float, cup_x: float, ok: bool) -> Image.Image:
    """Mimics /molmopoint/debug_image: point, SAM2 mask outline, stage banner."""
    img = color.copy()
    d = ImageDraw.Draw(img)
    if point is not None:
        cx, cy, s = cup_projection(base_x, cup_x)
        px, py = point
        cw, ch = 34 * s, 44 * s
        d.rectangle([px - cw / 2 - 2, py - ch / 2 - 6 * s, px + cw / 2 + 2, py + ch / 2 + 2],
                    outline=(73, 209, 122), width=2)
        for i in range(-2, 3):
            for j in range(-2, 3):
                d.point((px + i * cw / 6, py + j * ch / 6), fill=(80, 220, 255))
        d.ellipse([px - 5, py - 5, px + 5, py + 5], fill=(255, 255, 255), outline=(229, 51, 42), width=3)
    banner = (30, 122, 70) if ok else (183, 121, 31)
    d.rectangle([0, 0, W, 20], fill=banner)
    d.text((6, 3), f"{stage}: {reason}", fill=(255, 255, 255), font=F_BANNER)
    return img


def render_overview(base_x: float, base_y: float, theta_deg: float, cup_x: float,
                    arm_up: bool, t_wall: float) -> Image.Image:
    """D455 side/overview camera: the robot and the table from the room corner."""
    img = Image.new("RGB", (W, H), (178, 182, 176))
    d = ImageDraw.Draw(img)
    d.rectangle([0, int(H * 0.62), W, H], fill=(140, 136, 128))  # floor
    # tape mark at start spot
    sx = 60
    d.rectangle([sx - 18, int(H * 0.86), sx + 18, int(H * 0.86) + 3], fill=(230, 196, 60))
    # table and machine
    tx = sx + cup_x * 260
    d.rectangle([tx - 10, int(H * 0.38), tx + 90, int(H * 0.42)], fill=(118, 96, 74))
    d.rectangle([tx + 6, int(H * 0.42), tx + 12, int(H * 0.86)], fill=(90, 74, 58))
    d.rectangle([tx + 20, int(H * 0.16), tx + 70, int(H * 0.38)], fill=(42, 44, 48))
    d.rectangle([tx - 4, int(H * 0.31), tx + 8, int(H * 0.38)], fill=(182, 52, 44))
    # robot
    rx = sx + base_x * 260
    d.rectangle([rx - 34, int(H * 0.70), rx + 34, int(H * 0.86)], fill=(60, 66, 72))
    d.ellipse([rx - 30, int(H * 0.84), rx - 18, int(H * 0.88) + 4], fill=(20, 20, 20))
    d.ellipse([rx + 18, int(H * 0.84), rx + 30, int(H * 0.88) + 4], fill=(20, 20, 20))
    d.rectangle([rx - 4, int(H * 0.40), rx + 4, int(H * 0.70)], fill=(80, 86, 92))
    if arm_up:
        d.line([rx, int(H * 0.42), rx + 50, int(H * 0.36)], fill=(230, 230, 230), width=5)
    else:
        d.line([rx, int(H * 0.42), rx + 10, int(H * 0.62)], fill=(230, 230, 230), width=5)
    _stamp(d, "d455 overview", t_wall, dark=False)
    return img
