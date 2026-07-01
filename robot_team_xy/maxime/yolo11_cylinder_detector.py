#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Detecte bottle/cup/vase avec YOLO11 sans PyTorch sur le Raspberry Pi.

Chemin leger:
  - modele ONNX: execution via OpenCV DNN (OpenCV deja installe sur le Pi)
  - modele NCNN: execution via le paquet Python ncnn si installe

Sortie:
  - terminal: classe, confiance, angle centre, bords angulaires, largeur si sonar
  - image: yolo11_debug.jpg annotee
"""

import argparse
import glob
import importlib.util
import math
import os
import sys
import time
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np


HERE = os.path.dirname(os.path.abspath(__file__))
ROBOT_DIR = os.path.dirname(HERE)
sys.path.insert(0, ROBOT_DIR)

IMG_W, IMG_H = 640, 480
DEFAULT_IMGSZ = 320
DEFAULT_HFOV_DEG = 54.0  # OV5647 / Pi Camera v1: environ 54 deg horizontaux
DEFAULT_CONF = 0.35
DEFAULT_IOU = 0.45

US_CH = 1
US_FORWARD = 100
VALID_MIN_MM = 30
VALID_MAX_MM = 1900

TARGET_CLASSES = {"bottle", "cup", "wine glass", "vase"}

COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep",
    "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis",
    "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass",
    "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza",
    "donut", "cake", "chair", "couch", "potted plant", "bed",
    "dining table", "toilet", "tv", "laptop", "mouse", "remote",
    "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush",
]


@dataclass
class LetterboxInfo:
    ratio: float
    pad_x: float
    pad_y: float
    input_w: int
    input_h: int


@dataclass
class Detection:
    label: str
    class_id: int
    score: float
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def cx(self) -> float:
        return 0.5 * (self.x1 + self.x2)

    @property
    def cy(self) -> float:
        return 0.5 * (self.y1 + self.y2)

    @property
    def w(self) -> float:
        return self.x2 - self.x1

    @property
    def h(self) -> float:
        return self.y2 - self.y1


def find_default_model() -> Optional[str]:
    candidates = [
        os.path.join(HERE, "models", "yolo11n.onnx"),
        os.path.join(HERE, "yolo11n.onnx"),
        os.path.join(HERE, "models", "yolo11n_ncnn_model"),
        os.path.join(HERE, "yolo11n_ncnn_model"),
    ]
    for path in candidates:
        if os.path.exists(path):
            return path
    return None


def letterbox(frame: np.ndarray, size: int) -> Tuple[np.ndarray, LetterboxInfo]:
    h, w = frame.shape[:2]
    ratio = min(size / float(w), size / float(h))
    new_w = int(round(w * ratio))
    new_h = int(round(h * ratio))
    resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

    pad_x = (size - new_w) / 2.0
    pad_y = (size - new_h) / 2.0
    left = int(round(pad_x - 0.1))
    right = int(round(pad_x + 0.1))
    top = int(round(pad_y - 0.1))
    bottom = int(round(pad_y + 0.1))
    padded = cv2.copyMakeBorder(
        resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114)
    )
    return padded, LetterboxInfo(ratio, left, top, size, size)


def angle_from_px(px: float, image_w: int, hfov_deg: float) -> float:
    """Angle horizontal pinhole, + = droite."""
    half_fov = math.radians(hfov_deg) / 2.0
    norm = (px - image_w / 2.0) / (image_w / 2.0)
    return math.degrees(math.atan(norm * math.tan(half_fov)))


def width_mm_from_angles(dist_mm: float, left_deg: float, right_deg: float) -> float:
    return dist_mm * (
        math.tan(math.radians(right_deg)) - math.tan(math.radians(left_deg))
    )


def capture_camera(width: int, height: int) -> np.ndarray:
    try:
        from picamera2 import Picamera2
    except ImportError as exc:
        raise RuntimeError(
            "picamera2 indisponible. Utilise --image une_photo.jpg pour tester sur PC."
        ) from exc

    cam = Picamera2()
    cfg = cam.preview_configuration
    cfg.size = (width, height)
    cfg.format = "RGB888"
    cam.configure("preview")
    cam.start()
    time.sleep(1.0)
    rgb = cam.capture_array()
    cam.stop()
    cam.close()
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)


def sonar_distance() -> Optional[float]:
    try:
        from _03_servo import set_angle
        from _05_ultrason import checkdist
    except Exception as exc:
        print("Sonar ignore (modules materiel indisponibles): %s" % exc)
        return None

    set_angle(US_CH, US_FORWARD)
    time.sleep(0.10)
    vals = []
    for _ in range(5):
        d = checkdist()
        if d is not None and VALID_MIN_MM <= d < VALID_MAX_MM:
            vals.append(float(d))
        time.sleep(0.02)
    if not vals:
        return None
    vals.sort()
    return vals[len(vals) // 2]


def clamp_box(det: Detection, w: int, h: int) -> Detection:
    return Detection(
        det.label,
        det.class_id,
        det.score,
        max(0.0, min(float(w - 1), det.x1)),
        max(0.0, min(float(h - 1), det.y1)),
        max(0.0, min(float(w - 1), det.x2)),
        max(0.0, min(float(h - 1), det.y2)),
    )


def decode_yolo_output(
    outputs: Sequence[np.ndarray],
    info: LetterboxInfo,
    original_shape: Tuple[int, int],
    conf_min: float,
    iou_thr: float,
    labels: Sequence[str],
) -> List[Detection]:
    raw = None
    for out in outputs:
        arr = np.asarray(out)
        if arr.size == 0:
            continue
        if raw is None or arr.size > raw.size:
            raw = arr
    if raw is None:
        return []

    pred = np.squeeze(raw)
    if pred.ndim != 2:
        pred = pred.reshape(-1, pred.shape[-1])

    # Ultralytics detect export courant: (84, N). On veut (N, 84).
    if pred.shape[0] < pred.shape[1] and pred.shape[0] <= len(labels) + 6:
        pred = pred.T

    h, w = original_shape[:2]
    candidates = []

    for row in pred:
        if row.shape[0] < 6:
            continue

        if row.shape[0] == 6:
            x1, y1, x2, y2, score, class_id = row[:6]
            class_id = int(class_id)
            score = float(score)
        else:
            x, y, bw, bh = row[:4]
            scores = row[4:]
            if row.shape[0] == len(labels) + 5:
                obj = float(row[4])
                scores = row[5:] * obj
            class_id = int(np.argmax(scores))
            score = float(scores[class_id])
            x1 = x - bw / 2.0
            y1 = y - bh / 2.0
            x2 = x + bw / 2.0
            y2 = y + bh / 2.0

        if score < conf_min or class_id < 0 or class_id >= len(labels):
            continue

        # Support d'une sortie eventuellement normalisee.
        if max(abs(float(x1)), abs(float(y1)), abs(float(x2)), abs(float(y2))) <= 2.0:
            x1 *= info.input_w
            x2 *= info.input_w
            y1 *= info.input_h
            y2 *= info.input_h

        if x2 < x1 or y2 < y1:
            # Certains exports end2end renvoient cx,cy,w,h,score,class.
            cx, cy, bw, bh = x1, y1, x2, y2
            x1 = cx - bw / 2.0
            y1 = cy - bh / 2.0
            x2 = cx + bw / 2.0
            y2 = cy + bh / 2.0

        x1 = (float(x1) - info.pad_x) / info.ratio
        x2 = (float(x2) - info.pad_x) / info.ratio
        y1 = (float(y1) - info.pad_y) / info.ratio
        y2 = (float(y2) - info.pad_y) / info.ratio
        det = clamp_box(
            Detection(labels[class_id], class_id, score, x1, y1, x2, y2), w, h
        )
        if det.w > 1 and det.h > 1:
            candidates.append(det)

    boxes = [[int(d.x1), int(d.y1), int(d.w), int(d.h)] for d in candidates]
    scores = [float(d.score) for d in candidates]
    keep = cv2.dnn.NMSBoxes(boxes, scores, conf_min, iou_thr)
    if len(keep) == 0:
        return []
    indexes = np.array(keep).reshape(-1).tolist()
    return [candidates[i] for i in indexes]


class OpenCVDnnYolo:
    def __init__(self, model_path: str, imgsz: int, threads: int):
        self.model_path = model_path
        self.imgsz = imgsz
        if threads > 0:
            cv2.setNumThreads(threads)
        self.net = cv2.dnn.readNetFromONNX(model_path)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    def infer(self, frame_bgr: np.ndarray) -> Tuple[List[np.ndarray], LetterboxInfo]:
        img, info = letterbox(frame_bgr, self.imgsz)
        blob = cv2.dnn.blobFromImage(
            img, scalefactor=1.0 / 255.0, size=(self.imgsz, self.imgsz), swapRB=True
        )
        self.net.setInput(blob)
        names = self.net.getUnconnectedOutLayersNames()
        outputs = self.net.forward(names)
        return [np.asarray(o) for o in outputs], info


class NcnnYolo:
    def __init__(self, model_dir: str, imgsz: int, threads: int):
        try:
            import ncnn
        except ImportError as exc:
            raise RuntimeError(
                "Le paquet Python 'ncnn' n'est pas installe. "
                "Installe-le ou utilise le modele ONNX avec OpenCV."
            ) from exc

        self.ncnn = ncnn
        self.imgsz = imgsz
        self.threads = max(1, int(threads or 4))
        self.input_name, self.output_name = self._read_names(model_dir)
        param = self._first_existing(
            model_dir, ["model.ncnn.param", "*.ncnn.param", "*.param"]
        )
        bin_file = self._first_existing(
            model_dir, ["model.ncnn.bin", "*.ncnn.bin", "*.bin"]
        )
        if not param or not bin_file:
            raise RuntimeError("Dossier NCNN incomplet: .param/.bin introuvables.")

        self.net = ncnn.Net()
        self.net.opt.num_threads = self.threads
        self.net.opt.use_vulkan_compute = False
        self.net.load_param(param)
        self.net.load_model(bin_file)

    @staticmethod
    def _first_existing(model_dir: str, patterns: Iterable[str]) -> Optional[str]:
        for pattern in patterns:
            matches = glob.glob(os.path.join(model_dir, pattern))
            if matches:
                return matches[0]
        return None

    @staticmethod
    def _read_names(model_dir: str) -> Tuple[str, str]:
        # Les exports Ultralytics/PNNX utilisent typiquement in0/out0.
        meta = os.path.join(model_dir, "metadata.yaml")
        if os.path.exists(meta):
            try:
                import yaml

                with open(meta, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f) or {}
                input_name = (data.get("input_names") or ["in0"])[0]
                output_name = (data.get("output_names") or ["out0"])[0]
                return str(input_name), str(output_name)
            except Exception:
                pass
        return "in0", "out0"

    def infer(self, frame_bgr: np.ndarray) -> Tuple[List[np.ndarray], LetterboxInfo]:
        img, info = letterbox(frame_bgr, self.imgsz)
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        mat = self.ncnn.Mat.from_pixels(
            img_rgb, self.ncnn.Mat.PixelType.PIXEL_RGB, self.imgsz, self.imgsz
        )
        mat.substract_mean_normalize([], [1.0 / 255.0, 1.0 / 255.0, 1.0 / 255.0])

        ex = self.net.create_extractor()
        ex.set_light_mode(True)
        ex.set_num_threads(self.threads)
        ex.input(self.input_name, mat)
        ret, out = ex.extract(self.output_name)
        if ret != 0:
            raise RuntimeError("NCNN extract('%s') a echoue: %s" % (self.output_name, ret))
        return [np.array(out)], info


def load_backend(model_path: str, imgsz: int, threads: int):
    if os.path.isdir(model_path):
        return NcnnYolo(model_path, imgsz, threads), "NCNN"
    if model_path.lower().endswith(".onnx"):
        return OpenCVDnnYolo(model_path, imgsz, threads), "OpenCV DNN / ONNX"
    raise RuntimeError("Modele non supporte: %s (attendu .onnx ou dossier NCNN)" % model_path)


def read_frame(args: argparse.Namespace) -> np.ndarray:
    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            raise RuntimeError("Image illisible: %s" % args.image)
        return frame
    return capture_camera(args.width, args.height)


def draw_detections(
    frame: np.ndarray,
    detections: Sequence[Detection],
    targets: Sequence[Detection],
    dist_mm: Optional[float],
    hfov_deg: float,
) -> np.ndarray:
    annotated = frame.copy()
    for det in detections:
        is_target = det in targets
        color = (0, 0, 255) if is_target else (150, 150, 150)
        thick = 2 if is_target else 1
        x1, y1, x2, y2 = map(int, [det.x1, det.y1, det.x2, det.y2])
        cv2.rectangle(annotated, (x1, y1), (x2, y2), color, thick)
        center = angle_from_px(det.cx, frame.shape[1], hfov_deg)
        label = "%s %.0f%% %+.1fdeg" % (det.label, det.score * 100.0, center)
        if is_target and dist_mm is not None:
            left = angle_from_px(det.x1, frame.shape[1], hfov_deg)
            right = angle_from_px(det.x2, frame.shape[1], hfov_deg)
            label += " ~%.0fmm" % width_mm_from_angles(dist_mm, left, right)
        cv2.putText(
            annotated,
            label,
            (x1, max(18, y1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            color,
            2 if is_target else 1,
        )
    return annotated


def print_results(
    detections: Sequence[Detection],
    targets: Sequence[Detection],
    dist_mm: Optional[float],
    hfov_deg: float,
    image_w: int,
    elapsed_s: float,
    backend_name: str,
) -> None:
    msg_d = "sonar: %.0f mm" % dist_mm if dist_mm is not None else "sonar: ignore/pas d'echo"
    print("\n=================== YOLO11 CYLINDRES ===================")
    print("  backend: %s | inference %.0f ms | %s" % (backend_name, elapsed_s * 1000.0, msg_d))
    print("  detections brutes: %d | cibles: %d" % (len(detections), len(targets)))

    if not targets:
        print("  aucune cible bottle/cup/wine glass/vase retenue.")
        top = sorted(detections, key=lambda d: -d.score)[:8]
        if top:
            print("  top brut:")
            for det in top:
                print("    %-14s %.0f%%" % (det.label, det.score * 100.0))
        print("========================================================")
        return

    ordered = sorted(targets, key=lambda d: abs(d.cx - image_w / 2.0))
    for i, det in enumerate(ordered):
        left = angle_from_px(det.x1, image_w, hfov_deg)
        right = angle_from_px(det.x2, image_w, hfov_deg)
        center = angle_from_px(det.cx, image_w, hfov_deg)
        line = "  %s %.0f%% centre %+.1f deg (bords %+.1f..%+.1f)" % (
            det.label,
            det.score * 100.0,
            center,
            left,
            right,
        )
        if dist_mm is not None:
            line += " largeur ~%.0f mm" % width_mm_from_angles(dist_mm, left, right)
        if i == 0:
            line += "  <== le plus centre"
        print(line)
    print("========================================================")


def build_arg_parser() -> argparse.ArgumentParser:
    default_model = find_default_model()
    p = argparse.ArgumentParser(
        description="Detection YOLO11n bottle/cup legere pour PiCar-B."
    )
    p.add_argument("--model", default=default_model, help=".onnx ou dossier *_ncnn_model")
    p.add_argument("--image", help="Image de test. Sans --image, capture Picamera2.")
    p.add_argument("--output", default=os.path.join(HERE, "yolo11_debug.jpg"))
    p.add_argument("--imgsz", type=int, default=DEFAULT_IMGSZ)
    p.add_argument("--conf", type=float, default=DEFAULT_CONF)
    p.add_argument("--iou", type=float, default=DEFAULT_IOU)
    p.add_argument("--hfov", type=float, default=DEFAULT_HFOV_DEG)
    p.add_argument("--width", type=int, default=IMG_W)
    p.add_argument("--height", type=int, default=IMG_H)
    p.add_argument("--threads", type=int, default=4)
    p.add_argument("--no-sonar", action="store_true")
    p.add_argument(
        "--show-all",
        action="store_true",
        help="Dessine aussi les non-cibles en gris sur l'image annotee.",
    )
    return p


def main() -> int:
    args = build_arg_parser().parse_args()
    if not args.model:
        print("Modele introuvable.")
        print("Place par exemple robot_team_xy/maxime/models/yolo11n.onnx")
        print("ou robot_team_xy/maxime/models/yolo11n_ncnn_model/.")
        print("Voir README_yolo11.md pour l'export.")
        return 2

    backend, backend_name = load_backend(args.model, args.imgsz, args.threads)
    frame = read_frame(args)

    dist = None if args.no_sonar else sonar_distance()

    t0 = time.time()
    outputs, info = backend.infer(frame)
    detections = decode_yolo_output(outputs, info, frame.shape, args.conf, args.iou, COCO_CLASSES)
    elapsed = time.time() - t0

    targets = [d for d in detections if d.label in TARGET_CLASSES]
    targets.sort(key=lambda d: abs(d.cx - frame.shape[1] / 2.0))

    shown = detections if args.show_all else targets
    annotated = draw_detections(frame, shown, targets, dist, args.hfov)
    cv2.imwrite(args.output, annotated)

    print_results(detections, targets, dist, args.hfov, frame.shape[1], elapsed, backend_name)
    print("\nImage annotee: %s" % args.output)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        try:
            from _03_servo import set_angle

            set_angle(US_CH, US_FORWARD)
        except Exception:
            pass
        print("\nInterrompu.")
