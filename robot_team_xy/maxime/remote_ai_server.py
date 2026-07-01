#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Serveur IA a lancer sur le laptop GPU.

Le Raspberry envoie une image JPEG en JSON/base64 sur /detect.
Le serveur renvoie les detections + un centre de passage estime en degres.

Modele par defaut: YOLO-World, avec prompts pour cylindres/obstacles.
"""

import argparse
import base64
import json
import math
import os
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from threading import Lock
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np


DEFAULT_PROMPTS = (
    "cylinder",
    "metal cylinder",
    "plastic bottle",
    "bottle",
    "cup",
    "can",
    "obstacle",
)
DEFAULT_HFOV_DEG = 54.0

LIVE_HTML = """<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>PiCar IA Live</title>
  <style>
    html, body { height: 100%; }
    body { margin: 0; background: #111; color: #eee; font-family: Arial, sans-serif; overflow: hidden; }
    header { height: 48px; padding: 0 18px; background: #1f1f1f; display: flex; gap: 18px; align-items: center; }
    h1 { font-size: 20px; margin: 0; }
    #status { color: #aaa; font-size: 14px; }
    main { height: calc(100vh - 48px); display: grid; grid-template-columns: minmax(0, 1fr) 360px; gap: 14px; padding: 14px; box-sizing: border-box; }
    .video { min-width: 0; min-height: 0; display: flex; align-items: center; justify-content: center; overflow: hidden; background: #151515; border: 1px solid #333; }
    img { width: auto; height: auto; max-width: 100%; max-height: 100%; object-fit: contain; display: block; }
    pre { margin: 0; white-space: pre-wrap; word-break: break-word; font-size: 12px; }
    .panel { min-height: 0; overflow: auto; background: #181818; border: 1px solid #333; padding: 12px; }
    @media (max-width: 980px) {
      body { overflow: auto; }
      main { height: auto; grid-template-columns: 1fr; }
      .video { height: calc(100vh - 84px); }
    }
  </style>
</head>
<body>
  <header><h1>PiCar IA Live</h1><span id="status">en attente image...</span></header>
  <main>
    <section class="video"><img id="frame" src="/last.jpg"></section>
    <section class="panel"><pre id="json">{}</pre></section>
  </main>
  <script>
    async function tick() {
      const ts = Date.now();
      document.getElementById('frame').src = '/last.jpg?t=' + ts;
      try {
        const r = await fetch('/last.json?t=' + ts);
        const j = await r.json();
        document.getElementById('json').textContent = JSON.stringify(j, null, 2);
        const age = j.age_s == null ? '?' : j.age_s.toFixed(2);
        const gate = j.result && j.result.gate && j.result.gate.ok
          ? ('passage ' + j.result.gate.gate_center_deg.toFixed(1) + ' deg')
          : 'pas de passage fiable';
        document.getElementById('status').textContent = 'age ' + age + 's | ' + gate;
      } catch (e) {
        document.getElementById('status').textContent = 'en attente image...';
      }
    }
    setInterval(tick, 250);
    tick();
  </script>
</body>
</html>
"""


def resolve_device(device: str) -> str:
    if device != "auto":
        return device
    try:
        import torch
        if torch.cuda.is_available():
            return "0"
    except Exception:
        pass
    return "cpu"


def angle_from_px(px: float, image_w: int, hfov_deg: float) -> float:
    half_fov = math.radians(hfov_deg) / 2.0
    norm = (px - image_w / 2.0) / (image_w / 2.0)
    return math.degrees(math.atan(norm * math.tan(half_fov)))


def read_image_from_b64(image_b64: str, swap_rb: bool = False) -> np.ndarray:
    data = base64.b64decode(image_b64)
    arr = np.frombuffer(data, dtype=np.uint8)
    frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if frame is None:
        raise ValueError("image JPEG illisible")
    if swap_rb:
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    return frame


def image_to_b64(frame: np.ndarray, quality: int = 75) -> str:
    return base64.b64encode(image_to_jpeg_bytes(frame, quality)).decode("ascii")


def image_to_jpeg_bytes(frame: np.ndarray, quality: int = 75) -> bytes:
    ok, enc = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        raise ValueError("encodage JPEG impossible")
    return enc.tobytes()


def box_angle(det: Dict, image_w: int, hfov_deg: float) -> Tuple[float, float, float]:
    left = angle_from_px(det["x1"], image_w, hfov_deg)
    right = angle_from_px(det["x2"], image_w, hfov_deg)
    center = angle_from_px((det["x1"] + det["x2"]) * 0.5, image_w, hfov_deg)
    return left, center, right


def estimate_gate(
    detections: Sequence[Dict],
    image_w: int,
    image_h: int,
    hfov_deg: float,
    min_gap_px: int = 70,
) -> Dict:
    """Cherche le meilleur espace horizontal libre entre les boites detectees.

    C'est volontairement simple: on dilate un peu chaque obstacle en largeur,
    puis on choisit le gap libre qui contient le centre image ou qui en est le
    plus proche. Si rien n'est fiable, ok=False.
    """
    if not detections:
        return {
            "ok": False,
            "reason": "no_detection",
            "gate_center_deg": None,
            "gate_width_deg": None,
        }

    margin = int(0.035 * image_w)
    intervals = []
    for det in detections:
        # Ignore les detections tres hautes/minuscules peu utiles pour la conduite.
        if det["score"] < 0.15:
            continue
        if det["y2"] < image_h * 0.35:
            continue
        x1 = max(0, int(det["x1"]) - margin)
        x2 = min(image_w - 1, int(det["x2"]) + margin)
        if x2 > x1:
            intervals.append((x1, x2))

    if not intervals:
        return {
            "ok": False,
            "reason": "no_low_obstacle",
            "gate_center_deg": None,
            "gate_width_deg": None,
        }

    intervals.sort()
    merged = []
    for x1, x2 in intervals:
        if not merged or x1 > merged[-1][1]:
            merged.append([x1, x2])
        else:
            merged[-1][1] = max(merged[-1][1], x2)

    gaps = []
    cursor = 0
    for x1, x2 in merged:
        if x1 - cursor >= min_gap_px:
            gaps.append((cursor, x1))
        cursor = max(cursor, x2)
    if image_w - cursor >= min_gap_px:
        gaps.append((cursor, image_w))

    if not gaps:
        return {
            "ok": False,
            "reason": "no_gap",
            "gate_center_deg": None,
            "gate_width_deg": None,
        }

    center_x = image_w * 0.5

    def gap_cost(gap):
        gx = (gap[0] + gap[1]) * 0.5
        contains_center = gap[0] <= center_x <= gap[1]
        center_penalty = abs(gx - center_x)
        width_bonus = (gap[1] - gap[0]) * 0.25
        return (0 if contains_center else 1, center_penalty - width_bonus)

    best = min(gaps, key=gap_cost)
    gx = (best[0] + best[1]) * 0.5
    left_deg = angle_from_px(best[0], image_w, hfov_deg)
    right_deg = angle_from_px(best[1], image_w, hfov_deg)
    center_deg = angle_from_px(gx, image_w, hfov_deg)
    return {
        "ok": True,
        "reason": "gap_found",
        "gate_center_deg": center_deg,
        "gate_width_deg": abs(right_deg - left_deg),
        "gate_px": [int(best[0]), int(best[1])],
        "obstacle_intervals_px": [[int(a), int(b)] for a, b in merged],
    }


def draw_debug(frame: np.ndarray, detections: Sequence[Dict], gate: Dict, hfov_deg: float) -> np.ndarray:
    out = frame.copy()
    h, w = out.shape[:2]
    for det in detections:
        x1, y1, x2, y2 = map(int, [det["x1"], det["y1"], det["x2"], det["y2"]])
        color = (0, 0, 255)
        cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
        left, center, right = box_angle(det, w, hfov_deg)
        label = "%s %.0f%% %+.1fdeg" % (det["label"], det["score"] * 100.0, center)
        cv2.putText(out, label, (x1, max(18, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, color, 2)

    if gate.get("ok") and gate.get("gate_px"):
        a, b = gate["gate_px"]
        cx = int((a + b) * 0.5)
        cv2.rectangle(out, (a, int(h * 0.55)), (b, h - 1), (0, 220, 0), 2)
        cv2.line(out, (cx, 0), (cx, h - 1), (0, 220, 0), 2)
        cv2.putText(out, "gate %+.1fdeg width %.1fdeg" % (
            gate["gate_center_deg"], gate["gate_width_deg"]),
            (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 220, 0), 2)
    else:
        cv2.putText(out, "no reliable gate: %s" % gate.get("reason", "?"),
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 180, 255), 2)
    return out


class RemoteAI:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.args.device = resolve_device(args.device)
        self.prompts = [p.strip() for p in args.prompts.split(",") if p.strip()]
        self.live_lock = Lock()
        self.live_jpeg: Optional[bytes] = None
        self.live_result: Optional[Dict] = None
        self.live_t = 0.0
        print("Device:", self.args.device, flush=True)
        self.model = self._load_model()

    def _load_model(self):
        print("Chargement modele:", self.args.model, flush=True)
        try:
            from ultralytics import YOLOWorld
            model = YOLOWorld(self.args.model)
        except Exception:
            from ultralytics import YOLO
            model = YOLO(self.args.model)

        if hasattr(model, "set_classes"):
            print("Prompts:", ", ".join(self.prompts), flush=True)
            model.set_classes(self.prompts)
        else:
            print("Modele sans set_classes(); classes fixes.", flush=True)
        return model

    def detect(self, frame: np.ndarray, hfov_deg: float, include_debug: bool) -> Dict:
        t0 = time.time()
        results = self.model.predict(
            source=frame,
            imgsz=self.args.imgsz,
            conf=self.args.conf,
            iou=self.args.iou,
            device=self.args.device,
            verbose=False,
        )
        infer_ms = (time.time() - t0) * 1000.0
        res = results[0]
        names = res.names
        detections: List[Dict] = []
        if res.boxes is not None:
            boxes = res.boxes.xyxy.cpu().numpy()
            confs = res.boxes.conf.cpu().numpy()
            classes = res.boxes.cls.cpu().numpy().astype(int)
            for box, score, cls_id in zip(boxes, confs, classes):
                x1, y1, x2, y2 = [float(v) for v in box]
                label = str(names.get(int(cls_id), cls_id))
                detections.append({
                    "label": label,
                    "class_id": int(cls_id),
                    "score": float(score),
                    "x1": x1,
                    "y1": y1,
                    "x2": x2,
                    "y2": y2,
                    "angle_left_deg": angle_from_px(x1, frame.shape[1], hfov_deg),
                    "angle_center_deg": angle_from_px((x1 + x2) * 0.5, frame.shape[1], hfov_deg),
                    "angle_right_deg": angle_from_px(x2, frame.shape[1], hfov_deg),
                })

        detections.sort(key=lambda d: d["score"], reverse=True)
        gate = estimate_gate(detections, frame.shape[1], frame.shape[0], hfov_deg)
        response = {
            "ok": True,
            "model": self.args.model,
            "device": self.args.device,
            "hfov_deg": hfov_deg,
            "inference_ms": infer_ms,
            "detections": detections,
            "gate": gate,
        }
        debug = draw_debug(frame, detections, gate, hfov_deg)
        debug_jpeg = image_to_jpeg_bytes(debug, quality=78)
        self.update_live(debug_jpeg, response)
        if include_debug:
            response["debug_jpeg_b64"] = base64.b64encode(debug_jpeg).decode("ascii")
        return response

    def update_live(self, jpeg: bytes, result: Dict):
        public_result = {k: v for k, v in result.items() if k != "debug_jpeg_b64"}
        with self.live_lock:
            self.live_jpeg = jpeg
            self.live_result = public_result
            self.live_t = time.time()

    def live_snapshot(self):
        with self.live_lock:
            return self.live_jpeg, self.live_result, self.live_t


def make_handler(ai: RemoteAI):
    class Handler(BaseHTTPRequestHandler):
        def _send_json(self, status: int, payload: Dict):
            body = json.dumps(payload).encode("utf-8")
            self._send_bytes(status, body, "application/json")

        def _send_bytes(self, status: int, body: bytes, content_type: str):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path in ("", "/", "/index.html"):
                self._send_bytes(200, LIVE_HTML.encode("utf-8"), "text/html; charset=utf-8")
            elif path == "/health":
                self._send_json(200, {
                    "ok": True,
                    "model": ai.args.model,
                    "device": ai.args.device,
                    "prompts": ai.prompts,
                })
            elif path == "/last.jpg":
                jpeg, _result, _live_t = ai.live_snapshot()
                if jpeg is None:
                    self._send_json(404, {"ok": False, "error": "no_frame_yet"})
                else:
                    self._send_bytes(200, jpeg, "image/jpeg")
            elif path == "/last.json":
                _jpeg, result, live_t = ai.live_snapshot()
                if result is None:
                    self._send_json(200, {"ok": False, "age_s": None, "result": None})
                else:
                    self._send_json(200, {
                        "ok": True,
                        "age_s": time.time() - live_t,
                        "result": result,
                    })
            else:
                self._send_json(404, {"ok": False, "error": "not_found"})

        def do_POST(self):
            if self.path != "/detect":
                self._send_json(404, {"ok": False, "error": "not_found"})
                return
            try:
                n = int(self.headers.get("Content-Length", "0"))
                payload = json.loads(self.rfile.read(n).decode("utf-8"))
                frame = read_image_from_b64(payload["image_b64"], ai.args.swap_rb)
                hfov = float(payload.get("hfov_deg", ai.args.hfov))
                include_debug = bool(payload.get("debug", True))
                result = ai.detect(frame, hfov, include_debug)
                self._send_json(200, result)
            except Exception as exc:
                self._send_json(500, {"ok": False, "error": str(exc)})

        def log_message(self, fmt, *args):
            if ai.args.quiet:
                return
            super().log_message(fmt, *args)

    return Handler


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Serveur vision IA deporte pour le PiCar.")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8765)
    p.add_argument("--model", default="yolov8s-worldv2.pt")
    p.add_argument("--prompts", default=",".join(DEFAULT_PROMPTS))
    p.add_argument("--device", default="auto", help="'auto', '0' pour GPU CUDA, ou 'cpu'")
    p.add_argument("--imgsz", type=int, default=1280)
    p.add_argument("--conf", type=float, default=0.12)
    p.add_argument("--iou", type=float, default=0.45)
    p.add_argument("--hfov", type=float, default=DEFAULT_HFOV_DEG)
    p.add_argument("--quiet", action="store_true")
    p.add_argument("--swap-rb", action="store_true", help="inverse rouge/bleu sur les images recues")
    return p


def main() -> int:
    args = build_arg_parser().parse_args()
    ai = RemoteAI(args)
    server = ThreadingHTTPServer((args.host, args.port), make_handler(ai))
    print("Serveur pret: http://%s:%d/detect" % (args.host, args.port), flush=True)
    print("Healthcheck: http://127.0.0.1:%d/health" % args.port, flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nArret serveur.")
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
