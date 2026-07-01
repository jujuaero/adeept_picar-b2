#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Client Raspberry Pi pour le serveur IA deporte.

Exemples:
  python3 remote_ai_client.py --server http://192.168.1.42:8765
  python3 remote_ai_client.py --image test.jpg --server http://192.168.1.42:8765
"""

import argparse
import base64
import json
import os
import time
import urllib.request
from typing import Dict, Optional

import cv2
import numpy as np


HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_OUTPUT = os.path.join(HERE, "remote_ai_debug.jpg")
# Aligne sur la mission _12_MissionBObstacle.py (REMOTE_AI_W/H, REMOTE_AI_TIMEOUT) :
# 960x720 pour alleger l'encodage/upload sur le Pi, l'inference GPU (~50-70ms)
# n'etant pas le goulot. Surchargeable via --width/--height/--timeout.
DEFAULT_HFOV_DEG = 54.0
DEFAULT_WIDTH = 960
DEFAULT_HEIGHT = 720


def detect_url(server_url: str) -> str:
    server_url = server_url.strip()
    if "IP_DU_LAPTOP" in server_url:
        raise RuntimeError(
            "Remplace IP_DU_LAPTOP par l'adresse IPv4 de ton laptop, "
            "par exemple http://192.168.1.42:8765"
        )
    if server_url.endswith("/"):
        server_url = server_url[:-1]
    return server_url if server_url.endswith("/detect") else server_url + "/detect"


class CameraSource:
    def __init__(self, width: int, height: int):
        try:
            from picamera2 import Picamera2
        except ImportError as exc:
            raise RuntimeError("picamera2 indisponible; utilise --image pour tester") from exc

        self.cam = Picamera2()
        cfg = self.cam.preview_configuration
        cfg.size = (width, height)
        cfg.format = "RGB888"
        self.cam.configure("preview")
        self.cam.start()
        time.sleep(0.25)

    def capture(self) -> np.ndarray:
        rgb = self.cam.capture_array()
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    def close(self) -> None:
        try:
            self.cam.stop()
            self.cam.close()
        except Exception:
            pass


def capture_camera(width: int, height: int) -> np.ndarray:
    try:
        from picamera2 import Picamera2
    except ImportError as exc:
        raise RuntimeError("picamera2 indisponible; utilise --image pour tester") from exc

    cam = Picamera2()
    cfg = cam.preview_configuration
    cfg.size = (width, height)
    cfg.format = "RGB888"
    cam.configure("preview")
    cam.start()
    time.sleep(0.25)
    rgb = cam.capture_array()
    cam.stop()
    cam.close()
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)


def frame_to_b64(frame: np.ndarray, quality: int) -> str:
    ok, enc = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        raise RuntimeError("encodage JPEG impossible")
    return base64.b64encode(enc.tobytes()).decode("ascii")


def b64_to_file(image_b64: str, path: str) -> None:
    data = base64.b64decode(image_b64)
    with open(path, "wb") as f:
        f.write(data)


def call_server(
    server_url: str,
    frame: np.ndarray,
    hfov_deg: float,
    timeout_s: float,
    quality: int,
    debug: bool,
) -> Dict:
    server_url = detect_url(server_url)
    payload = {
        "image_b64": frame_to_b64(frame, quality),
        "hfov_deg": hfov_deg,
        "debug": debug,
    }
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
        server_url,
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=timeout_s) as resp:
        return json.loads(resp.read().decode("utf-8"))


def load_frame(args: argparse.Namespace) -> np.ndarray:
    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            raise RuntimeError("image illisible: %s" % args.image)
        if args.swap_rb:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        return frame
    frame = capture_camera(args.width, args.height)
    if args.swap_rb:
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    return frame


def print_response(result: Dict) -> None:
    if not result.get("ok"):
        print("ERREUR IA:", result.get("error", result))
        return
    gate = result.get("gate", {})
    dets = result.get("detections", [])
    print("\n================ REMOTE AI ================")
    print("inference: %.0f ms | detections: %d" % (result.get("inference_ms", 0.0), len(dets)))
    if gate.get("ok"):
        print("passage: centre %+.1f deg | largeur %.1f deg" % (
            gate.get("gate_center_deg", 0.0),
            gate.get("gate_width_deg", 0.0),
        ))
    else:
        print("passage: non fiable (%s)" % gate.get("reason", "?"))

    for det in dets[:8]:
        print("  %-16s %.0f%%  angle %+.1f deg" % (
            det.get("label", "?"),
            det.get("score", 0.0) * 100.0,
            det.get("angle_center_deg", 0.0),
        ))
    print("===========================================")


def print_compact(result: Dict, dt_ms: float) -> None:
    if not result.get("ok"):
        msg = "ERREUR IA: %s" % result.get("error", result)
    else:
        gate = result.get("gate", {})
        dets = result.get("detections", [])
        if gate.get("ok"):
            msg = "IA %4.0fms | net %4.0fms | det %2d | passage %+5.1f deg larg %.1f" % (
                result.get("inference_ms", 0.0),
                dt_ms,
                len(dets),
                gate.get("gate_center_deg", 0.0),
                gate.get("gate_width_deg", 0.0),
            )
        else:
            msg = "IA %4.0fms | net %4.0fms | det %2d | passage non fiable (%s)" % (
                result.get("inference_ms", 0.0),
                dt_ms,
                len(dets),
                gate.get("reason", "?"),
            )
    print("\r" + msg[:118].ljust(118), end="", flush=True)


def run_once(args: argparse.Namespace, source: Optional[CameraSource] = None,
             compact: bool = False) -> Optional[Dict]:
    frame = source.capture() if source is not None else load_frame(args)
    if source is not None and args.swap_rb:
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    t0 = time.time()
    result = call_server(
        args.server,
        frame,
        args.hfov,
        args.timeout,
        args.quality,
        args.include_debug,
    )
    dt = (time.time() - t0) * 1000.0
    if compact:
        print_compact(result, dt)
    else:
        print_response(result)
        print("aller-retour: %.0f ms" % dt)
    if result.get("debug_jpeg_b64"):
        b64_to_file(result["debug_jpeg_b64"], args.output)
        if not compact:
            print("image debug:", args.output)
    return result


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Client Pi -> laptop IA.")
    p.add_argument("--server", required=True, help="ex: http://192.168.1.20:8765")
    p.add_argument("--image", help="image locale au lieu de Picamera2")
    p.add_argument("--output", default=DEFAULT_OUTPUT)
    p.add_argument("--width", type=int, default=DEFAULT_WIDTH)
    p.add_argument("--height", type=int, default=DEFAULT_HEIGHT)
    p.add_argument("--hfov", type=float, default=DEFAULT_HFOV_DEG)
    p.add_argument("--quality", type=int, default=82)
    p.add_argument("--timeout", type=float, default=1.5)
    p.add_argument("--loop", action="store_true")
    p.add_argument("--period", type=float, default=0.25)
    p.add_argument("--no-debug", action="store_true")
    p.add_argument("--debug", action="store_true", help="renvoie/ecrit l'image debug meme en --loop")
    p.add_argument("--verbose", action="store_true", help="affiche le detail complet a chaque frame")
    p.add_argument("--swap-rb", action="store_true", help="inverse rouge/bleu si la camera sort les couleurs a l'envers")
    return p


def main() -> int:
    args = build_arg_parser().parse_args()
    try:
        args.server = detect_url(args.server)
    except Exception as exc:
        print("Erreur:", exc)
        return 2
    args.include_debug = (not args.no_debug)
    if args.loop and not args.debug:
        args.include_debug = False
    if args.loop:
        source = None if args.image else CameraSource(args.width, args.height)
        try:
            while True:
                try:
                    run_once(args, source, compact=not args.verbose)
                except KeyboardInterrupt:
                    print("\nInterrompu.")
                    break
                except Exception as exc:
                    print("Erreur:", exc)
                time.sleep(args.period)
        finally:
            if source is not None:
                source.close()
        return 0
    run_once(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
