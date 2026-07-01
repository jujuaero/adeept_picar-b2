#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# TEST autonome : reperer une BOUTEILLE / un GOBELET avec un detecteur tout fait
# (TensorFlow Lite, modele COCO MobileNet-SSD). Robuste au fond charge, sans
# entrainement. NE TOUCHE PAS a la mission.
#
# Idee :
#   - le modele reconnait directement les objets ("bottle", "cup"...) meme avec
#     radiateur / fenetre / parquet brillant derriere.
#   - la CAMERA donne l'angle precis (colonne de l'objet dans l'image).
#   - le SONAR donne la distance -> position laterale + largeur en mm.
#
# PRE-REQUIS (a faire une fois sur le robot) :
#   pip3 install tflite-runtime
#   cd ~/adeept_picar-b2/robot_team_xy/maxime      # (ou le dossier de ce script)
#   wget https://storage.googleapis.com/download.tensorflow.org/models/tflite/coco_ssd_mobilenet_v1_1.0_quant_2018_06_29.zip
#   unzip coco_ssd_mobilenet_v1_1.0_quant_2018_06_29.zip
#   -> ca cree detect.tflite et labelmap.txt a cote de ce script.
#
# Sortie : objets detectes (classe, angle, distance) dans le terminal
# + 'bouteille_debug.jpg' (image annotee) a regarder.

import os
import sys
import time
import math
import numpy as np
import cv2
from picamera2 import Picamera2

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from _03_servo import set_angle
from _05_ultrason import checkdist

try:
    from tflite_runtime.interpreter import Interpreter
except ImportError:
    try:
        from ai_edge_litert.interpreter import Interpreter    # successeur de tflite-runtime
    except ImportError:
        from tensorflow.lite.python.interpreter import Interpreter

# ----- a regler -----
IMG_W, IMG_H = 640, 480
HFOV_DEG     = 62.0          # champ horizontal camera (Pi cam v2 ~62, v1 ~54) -> calibrer
SCORE_MIN    = 0.40          # confiance mini pour garder une detection
TARGET_CLASSES = {'bottle', 'cup', 'wine glass', 'vase'}   # ce qui compte comme "cylindre"

HERE = os.path.dirname(os.path.abspath(__file__))
# On prend le meilleur modele present : EfficientDet-Lite0 (fort) sinon SSD v1 (faible).
_CANDIDATES = ['efficientdet_lite0.tflite', 'detect.tflite']
MODEL_PATH = next((os.path.join(HERE, m) for m in _CANDIDATES
                   if os.path.exists(os.path.join(HERE, m))), os.path.join(HERE, 'detect.tflite'))
LABEL_PATH = os.path.join(HERE, 'labelmap.txt')

US_CH, US_FORWARD    = 1, 100
VALID_MIN, VALID_MAX = 30, 1900


def capture():
    cam = Picamera2()
    cfg = cam.preview_configuration
    cfg.size = (IMG_W, IMG_H)
    cfg.format = 'RGB888'        # array en ordre BGR (compatible cv2)
    cam.configure("preview")
    cam.start()
    time.sleep(1.0)              # laisse expo / balance des blancs se stabiliser
    frame = cam.capture_array()
    cam.stop(); cam.close()
    return frame


def sonar_distance():
    set_angle(US_CH, US_FORWARD)
    time.sleep(0.1)
    vals = []
    for _ in range(5):
        d = checkdist()
        if d is not None and VALID_MIN <= d < VALID_MAX:
            vals.append(float(d))
        time.sleep(0.02)
    if not vals:
        return None
    vals.sort()
    return vals[len(vals) // 2]


def load_labels(path):
    with open(path, 'r') as f:
        return [ln.strip() for ln in f.readlines()]


def px_to_angle_norm(xc_norm):
    """Centre horizontal normalise (0..1) -> angle (deg), + = droite."""
    return (xc_norm - 0.5) * HFOV_DEG


RAW_DEBUG_MIN = 0.20        # seuil bas juste pour voir ce que le modele detecte

def detect(interpreter, labels, frame, swap_rb):
    inp = interpreter.get_input_details()[0]
    out = interpreter.get_output_details()
    _, in_h, in_w, _ = inp['shape']

    img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) if swap_rb else frame
    resized = cv2.resize(img, (in_w, in_h))
    data = np.expand_dims(resized, axis=0)
    if inp['dtype'] == np.float32:          # modele non quantifie
        data = (np.float32(data) - 127.5) / 127.5
    interpreter.set_tensor(inp['index'], data)
    interpreter.invoke()

    # Identifie les sorties par leur FORME/CONTENU (l'ordre varie selon le modele) :
    #   - boxes  : tableau [N,4]
    #   - scores : tableau [N] a valeurs dans [0,1]
    #   - classes: tableau [N] a valeurs entieres (ids), souvent > 1
    boxes = None
    vecs = []
    for o in out:
        a = np.array(interpreter.get_tensor(o['index']))
        if a.ndim >= 1 and a.shape[0] == 1:
            a = a[0]
        if a.ndim == 2 and a.shape[-1] == 4:
            boxes = a
        elif a.ndim == 1:
            vecs.append(a)
    scores, classes = None, None
    if len(vecs) >= 2:
        a, b = vecs[0], vecs[1]
        if a.max() <= 1.0 and b.max() > 1.0:
            scores, classes = a, b
        elif b.max() <= 1.0 and a.max() > 1.0:
            scores, classes = b, a
        else:                               # ambigu : suppose (classes, scores)
            classes, scores = a, b
    if boxes is None or scores is None:
        return []

    dets = []
    for i in range(len(scores)):
        if scores[i] < RAW_DEBUG_MIN:
            continue
        cid = int(round(classes[i]))
        name = labels[cid] if 0 <= cid < len(labels) else str(cid)
        ymin, xmin, ymax, xmax = boxes[i]
        dets.append({
            'name': name, 'cid': cid, 'score': float(scores[i]),
            'xmin': float(xmin), 'xmax': float(xmax),
            'ymin': float(ymin), 'ymax': float(ymax),
        })
    return dets


def main():
    if not os.path.exists(MODEL_PATH) or not os.path.exists(LABEL_PATH):
        print("Modele introuvable. Fais d'abord (voir en-tete du fichier) :")
        print("  wget .../coco_ssd_mobilenet_v1_1.0_quant_2018_06_29.zip && unzip ...")
        print("  attendu ici : %s  +  %s" % (MODEL_PATH, LABEL_PATH))
        return

    labels = load_labels(LABEL_PATH)
    print("labelmap : %d classes ; debut = %s" % (len(labels), labels[:4]))
    interpreter = Interpreter(model_path=MODEL_PATH)
    interpreter.allocate_tensors()

    print("Distance sonar + capture camera...")
    dist = sonar_distance()
    frame = capture()
    cv2.imwrite("capture.jpg", frame)        # image brute telle que cv2 la voit

    # A/B : on teste les DEUX ordres de couleur, on garde celui qui detecte le mieux
    def top(raw):
        return sorted(raw, key=lambda x: -x['score'])[:8]
    def score_targets(raw):
        return sum(d['score'] for d in raw if d['name'] in TARGET_CLASSES)

    t0 = time.time()
    raw_swap = detect(interpreter, labels, frame, swap_rb=True)
    raw_noswap = detect(interpreter, labels, frame, swap_rb=False)
    dt = time.time() - t0

    print("\n--- ordre couleur A (avec swap R/B) ---")
    for d in top(raw_swap):
        print("  %-14s %.0f%%" % (d['name'], d['score'] * 100))
    print("--- ordre couleur B (sans swap) ---")
    for d in top(raw_noswap):
        print("  %-14s %.0f%%" % (d['name'], d['score'] * 100))

    # on retient l'ordre qui donne le plus de "cible" ; sinon le plus confiant
    raw = raw_swap if score_targets(raw_swap) >= score_targets(raw_noswap) else raw_noswap
    used = "A (swap)" if raw is raw_swap else "B (sans swap)"
    print("\n>>> ordre retenu : %s" % used)

    # cibles = cylindres, au-dessus du vrai seuil
    dets = [d for d in raw if d['name'] in TARGET_CLASSES and d['score'] >= SCORE_MIN]

    annotated = frame.copy()
    for d in raw:                            # tout en gris, meme non-cible
        gx1, gy1 = int(d['xmin'] * IMG_W), int(d['ymin'] * IMG_H)
        gx2, gy2 = int(d['xmax'] * IMG_W), int(d['ymax'] * IMG_H)
        cv2.rectangle(annotated, (gx1, gy1), (gx2, gy2), (150, 150, 150), 1)
        cv2.putText(annotated, "%s %.0f%%" % (d['name'], d['score'] * 100),
                    (gx1, max(12, gy1 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)
    msg_d = "sonar: %.0f mm" % dist if dist else "sonar: (pas d'echo)"
    print("\n=================== CYLINDRES (IA + sonar) ===================")
    print("  %s   |   detection en %.0f ms" % (msg_d, dt * 1000))
    if not dets:
        print("  aucun objet cible retenu (voir la liste brute ci-dessus).")
    # objet le plus centre en premier (le plus pertinent devant le robot)
    dets.sort(key=lambda d: abs((d['xmin'] + d['xmax']) / 2.0 - 0.5))
    for k, d in enumerate(dets):
        xc = (d['xmin'] + d['xmax']) / 2.0
        ang_l = px_to_angle_norm(d['xmin'])
        ang_r = px_to_angle_norm(d['xmax'])
        center = px_to_angle_norm(xc)
        x1, x2 = int(d['xmin'] * IMG_W), int(d['xmax'] * IMG_W)
        y1, y2 = int(d['ymin'] * IMG_H), int(d['ymax'] * IMG_H)
        col = (0, 0, 255) if k == 0 else (0, 180, 255)
        cv2.rectangle(annotated, (x1, y1), (x2, y2), col, 2)
        label = "%s %.0f%% %+.0fdeg" % (d['name'], d['score'] * 100, center)
        if dist:
            width = dist * (math.tan(math.radians(ang_r)) - math.tan(math.radians(ang_l)))
            label += " ~%.0fmm" % width
        cv2.putText(annotated, label, (x1, max(18, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)
        line = "  %s %.0f%%  centre %+.1f deg  (bords %+.1f..%+.1f)" % (
            d['name'], d['score'] * 100, center, ang_l, ang_r)
        if dist:
            line += "  largeur ~%.0f mm" % (
                dist * (math.tan(math.radians(ang_r)) - math.tan(math.radians(ang_l))))
        print(line + ("   <== le plus centre" if k == 0 else ""))
    print("==============================================================")

    cv2.imwrite("bouteille_debug.jpg", annotated)
    print("\nImage : bouteille_debug.jpg (rectangles sur les objets).")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        set_angle(US_CH, US_FORWARD)
        print("\nInterrompu.")
