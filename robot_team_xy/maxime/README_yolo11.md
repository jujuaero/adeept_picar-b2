# YOLO11n leger pour les cylindres

Objectif: remplacer le vieux SSD MobileNet par un detecteur COCO moderne pour
obtenir l'angle camera des objets `bottle`, `cup`, `wine glass` et `vase`.

Le script principal est:

```bash
python3 robot_team_xy/maxime/yolo11_cylinder_detector.py
```

Il ne modifie pas la mission. Il ecrit une image annotee:

```text
robot_team_xy/maxime/yolo11_debug.jpg
```

## Option A: ONNX via OpenCV DNN

C'est le chemin le plus simple si OpenCV est deja installe sur le Pi.
Il ne demande pas PyTorch sur le Pi.

Exporter une fois sur un PC:

```bash
python -m venv .venv-yolo
.venv-yolo\Scripts\activate
pip install ultralytics
yolo export model=yolo11n.pt format=onnx imgsz=320 opset=12 simplify=True
```

Copier ensuite `yolo11n.onnx` dans:

```text
robot_team_xy/maxime/models/yolo11n.onnx
```

Lancer sur le Pi:

```bash
python3 robot_team_xy/maxime/yolo11_cylinder_detector.py --model robot_team_xy/maxime/models/yolo11n.onnx
```

Test sur une image sans camera ni sonar:

```bash
python3 robot_team_xy/maxime/yolo11_cylinder_detector.py --image test.jpg --no-sonar --show-all
```

## Option B: NCNN

NCNN est le format le plus adapte aux CPU ARM. Il demande seulement le runtime
`ncnn`, pas PyTorch.

Exporter une fois sur un PC:

```bash
python -m venv .venv-yolo
.venv-yolo\Scripts\activate
pip install ultralytics ncnn pnnx
yolo export model=yolo11n.pt format=ncnn imgsz=320
```

Copier le dossier exporte dans:

```text
robot_team_xy/maxime/models/yolo11n_ncnn_model/
```

Installer le runtime leger sur le Pi si necessaire:

```bash
pip3 install ncnn --break-system-packages
```

Lancer:

```bash
python3 robot_team_xy/maxime/yolo11_cylinder_detector.py --model robot_team_xy/maxime/models/yolo11n_ncnn_model
```

## Reglages utiles

`--hfov 54` correspond a la camera OV5647 / Pi Cam v1. Si l'angle mesure est
decale, ajuste cette valeur.

`--imgsz 320` est le compromis vitesse/precision vise pour un robot lent. Monter
a `--imgsz 640` detecte mieux mais ralentit beaucoup.

`--conf 0.35` filtre les detections faibles. Si la bouteille est ratee mais vue
dans `--show-all`, descends vers `0.25`.

La largeur en mm n'est affichee que si le sonar donne une distance valide; sinon
le script donne quand meme l'angle centre et les bords angulaires.
