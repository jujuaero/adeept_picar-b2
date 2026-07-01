# IA deportee laptop -> Raspberry

But: faire tourner YOLO-World sur le laptop RTX, et laisser le Raspberry gerer
camera/moteurs/capteurs.

Resolution par defaut du pipeline live: camera `1280x960`, YOLO `imgsz=1280`.

## 1. Laptop Windows

Dans `robot_team_xy/maxime`, lance:

```bat
.\start_laptop_ai.bat
```

La premiere fois, le script cree un venv et installe PyTorch CUDA,
`ultralytics`, `opencv-python` et CLIP. Ensuite il lance:

```text
http://0.0.0.0:8765/detect
```

Le script affiche aussi un diagnostic GPU:

```text
cuda available: True
gpu: NVIDIA GeForce RTX 3070 ...
```

Si tu vois `cuda available: False`, le serveur peut quand meme marcher en CPU,
mais il sera beaucoup plus lent. Verifie alors le driver NVIDIA avec:

```bat
nvidia-smi
```

Dans un navigateur, teste plutot:

```text
http://127.0.0.1:8765/health
```

Un `404` sur `http://127.0.0.1:8765/detect` est normal en navigateur :
`/detect` attend une requete POST envoyee par le client Python du Pi.

Le lanceur active `--swap-rb` cote serveur pour corriger l'inversion
rouge/bleu observee sur la camera. Si une autre camera affiche deja les bonnes
couleurs, retire `--swap-rb` de `start_laptop_ai.bat`.

Pour voir en direct la derniere image recue du robot avec les detections:

```text
http://127.0.0.1:8765/
```

Important: le serveur laptop ne capture pas la camera tout seul. Il affiche la
derniere image que le Pi lui a envoyee. Pour alimenter la vue live, lance soit
la commande ci-dessous en remplacant `192.168.1.42` par l'IPv4 de ton laptop:

```bash
python3 remote_ai_client.py --server http://192.168.1.42:8765 --loop
```

soit la mission `_12_MissionBObstacle.py` avec `PICAR_REMOTE_AI_URL`.
En mode `--loop`, la console du Pi affiche une seule ligne compacte. Pour le
detail complet a chaque frame:

```bash
python3 remote_ai_client.py --server http://192.168.1.42:8765 --loop --verbose --debug
```

Pour trouver l'IP du laptop:

```bat
ipconfig
```

Cherche l'adresse IPv4 du Wi-Fi, par exemple `192.168.1.42`.

Si Windows Firewall demande l'autorisation reseau pour Python, accepte sur le
reseau prive.

## 2. Raspberry Pi

Depuis le dossier `robot_team_xy/maxime`, en remplacant `192.168.1.42` par
l'IPv4 de ton laptop:

```bash
python3 remote_ai_client.py --server http://192.168.1.42:8765
```

Le script affiche:

```text
passage: centre -3.2 deg | largeur 18.4 deg
detections: cylinder / bottle / cup...
```

et ecrit:

```text
remote_ai_debug.jpg
```

Le client capture par defaut en `1280x960`. Pour revenir au mode leger:

```bash
python3 remote_ai_client.py --server http://192.168.1.42:8765 --loop --width 640 --height 480 --quality 70
```

## 3. Lancer la mission avec IA

Quand le test client marche, lance la mission depuis `robot_team_xy`, avec la
meme IPv4 laptop:

```bash
PICAR_REMOTE_AI_URL=http://192.168.1.42:8765 python3 _12_MissionBObstacle.py --no-gui
```

Le code ajoute `/detect` tout seul si tu ne le mets pas. Sans
`PICAR_REMOTE_AI_URL`, la mission revient au mode sonar seul.

Si tu veux passer par le tunnel SSH de MobaXterm au lieu de l'IP Wi-Fi:

```bash
ssh -R 8765:127.0.0.1:8765 pi@IP_DU_PI
```

Puis, sur le Pi:

```bash
PICAR_REMOTE_AI_URL=http://127.0.0.1:8765 python3 _12_MissionBObstacle.py --no-gui
```

## 4. Test sans camera

Sur le Pi ou le laptop:

```bash
python3 remote_ai_client.py --image test.jpg --server http://192.168.1.42:8765
```

## Notes

Le serveur utilise par defaut `yolov8s-worldv2.pt` avec les prompts:

```text
cylinder, metal cylinder, plastic bottle, bottle, cup, can, obstacle
```

Pour tester d'autres prompts:

```bat
.venv-remote-ai\Scripts\python.exe remote_ai_server.py --prompts "cylinder,metal cylinder,cup,bottle,can"
```

Pour forcer le CPU:

```text
--device cpu
```

Par defaut le serveur est en `--device auto`: il prend CUDA si disponible,
sinon CPU.

Par defaut le serveur utilise `--imgsz 1280`. Si le laptop rame, baisse a
`--imgsz 960` ou `--imgsz 640`.
