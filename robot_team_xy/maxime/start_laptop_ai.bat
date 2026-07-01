@echo off
setlocal
cd /d "%~dp0"

if not exist ".venv-remote-ai\Scripts\python.exe" (
    echo [1/3] Creation du venv laptop IA...
    py -3 -m venv .venv-remote-ai
    if errorlevel 1 (
        echo Impossible de creer le venv. Verifie que Python est installe.
        pause
        exit /b 1
    )
)

echo [2/3] Installation/verification des paquets...
".venv-remote-ai\Scripts\python.exe" -m pip install --upgrade pip

echo.
echo Installation PyTorch CUDA pour NVIDIA RTX...
".venv-remote-ai\Scripts\python.exe" -m pip install --upgrade torch torchvision --index-url https://download.pytorch.org/whl/cu126
if errorlevel 1 (
    echo Installation PyTorch CUDA echouee.
    pause
    exit /b 1
)

echo.
echo Installation Ultralytics + OpenCV + CLIP...
".venv-remote-ai\Scripts\python.exe" -m pip install ultralytics opencv-python "git+https://github.com/ultralytics/CLIP.git"
if errorlevel 1 (
    echo Installation echouee.
    pause
    exit /b 1
)

".venv-remote-ai\Scripts\python.exe" -c "import torch; print('torch:', torch.__version__); print('cuda available:', torch.cuda.is_available()); print('cuda version:', torch.version.cuda); print('gpu:', torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'CPU')"
echo.

echo [3/3] Lancement serveur IA sur http://0.0.0.0:8765/detect
echo.
echo Sur le Pi, utilise:
echo   python3 remote_ai_client.py --server http://IP_DU_LAPTOP:8765
echo.
echo Pour lancer la mission avec IA:
echo   cd ~/adeept_picar-b2/robot_team_xy
echo   PICAR_REMOTE_AI_URL=http://IP_DU_LAPTOP:8765 python3 _12_MissionBObstacle.py --no-gui
echo.
".venv-remote-ai\Scripts\python.exe" remote_ai_server.py --host 0.0.0.0 --port 8765 --device auto --quiet --swap-rb --imgsz 1280
pause
