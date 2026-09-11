#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ "$(uname -s)" != "Darwin" || "$(uname -m)" != "arm64" ]]; then
  echo "Build on an Apple Silicon Mac using an ARM64 Python 3.11 or 3.12."
  exit 1
fi
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt pyinstaller==6.12.0
.venv/bin/python -m unittest discover -s tests -v
.venv/bin/python -m PyInstaller --noconfirm --clean --windowed --onedir \
  --target-architecture arm64 --name "Robot Arm Controller" \
  --osx-bundle-identifier com.pbharrin.robot-arm-controller main.py
ditto -c -k --sequesterRsrc --keepParent "dist/Robot Arm Controller.app" dist/Robot-Arm-Controller-macOS-arm64.zip
echo "Built dist/Robot Arm Controller.app"
