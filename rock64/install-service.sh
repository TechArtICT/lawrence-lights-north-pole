#!/usr/bin/env bash
# Install height.py as a system service: starts at boot (nobody needs to log in) and
# restarts within a few seconds if it ever stops.
#
#   bash rock64/install-service.sh
#
# Afterwards:
#   journalctl -u northpole-height -f        live output (Ctrl+C to stop watching)
#   sudo systemctl restart northpole-height  restart (e.g. after git pull)
#   sudo systemctl stop northpole-height     stop (e.g. to run height.py by hand)
set -euo pipefail

SERVICE=northpole-height
HERE="$(cd "$(dirname "$0")" && pwd)"
PY="$HOME/rs/bin/python"
[ -x "$PY" ] || { echo "missing $PY (the Python venv)"; exit 1; }

sudo tee /etc/systemd/system/$SERVICE.service >/dev/null <<EOF
[Unit]
Description=North Pole height sensor (RealSense on the Rock64 -> ESP32)

[Service]
User=$USER
SupplementaryGroups=video dialout
WorkingDirectory=$HERE
ExecStart=$PY -u $HERE/height.py
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

# don't fight a copy started by hand over the serial port
pkill -f "height.py" || true

sudo systemctl daemon-reload
sudo systemctl enable --now $SERVICE
sleep 2
systemctl --no-pager status $SERVICE | head -n 5
echo
echo "Live output:  journalctl -u $SERVICE -f"
