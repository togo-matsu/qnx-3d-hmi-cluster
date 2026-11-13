#!/bin/bash
AUTH_FILE=$(ps aux | grep -oP '(?<=-auth )\S+' | head -n 1)
if [ -n "$AUTH_FILE" ] && [ -f "$AUTH_FILE" ]; then
    sudo cp "$AUTH_FILE" /home/ubuntu/.Xauthority
    sudo chown ubuntu:ubuntu /home/ubuntu/.Xauthority
fi

export DISPLAY=:0
dbus-send --session --dest=org.gnome.Shell --type=method_call /org/gnome/Shell org.gnome.Shell.Eval string:'Main.overview.hide();' 2>/dev/null || true

cd "$(dirname "$0")/build"
python3 ../fetch_db_history.py || true
exec ./qnx_hmi_cluster
