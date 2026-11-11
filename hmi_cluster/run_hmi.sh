#!/bin/bash
AUTH_FILE=$(ps aux | grep -oP '(?<=-auth )\S+' | head -n 1)
if [ -n "$AUTH_FILE" ] && [ -f "$AUTH_FILE" ]; then
    sudo cp "$AUTH_FILE" /home/ubuntu/.Xauthority
    sudo chown ubuntu:ubuntu /home/ubuntu/.Xauthority
fi
export DISPLAY=:0
exec ~/qnx_hmi_cluster/build/qnx_hmi_cluster
