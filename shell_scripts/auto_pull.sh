#!/bin/bash

REPO_PATH="/path/to/your/repo"
cd "$REPO_PATH"

while true; do
    echo "[AutoPull] $(date): pulling repo..."
    git pull
    sleep 300   # 300 seconds = 5 minutes
done
