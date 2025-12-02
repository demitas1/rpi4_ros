#!/bin/bash
# ROS2コンテナ停止スクリプト

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "ROS2 Kiltedコンテナを停止しています..."
docker compose down

if [ $? -eq 0 ]; then
    echo "コンテナが停止しました"
else
    echo "エラー: コンテナの停止に失敗しました"
    exit 1
fi
