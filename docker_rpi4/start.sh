#!/bin/bash
# ROS2コンテナ起動スクリプト

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "ROS2 Kiltedコンテナを起動しています..."
docker compose up -d

if [ $? -eq 0 ]; then
    echo "コンテナが起動しました"
    echo "コンテナに入るには: docker exec -it ros2_kilted_container bash"
else
    echo "エラー: コンテナの起動に失敗しました"
    exit 1
fi
