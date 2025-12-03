#!/bin/bash
# ROS2コンテナ起動スクリプト（ghcr.ioからイメージを取得）

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "ROS2 コンテナを起動しています（ghcr.io）..."
docker compose pull
docker compose up -d

if [ $? -eq 0 ]; then
    echo "コンテナが起動しました"
    echo "コンテナに入るには: docker exec -it ros2_jazzy_container bash"
else
    echo "エラー: コンテナの起動に失敗しました"
    exit 1
fi
