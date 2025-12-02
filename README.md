# Docker: ROS2 for Raspberry Pi

Raspberry Pi 4向けのROS2開発環境をDockerコンテナで提供します。

## Raspberry Piへのdockerインストール

```bash
# ダウンロードとインストール
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# ユーザーをdockerグループに追加
sudo usermod -aG docker $USER

# 確認
docker run hello-world
docker images
docker ps
```

## ビルド

```bash
cd docker_rpi4
bash build.sh
```

## コンテナの起動・停止

### Docker Compose を使用（推奨）

```bash
cd docker_rpi4

# 起動
bash start.sh

# コンテナに入る
docker exec -it ros2_kilted_container bash

# 停止
bash stop.sh
```

### 手動で起動

```bash
cd docker_rpi4
bash run.sh
docker exec -it ros2_kilted_container bash
```

## ROS2ノードの実行

コンテナ内で以下を実行:

```bash
source ./install/setup.bash

# Publisher
ros2 run py_pubsub talker

# Subscriber（別ターミナルで）
ros2 run py_pubsub listener
```

## パッケージの再ビルド

コンテナ内で:

```bash
source /opt/ros/kilted/setup.bash
cd /root/ros2_ws
colcon build --packages-select py_pubsub
source ./install/setup.bash
```

## テストの実行

コンテナ内で:

```bash
colcon test --packages-select py_pubsub
colcon test-result --verbose
```

## ディレクトリ構成

```
rpi4_ros2/
├── docker_rpi4/
│   ├── docker-compose.yml  # Docker Compose設定
│   ├── Dockerfile.kilted   # ROS2 Kilted用（メイン）
│   ├── Dockerfile.jazzy    # ROS2 Jazzy用
│   ├── Dockerfile.humble   # ROS2 Humble用（レガシー）
│   ├── build.sh            # イメージビルドスクリプト
│   ├── start.sh            # コンテナ起動スクリプト
│   ├── stop.sh             # コンテナ停止スクリプト
│   ├── run.sh              # 手動起動スクリプト
│   └── ros2_ws/            # ROS2ワークスペース
│       └── src/
│           └── py_pubsub/  # サンプルパッケージ
└── tools/                  # ユーティリティスクリプト
```

## Tools

### Raspberry PiのIPアドレス検索

`./tools/find_raspberrypi_en.sh` を使用してネットワークをスキャンします。

```bash
$ bash ./tools/find_raspberrypi_en.sh
Available interfaces:
     1  br-cafe0123babe
     2  docker0
     3  eno1
     4  lo
     5  wlx0011deadbeef
Enter the number of the interface you want to use: 5
[sudo] password for yourname:
The IP address of Raspberry Pi is 192.168.1.161.
```

## License

MIT
