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

## クイックスタート（ビルド済みイメージを使用）

GitHub Container Registryから公開イメージを取得して使用できます。

```bash
# イメージを取得
docker pull ghcr.io/demitas1/ros2_kilted:latest

# または Jazzy/Humble
docker pull ghcr.io/demitas1/ros2_jazzy:latest
docker pull ghcr.io/demitas1/ros2_humble:latest
```

## ローカルでイメージをビルド

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

## 開発ワークフロー

このリポジトリでは、ROS2ベースイメージとノードのソースコードを分離して管理します。

- **ベースイメージ**: ROS2環境のみを含む（GitHub Actionsで自動ビルド）
- **ノード**: `ros2_ws/src/` にcloneして使用（ボリュームマウント）

### 1. ワークスペースのセットアップ

```bash
cd docker_rpi4/ros2_ws/src

# 必要なノードをclone
git clone https://github.com/yourname/my_robot.git
git clone https://github.com/yourname/my_sensors.git
```

### 2. コンテナを起動

```bash
cd docker_rpi4
bash start.sh
```

### 3. コンテナ内でビルド

```bash
docker exec -it ros2_kilted_container bash

# ROS2環境をsource
source /opt/ros/kilted/setup.bash

# 依存パッケージをインストール
rosdep install -i --from-path src --rosdistro kilted -y

# ビルド
colcon build

# ビルド成果物をsource
source install/setup.bash
```

### 4. ノードを実行

```bash
# 例: my_robotパッケージのmy_nodeを実行
ros2 run my_robot my_node
```

### 5. コード編集後の再ビルド

ホスト側でコードを編集後、コンテナ内で再ビルド:

```bash
# 特定パッケージのみ再ビルド
colcon build --packages-select my_robot

# 変更を反映
source install/setup.bash
```

## テストの実行

コンテナ内で:

```bash
colcon test --packages-select my_robot
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
│   └── ros2_ws/            # ROS2ワークスペース（マウント）
│       └── src/
│           ├── my_robot/   # git cloneしたノード
│           └── my_sensors/ # git cloneしたノード
└── tools/                  # ユーティリティスクリプト
```

## 公開イメージ

| イメージ | Ubuntu | アーキテクチャ |
|---------|--------|---------------|
| `ghcr.io/demitas1/ros2_kilted:latest` | 24.04 | amd64, arm64 |
| `ghcr.io/demitas1/ros2_jazzy:latest` | 24.04 | amd64, arm64 |
| `ghcr.io/demitas1/ros2_humble:latest` | 22.04 | amd64, arm64 |

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
