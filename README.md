# Docker: ROS2 for Raspberry Pi

Raspberry Pi 4向けのROS2開発環境をDockerコンテナで提供します。
ROS2 Jazzy (Ubuntu 24.04) をメインディストリビューションとして使用。

## 公開イメージ

| イメージ | Ubuntu | アーキテクチャ |
|---------|--------|---------------|
| `ghcr.io/demitas1/ros2_jazzy:latest` | 24.04 | amd64, arm64 | **推奨** |
| `ghcr.io/demitas1/ros2_kilted:latest` | 24.04 | amd64, arm64 |
| `ghcr.io/demitas1/ros2_humble:latest` | 22.04 | amd64, arm64 |

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

## クイックスタート

```bash
# コンテナ起動（ghcr.ioからイメージを取得）
bash docker_rpi4/start.sh

# コンテナに入る
docker exec -it ros2_jazzy_container bash

# コンテナ停止
bash docker_rpi4/stop.sh
```

## 開発ワークフロー

このリポジトリでは、ROS2ベースイメージとノードのソースコードを分離して管理します。

- **ベースイメージ**: ROS2環境のみを含む（GitHub Actionsで自動ビルド、ghcr.ioで公開）
- **ノード**: `ros2_ws/src/` にcloneして使用（ボリュームマウント）
- 対応アーキテクチャ: linux/amd64, linux/arm64

### 1. ワークスペースのセットアップ

```bash
cd docker_rpi4/ros2_ws/src

# 必要なノードをclone
git clone https://github.com/yourname/my_robot.git
git clone https://github.com/yourname/my_sensors.git
```

### 2. コンテナを起動

```bash
bash docker_rpi4/start.sh
```

### 3. コンテナ内でビルド

```bash
docker exec -it ros2_jazzy_container bash

# ROS2環境をsource（.bashrcで自動実行されるが明示的に記載）
source /opt/ros/jazzy/setup.bash

# 依存パッケージをインストール
rosdep install -i --from-path src --rosdistro jazzy -y

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

## ROS2 設定

- ビルドシステム: ament_python / ament_cmake with colcon
- ROS_DOMAIN_ID: 42（ネットワーク分離用）
- 新規パッケージは `docker_rpi4/ros2_ws/src/` にclone
- コンテナ内ユーザー: `ros2_user` (UID: 1000)
  - `build/`, `install/`, `log/` はホスト側でも通常ユーザー権限になります

## ディレクトリ構成

```
rpi4_ros2/
├── docker_rpi4/
│   ├── docker-compose.yml    # Docker Compose設定（ghcr.ioイメージ使用）
│   ├── Dockerfile.jazzy      # ROS2 Jazzy用（メイン）
│   ├── Dockerfile.kilted     # ROS2 Kilted用
│   ├── Dockerfile.humble     # ROS2 Humble用
│   ├── start.sh              # コンテナ起動（ghcr.ioからpull）
│   ├── stop.sh               # コンテナ停止
│   ├── build.sh              # ローカルビルド（通常不要）
│   ├── run_local.sh          # ローカルイメージで起動（通常不要）
│   └── ros2_ws/              # ROS2ワークスペース（マウント）
│       └── src/              # ノードをここにclone
├── .github/workflows/
│   └── docker-publish.yml    # GitHub Actions（自動ビルド・公開）
└── tools/                    # ユーティリティスクリプト
```

---

## ローカルビルド（上級者向け）

> ROS2ノードの開発のみであれば、このセクションは不要です。
> Dockerイメージ自体を修正する場合にのみ使用してください。

### ローカルでイメージをビルド

```bash
cd docker_rpi4
bash build.sh
```

### ローカルイメージでコンテナ起動

```bash
bash docker_rpi4/run_local.sh
```

### GitHub Actions

Dockerイメージは以下の条件で自動ビルド・公開されます：

- トリガー: `Dockerfile*` または `.github/workflows/docker-publish.yml` の変更時
- 対象ブランチ: master, develop
- 手動実行: workflow_dispatch で kilted/jazzy/humble/all を選択可能

---

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
