# Raspberry Pi 4 セットアップ手順（手動）

Raspberry Pi 4 上に Docker をインストールし、`ghcr.io` で公開済みの ROS2 Jazzy
イメージを取得してコンテナを起動するまでの手順をまとめる。

ここではイメージを **ghcr.io から pull する方法のみ** を扱う（Pi 上でのローカル
ビルドは扱わない）。

## 前提

- Raspberry Pi 4（arm64 / aarch64）
- OS: Raspberry Pi OS（Debian 13 trixie ベース）
  - 別のコードネームの場合は後述のリポジトリ登録で読み替えること
- インターネット接続が有効
- `sudo` が実行可能なユーザー
- 動作確認時の構成例:
  - Docker Engine 29.6.1 / Docker Compose v5.2.0
  - 公開イメージ `ghcr.io/demitas1/ros2_jazzy:latest`（amd64 / arm64 対応）

> リモート（SSH）から実行する場合は、各コマンドの先頭に
> `ssh <ホスト名> '...'` を付けるか、`ssh <ホスト名>` でログインしてから実行する。

---

## 1. Docker のインストール

Docker 公式 apt リポジトリからインストールする。

### 1-1. リポジトリの登録

```bash
# 前提パッケージ
sudo apt-get update
sudo apt-get install -y ca-certificates curl

# GPG 鍵の取得
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/debian/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# リポジトリの登録（コードネームは OS に合わせる。trixie の例）
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/debian trixie stable" \
  | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt-get update
```

> **コードネームの確認**: `. /etc/os-release && echo "$VERSION_CODENAME"`
> で確認できる。`trixie` 以外の場合はその値に置き換える。利用可能な
> コードネームは <https://download.docker.com/linux/debian/dists/> を参照。

### 1-2. Docker 本体のインストール

```bash
sudo apt-get install -y \
  docker-ce docker-ce-cli containerd.io \
  docker-buildx-plugin docker-compose-plugin
```

### 1-3. サービスの起動と自動起動設定

```bash
sudo systemctl enable --now docker
systemctl is-active docker   # active と表示されれば OK
```

### 1-4. sudo なしで docker を使えるようにする

```bash
sudo usermod -aG docker "$USER"
```

> **重要**: グループ追加は **再ログイン後に有効** になる。
> SSH の場合は一度ログアウトして接続し直す。ローカルの場合は再ログイン
> （または再起動）する。反映後 `id -nG` の出力に `docker` が含まれる。

### 1-5. 動作確認

```bash
docker --version
docker run --rm hello-world   # "Hello from Docker!" が出れば成功
```

---

## 2. リポジトリのクローン

公開リポジトリのため HTTPS でクローンできる。

```bash
cd ~
git clone https://github.com/demitas1/rpi4_ros.git
cd rpi4_ros
```

クローン後、Docker 関連ファイルは `docker_rpi4/` 配下にある。

```bash
ls docker_rpi4/
# build.sh  docker-compose.yml  Dockerfile.*  run_local.sh  start.sh  stop.sh  ros2_ws/
```

---

## 3. コンテナの起動（ghcr.io から pull）

`start.sh` が `docker compose pull`（イメージ取得）と
`docker compose up -d`（バックグラウンド起動）をまとめて実行する。

```bash
cd ~/rpi4_ros/docker_rpi4
bash start.sh
```

`docker-compose.yml` の主な設定:

- イメージ: `ghcr.io/demitas1/ros2_jazzy:latest`
- コンテナ名: `ros2_jazzy_container`
- `network_mode: host` / `ROS_DOMAIN_ID=42`
- `./ros2_ws` を `/home/ros2_user/ros2_ws` にマウント

### 起動確認

```bash
docker ps --format "table {{.Names}}\t{{.Image}}\t{{.Status}}"
# ros2_jazzy_container ... Up ... と表示されれば成功
```

### 停止

```bash
cd ~/rpi4_ros/docker_rpi4
bash stop.sh
```

---

## 4. docker exec による ROS2 動作確認

### コンテナに入る

```bash
docker exec -it ros2_jazzy_container bash
```

対話シェルで入ると `.bashrc` により `/opt/ros/jazzy/setup.bash` が
自動で読み込まれる。

### コンテナ内での確認

```bash
whoami                    # ros2_user
echo $ROS_DISTRO          # jazzy
echo $ROS_DOMAIN_ID       # 42
ros2 pkg list | wc -l     # インストール済みパッケージ数（例: 192）
ros2 topic list           # /parameter_events  /rosout が表示される
ls /home/ros2_user/ros2_ws/src   # マウントされた py_pubsub が見える
```

> **補足**: `docker exec ros2_jazzy_container bash -lc '...'` のように
> 非対話で実行すると `.bashrc` が読まれず `ROS_DISTRO` が空になる。
> その場合はコマンド内で `source /opt/ros/jazzy/setup.bash` を明示する。

### ワンライナーでの確認（コンテナに入らず実行）

```bash
docker exec ros2_jazzy_container bash -lc \
  'source /opt/ros/jazzy/setup.bash && ros2 topic list'
```

---

## トラブルシュート

- **`docker` コマンドで permission denied**: グループ反映前。再ログインする
  （手順 1-4）。応急的には `sudo docker ...` で実行可能。
- **pull が失敗する / イメージが見つからない**: ネットワークと
  `ghcr.io/demitas1/ros2_jazzy:latest` の公開状態を確認する。
  `docker manifest inspect ghcr.io/demitas1/ros2_jazzy:latest` で
  arm64 マニフェストの有無を確認できる。
- **apt のリポジトリ登録でエラー**: OS のコードネームが Docker 公式
  リポジトリに存在するか確認する（手順 1-1 の注記）。
