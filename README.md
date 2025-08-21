# Isaac Sim 5.0.0 と ROS 2 Humble 統合環境

このリポジトリには、Docker コンテナ内で NVIDIA Isaac Sim 5.0.0 と ROS 2 Humble 統合をセットアップするための設定ファイルとスクリプトが含まれています。

## 環境概要

### 含まれるコンポーネント
- **NVIDIA Isaac Sim 5.0.0**: 最新の物理シミュレーション環境
- **ROS 2 Humble**: ロボット開発フレームワーク


## 前提条件

### ハードウェア要件
- **GPU**: NVIDIA GPU（RTX 3060以上推奨）
- **メモリ**: 最低16GB RAM（32GB推奨）
- **ストレージ**: 最低50GBの空き容量
- **ディスプレイ**: X11対応ディスプレイ

### ソフトウェア要件
- **OS**: Ubuntu 22.04 LTS
- **Docker**: 最新版
- **NVIDIA Container Toolkit**: インストール済み
- **NVIDIA Driver**: 最新版（470以上）

### NVIDIA Container Toolkit のインストール
```bash
# NVIDIA Container Toolkit のインストール
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker
```

## セットアップ手順

### 1. リポジトリのクローン
```bash
git clone <repository-url> isaac_docker
cd isaac_docker
```

### 2. 永続ストレージディレクトリの作成
```bash
# Isaac Sim のキャッシュとデータ用ディレクトリを作成
mkdir -p ./isaac-sim/{cache/kit,cache/ov,cache/pip,cache/glcache,cache/computecache,logs,data,documents,config}

# 権限を設定
chmod -R 755 ./isaac-sim
```

### 3. Docker イメージのビルド
```bash
# Docker イメージをビルド（約30-60分かかります）
docker build -t prox_sim:5.0.0 .

# ビルドの進行状況を確認
docker images | grep prox_sim
```

### 4. 環境変数の設定
```bash
# .bashrc に環境変数を追加
export DISPLAY=<ローカルPCのIPアドレス>:0
source ~/.bashrc
```

## 使用方法

### コンテナの管理

#### 1. コンテナの起動（バックグラウンド実行）
```bash
# 実行権限を付与
chmod +x run_isaac_sim_docker.sh connect_to_container.sh stop_container.sh restart_container.sh

# コンテナをバックグラウンドで起動（エディタを閉じても継続）
./run_isaac_sim_docker.sh
```

#### 2. コンテナに接続
```bash
# 実行中のコンテナに接続
./connect_to_container.sh

# コンテナ内で 'exit' を入力してもコンテナは停止しません
```

#### 3. コンテナの管理
```bash
# コンテナを再起動（削除しない）
./restart_container.sh

# コンテナを停止して完全初期化（削除）
./stop_container.sh

# コンテナの状態確認
docker ps
```

#### 4. 手動でDockerコマンド実行（従来方式）
```bash
# X11サーバーへのアクセスを許可
xhost +

# コンテナを起動（エディタを閉じると停止）
docker run --name isaac-sim-ws -it --rm \
  --runtime=nvidia --gpus all \
  -e "ACCEPT_EULA=Y" \
  -e "PRIVACY_CONSENT=Y" \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $HOME/.Xauthority:/root/.Xauthority:ro \
  -v ./isaac-sim:/root/isaac-sim:rw \
  -v ./dev-toyota-ph1:/IsaacLab/scripts/dev-toyota-ph1:rw \
  prox_sim:5.0.0
```

### Isaac Sim の起動

#### GUI モード（ローカルディスプレイ）
```bash
# コンテナ内で実行
runapp
```

#### ヘッドレスモード（リモート接続）
```bash
# ヘッドレスサーバーを起動
runheadless

# 別のターミナルでクライアント接続
# Omniverse Streaming Client を使用
```

### ROS 2 の使用

#### ROS 2 環境の確認
```bash
# ROS 2 の環境変数を確認
echo $ROS_DISTRO
echo $RMW_IMPLEMENTATION

# ROS 2 のソース
source /opt/ros/humble/setup.bash
```

#### 基本的なROS 2 コマンド
```bash
# ROS 2 ノードの一覧
ros2 node list

# トピックの一覧
ros2 topic list

# サービス一覧
ros2 service list

# パラメータ一覧
ros2 param list
```

#### TurtleBot3 Lime の起動
```bash
# TurtleBot3 Lime シミュレーションを起動
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

# 別のターミナルでナビゲーションを起動
ros2 launch nav2_bringup bringup_launch.py
```

## プロジェクト構造

```
isaac_docker/
├── README.md                    # このファイル
├── Dockerfile                   # Isaac Sim + ROS 2 イメージ定義
├── run_isaac_sim_docker.sh     # コンテナ起動スクリプト（バックグラウンド）
├── connect_to_container.sh      # コンテナ接続スクリプト
├── stop_container.sh           # コンテナ停止・削除スクリプト
├── restart_container.sh        # コンテナ再起動スクリプト
├── LICENSE                      # MIT ライセンス
├── .gitignore                   # Git 除外設定
├── isaac-sim/                   # Isaac Sim 永続データ
│   ├── cache/                   # キャッシュファイル
│   ├── logs/                    # ログファイル
│   ├── data/                    # シミュレーションデータ
│   ├── documents/               # ドキュメント
│   └── config/                  # 設定ファイル
└── dev-toyota-ph1/             # Toyota PH1 プロジェクト
    ├── README.md               # プロジェクト説明
    ├── vlm_robot_controller.py # VLM ロボット制御
    ├── spot_env.py            # Spot 環境
    ├── captioned_image_viewer.py # 画像表示ツール
    └── requirements.txt        # Python 依存関係
```

## トラブルシューティング

### よくある問題と解決方法

#### 1. ディスプレイエラー
```bash
# X11サーバーへのアクセスを許可
xhost +

# ディスプレイ変数を確認
echo $DISPLAY
```

#### 2. GPU アクセスエラー
```bash
# NVIDIA Container Toolkit の確認
docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi

# Docker デーモンの再起動
sudo systemctl restart docker
```

#### 3. メモリ不足エラー
```bash
# Docker のメモリ制限を確認
docker system df

# 未使用リソースの削除
docker system prune -a
```

#### 4. ネットワークエラー
```bash
# ネットワーク設定の確認
docker network ls

# コンテナのネットワーク設定を確認
docker inspect isaac-sim-ws
```

### ログの確認
```bash
# Isaac Sim のログを確認
tail -f ./isaac-sim/logs/isaac-sim.log

# Docker コンテナのログを確認
docker logs isaac-sim-ws
```

## パフォーマンス最適化

### 環境変数設定
```bash
# GUI パフォーマンス設定
export QT_X11_NO_MITSHM=1
export QT_GRAPHICSSYSTEM=native
export QT_QPA_PLATFORM=xcb

# OpenGL パフォーマンス設定
export LIBGL_ALWAYS_SOFTWARE=0
export MESA_GL_VERSION_OVERRIDE=4.5
export __GL_SYNC_TO_VBLANK=0
export __GL_THREADED_OPTIMIZATIONS=1

# ROS 2 パフォーマンス設定
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=31
```

### システム設定
```bash
# GPU メモリの確認
nvidia-smi

# システムリソースの監視
htop
```

## 開発ガイド

### 新しいROS 2 パッケージの追加
```bash
# ワークスペースの作成
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# パッケージの作成
ros2 pkg create --build-type ament_python my_package

# ビルド
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Isaac Sim スクリプトの開発
```python
# Isaac Sim Python API の使用例
from omni.isaac.kit import SimulationApp

# シミュレーションアプリの初期化
simulation_app = SimulationApp({"headless": False})

# シミュレーションの実行
simulation_app.run()
```

## ライセンス

このプロジェクトは MIT ライセンスの下で提供されています。詳細は [LICENSE](LICENSE) ファイルをご覧ください。

## サポート

問題が発生した場合は、以下の手順でサポートを受けてください：

1. [トラブルシューティング](#トラブルシューティング) セクションを確認
2. ログファイルを確認
3. GitHub Issues で問題を報告

## 更新履歴

- **v1.0.0**: Isaac Sim 5.0.0 + ROS 2 Humble 統合環境の初期リリース
- **v1.1.0**: TurtleBot3 Lime サポートの追加
- **v1.2.0**: Toyota PH1 プロジェクトの統合
- **v1.3.0**: バックグラウンドコンテナ実行機能の追加
  - エディタを閉じてもコンテナが継続実行
  - コンテナ管理スクリプトの追加（接続・停止・再起動）
  - 完全初期化機能の実装
