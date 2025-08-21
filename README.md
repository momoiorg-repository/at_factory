# Toyota PH1 プロジェクト - Isaac Sim 5.0.0 と ROS 2 Humble 統合環境

このリポジトリには、Toyota PH1 プロジェクトのフェーズ1用として、Docker コンテナ内で NVIDIA Isaac Sim 5.0.0 と ROS 2 Humble 統合をセットアップするための設定ファイルとスクリプトが含まれています。

## 環境概要

### 含まれるコンポーネント
- **NVIDIA Isaac Sim 5.0.0**: 最新の物理シミュレーション環境
- **ROS 2 Humble**: ロボット開発フレームワーク

### Toyota PH1 プロジェクト（フェーズ1）
- **VLM Robot Controller**: ビジョン言語モデルベースのロボット制御システム
- **Spot Environment**: Boston Dynamics Spot ロボット用シミュレーション環境
- **Captioned Image Viewer**: 画像キャプション表示ツール


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
git clone https://github.com/Prox-Industries/dev-isaac-env-toyota.git isaac_docker
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
# 例： export DISPLAY=192.168.100.21:0
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

## ライセンス

このプロジェクトは MIT ライセンスの下で提供されています。詳細は [LICENSE](LICENSE) ファイルをご覧ください。


## 更新履歴

- **v1.0.0**: Isaac Sim 5.0.0 + ROS 2 Humble 統合環境の初期リリース
- **v1.1.0**: Toyota PH1 プロジェクトの統合
- **v1.2.0**: バックグラウンドコンテナ実行機能の追加
  - エディタを閉じてもコンテナが継続実行
  - コンテナ管理スクリプトの追加（接続・停止・再起動）
  - 完全初期化機能の実装
