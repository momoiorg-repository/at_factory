# Isaac@Factory プロジェクト - Isaac Sim 5.0.0 と ROS 2 Humble 統合環境

このリポジトリには、Isaac@Factory プロジェクト用として、Docker コンテナ内で NVIDIA Isaac Sim 5.0.0 と ROS 2 Humble 統合をセットアップするための設定ファイルとスクリプトが含まれています。

## 環境概要

### 含まれるコンポーネント
- **NVIDIA Isaac Sim 5.0.0**: 最新の物理シミュレーション環境
- **ROS 2 Humble**: ロボット開発フレームワーク

### Isaac@Factory プロジェクト
- **Factory Environment**: 工場環境の3Dモデルとシミュレーション
- **USD Files**: Universal Scene Description ファイルによる工場シーン
- **Dual Arm Robot**: デュアルアームロボットのシミュレーション
- **Factory Assets**: 工場設備と環境のアセット

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
git clone https://github.com/momoiorg-repository/-factory.git isaac_factory
cd isaac_factory
```

### 2. 初期化スクリプトの実行
```bash
# 自動セットアップスクリプトを実行
./init.sh
```

このスクリプトは以下を自動実行します：
- Isaac Sim の永続ストレージディレクトリの作成
- Docker イメージのビルド（約30-60分）
- スクリプトの実行権限設定
- 環境変数の設定ガイド表示

### 3. 環境変数の設定
```bash
# 例： export DISPLAY=192.168.100.21:0
export DISPLAY=<ローカルPCのIPアドレス>:0
```

## 使用方法

### コンテナの管理

#### 1. コンテナの起動（バックグラウンド実行）
```bash
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
  -v ./factory_v1:/IsaacLab/scripts/factory_v1:rw \
  isaac_factory:5.0.0
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
isaac_factory/
├── README.md                    # このファイル
├── Dockerfile                   # Isaac Sim + ROS 2 イメージ定義
├── init.sh                      # 自動初期化スクリプト
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
└── factory_v1/                  # Isaac@Factory プロジェクト
    ├── Custom_Dual_arm.usd     # デュアルアームロボット
    ├── factory_base.usd        # 工場ベース環境
    ├── factory_v1base.usd      # 工場v1ベース
    ├── factory_v1area.usd      # 工場エリア1
    ├── factory_v1area2.usd     # 工場エリア2
    ├── factory_v1Tores.usd     # 工場トレス
    ├── ridgeback_franka-roscon.usd # Ridgeback + Franka ロボット
    ├── quicktrun.usdc          # クイックターン
    ├── bad_area.usdc           # 不良エリア
    ├── bad_area-mesh.usd       # 不良エリアメッシュ
    ├── Materials/              # マテリアルファイル
    └── .thumbs/                # サムネイルファイル
```

## Factory Environment の特徴

### USD ファイルの説明
- **Custom_Dual_arm.usd**: カスタムデュアルアームロボットの完全なモデル
- **factory_base.usd**: 工場の基本環境とレイアウト
- **factory_v1base.usd**: 工場v1の基本構造
- **factory_v1area.usd**: 工場エリア1の詳細環境
- **factory_v1area2.usd**: 工場エリア2の詳細環境
- **ridgeback_franka-roscon.usd**: Ridgeback移動ロボットとFrankaアームの統合

### シミュレーション機能
- **物理シミュレーション**: リアルタイム物理演算
- **ロボット制御**: ROS 2 ベースのロボット制御
- **環境インタラクション**: 工場環境との相互作用
- **マルチロボット**: 複数ロボットの同時シミュレーション

## ライセンス

このプロジェクトは MIT ライセンスの下で提供されています。詳細は [LICENSE](LICENSE) ファイルをご覧ください。

## 更新履歴

- **v1.0.0**: Isaac Sim 5.0.0 + ROS 2 Humble 統合環境の初期リリース
- **v1.1.0**: Isaac@Factory プロジェクトの統合
- **v1.2.0**: バックグラウンドコンテナ実行機能の追加
  - エディタを閉じてもコンテナが継続実行
  - コンテナ管理スクリプトの追加（接続・停止・再起動）
  - 完全初期化機能の実装
- **v2.0.0**: Isaac@Factory 環境への移行
  - Factory USD ファイルの統合
  - 自動初期化スクリプトの追加
