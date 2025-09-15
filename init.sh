#!/bin/bash

# Isaac@Factory プロジェクト - Isaac Sim 5.0.0 と ROS 2 Humble 統合環境
# 初期化スクリプト

set -e  # エラー時に停止

echo "=========================================="
echo "Isaac@Factory プロジェクト - 初期化スクリプト"
echo "Isaac Sim 5.0.0 + ROS 2 Humble + Python 3.11"
echo "=========================================="

# 色付きの出力関数
print_info() {
    echo -e "\033[1;34m[INFO]\033[0m $1"
}

print_success() {
    echo -e "\033[1;32m[SUCCESS]\033[0m $1"
}

print_warning() {
    echo -e "\033[1;33m[WARNING]\033[0m $1"
}

print_error() {
    echo -e "\033[1;31m[ERROR]\033[0m $1"
}

# 1. 永続ストレージディレクトリの作成
create_directories() {
    print_info "Isaac Sim の永続ストレージディレクトリを確認しています..."
    
    # isaac-simディレクトリが存在し、ファイルがある場合はスキップ
    if [ -d "./isaac-sim" ] && [ "$(ls -A ./isaac-sim 2>/dev/null)" ]; then
        print_info "既存のisaac-simディレクトリが見つかりました。スキップします。"
        return
    fi
    
    print_info "Isaac Sim の永続ストレージディレクトリを作成しています..."
    
    # ディレクトリ構造を作成
    mkdir -p ./isaac-sim/{cache/kit,cache/ov,cache/pip,cache/glcache,cache/computecache,logs,data,documents,config}
    
    # DDS設定ディレクトリの確認
    if [ ! -d "./dds-config" ]; then
        print_warning "dds-config ディレクトリが見つかりません。作成します..."
        mkdir -p ./dds-config
    fi
    
    # 権限を設定（エラーを無視）
    chmod -R 755 ./isaac-sim 2>/dev/null || true
    chmod -R 755 ./dds-config 2>/dev/null || true
    
    print_success "ディレクトリの作成が完了しました"
}

# 2. 必要なファイルの確認
check_required_files() {
    print_info "必要なファイルの存在を確認しています..."
    
    local missing_files=()
    
    # 必須ファイルの確認
    if [ ! -f "./Dockerfile" ]; then
        missing_files+=("Dockerfile")
    fi
    
    if [ ! -f "./dds-config/cyclonedds.xml" ]; then
        missing_files+=("dds-config/cyclonedds.xml")
    fi
    
    if [ ! -d "./IsaacSim-ros_workspaces" ]; then
        missing_files+=("IsaacSim-ros_workspaces/")
    fi
    
    if [ ${#missing_files[@]} -gt 0 ]; then
        print_error "以下のファイル/ディレクトリが見つかりません："
        for file in "${missing_files[@]}"; do
            echo "  - $file"
        done
        print_error "必要なファイルを準備してから再実行してください"
        exit 1
    fi
    
    print_success "必要なファイルの確認が完了しました"
}

# 3. Docker イメージのビルド
build_docker_image() {
    print_info "Docker イメージをビルドしています..."
    print_warning "この処理には約60-90分かかります（Python 3.11 + ROS 2 ソースビルド）..."
    
    # タグを latest に設定
    local image_tag="isaac_factory:latest"
    
    # 既存のイメージを確認
    if docker images | grep -q "$image_tag"; then
        print_warning "既に $image_tag イメージが存在します"
        read -p "再ビルドしますか？ (y/N): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            print_info "既存のイメージを使用します"
            return
        fi
    fi
    
    # Docker イメージをビルド（メモリ制限を設定）
    print_info "Docker ビルドを開始します（メモリ使用量に注意してください）..."
    print_info "ビルドタグ: $image_tag"
    docker build --memory=8g --memory-swap=16g -t "$image_tag" .
    
    # ビルド結果を確認
    if docker images | grep -q "isaac_factory.*latest"; then
        print_success "Docker イメージのビルドが完了しました"
        print_info "イメージタグ: $image_tag"
    else
        print_error "Docker イメージのビルドに失敗しました"
        exit 1
    fi
}

# 4. スクリプトの実行権限を付与
set_script_permissions() {
    print_info "スクリプトに実行権限を付与しています..."
    
    # 存在するスクリプトファイルのみ権限を設定
    local scripts=("run_isaac_sim_docker.sh" "connect_to_container.sh" "stop_container.sh" "restart_container.sh" "finstall.sh")
    
    for script in "${scripts[@]}"; do
        if [ -f "./$script" ]; then
            chmod +x "./$script"
            print_info "  $script に実行権限を付与しました"
        else
            print_warning "  $script が見つかりません"
        fi
    done
    
    print_success "スクリプトの権限設定が完了しました"
}

# 5. 環境変数の設定ガイド
setup_environment() {
    print_info "環境変数の設定ガイドを表示しています..."
    
    echo ""
    echo "=========================================="
    echo "環境変数の設定が必要です"
    echo "=========================================="
    echo ""
    echo "以下のコマンドで DISPLAY 環境変数を設定してください："
    echo ""
    echo "例："
    echo "export DISPLAY=192.168.100.21:0"
    echo ""
    echo "または、.bashrc に追加して永続化："
    echo "echo 'export DISPLAY=192.168.100.21:0' >> ~/.bashrc"
    echo "source ~/.bashrc"
    echo ""
    echo "注意：IPアドレスは実際のローカルPCのIPアドレスに変更してください"
    echo ""
    echo "=========================================="
    echo "システム要件"
    echo "=========================================="
    echo ""
    echo "推奨システム要件："
    echo "- RAM: 16GB以上（ビルド時は32GB推奨）"
    echo "- ストレージ: 100GB以上の空き容量"
    echo "- GPU: NVIDIA GPU（CUDA対応）"
    echo "- Docker: 8GB以上のメモリ制限設定"
    echo ""
}

# 6. 使用方法の表示
show_usage() {
    print_info "使用方法を表示しています..."
    
    echo ""
    echo "=========================================="
    echo "使用方法"
    echo "=========================================="
    echo ""
    echo "1. コンテナの起動（バックグラウンド実行）："
    echo "   ./run_isaac_sim_docker.sh"
    echo ""
    echo "2. コンテナに接続："
    echo "   ./connect_to_container.sh"
    echo ""
    echo "3. コンテナの管理："
    echo "   ./restart_container.sh  # 再起動"
    echo "   ./stop_container.sh     # 停止・削除"
    echo ""
    echo "4. Isaac Sim の起動（コンテナ内で実行）："
    echo "   runapp                  # GUI モード"
    echo "   runheadless             # ヘッドレスモード"
    echo ""
    echo "5. ROS 2 の使用（コンテナ内で実行）："
    echo "   ros2 topic list         # トピック一覧"
    echo "   ros2 node list          # ノード一覧"
    echo "   rviz2                   # RViz2 起動"
    echo ""
    echo "6. Python 3.11 の使用："
    echo "   python3 --version       # Python バージョン確認"
    echo "   pip list                # インストール済みパッケージ確認"
    echo ""
}

# メイン処理
main() {
    echo ""
    print_info "初期化を開始します..."
    echo ""
    
    # 各処理を実行
    create_directories
    check_required_files
    build_docker_image
    set_script_permissions
    setup_environment
    show_usage
    
    echo ""
    print_success "初期化が完了しました！"
    echo ""
    print_info "次のステップ："
    echo "1. DISPLAY 環境変数を設定してください"
    echo "2. ./run_isaac_sim_docker.sh でコンテナを起動してください"
    echo "3. ./connect_to_container.sh でコンテナに接続してください"
    echo ""
    print_warning "注意：初回ビルドには大量のメモリと時間が必要です"
    echo ""
}

# スクリプトの実行
main "$@"
