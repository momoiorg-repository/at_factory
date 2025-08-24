#!/bin/bash

# Isaac@Factory プロジェクト - Isaac Sim 5.0.0 と ROS 2 Humble 統合環境
# 初期化スクリプト

set -e  # エラー時に停止

echo "=========================================="
echo "Isaac@Factory プロジェクト - 初期化スクリプト"
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
    print_info "Isaac Sim の永続ストレージディレクトリを作成しています..."
    
    # ディレクトリ構造を作成
    mkdir -p ./isaac-sim/{cache/kit,cache/ov,cache/pip,cache/glcache,cache/computecache,logs,data,documents,config}
    
    # 権限を設定
    chmod -R 755 ./isaac-sim
    
    print_success "ディレクトリの作成が完了しました"
}

# 2. Docker イメージのビルド
build_docker_image() {
    print_info "Docker イメージをビルドしています..."
    print_warning "この処理には約30-60分かかります..."
    
    # 既存のイメージを確認
    if docker images | grep -q "isaac_factory.*5.0.0"; then
        print_warning "既に isaac_factory:5.0.0 イメージが存在します"
        read -p "再ビルドしますか？ (y/N): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            print_info "既存のイメージを使用します"
            return
        fi
    fi
    
    # Docker イメージをビルド
    docker build -t isaac_factory:5.0.0 .
    
    # ビルド結果を確認
    if docker images | grep -q "isaac_factory.*5.0.0"; then
        print_success "Docker イメージのビルドが完了しました"
    else
        print_error "Docker イメージのビルドに失敗しました"
        exit 1
    fi
}

# 3. スクリプトの実行権限を付与
set_script_permissions() {
    print_info "スクリプトに実行権限を付与しています..."
    
    chmod +x run_isaac_sim_docker.sh connect_to_container.sh stop_container.sh restart_container.sh
    
    print_success "スクリプトの権限設定が完了しました"
}

# 4. 環境変数の設定ガイド
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
}

# 5. 使用方法の表示
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
}

# メイン処理
main() {
    echo ""
    print_info "初期化を開始します..."
    echo ""
    
    # 各処理を実行
    create_directories
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
    echo ""
}

# スクリプトの実行
main "$@"
