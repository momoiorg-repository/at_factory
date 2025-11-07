#!/bin/bash
# stop_container.sh
#
# このスクリプトは、Isaac Simコンテナを停止し、完全に初期化するためのスクリプトです。

CONTAINER_NAME="at-factory-jazzy"

# コンテナが存在するかチェック
if ! docker ps -a --format "table {{.Names}}" | grep -q "^${CONTAINER_NAME}$"; then
    echo "❌ コンテナ ${CONTAINER_NAME} が見つかりません。"
    exit 1
fi

# コンテナが実行中かチェック
if docker ps --format "table {{.Names}}" | grep -q "^${CONTAINER_NAME}$"; then
    echo "🛑 コンテナ ${CONTAINER_NAME} を停止しています..."
    docker stop ${CONTAINER_NAME}
    
    if [ $? -eq 0 ]; then
        echo "✅ コンテナが停止されました。"
    else
        echo "❌ コンテナの停止に失敗しました。"
        exit 1
    fi
fi

# コンテナを削除して完全初期化
echo "🗑️  コンテナ ${CONTAINER_NAME} を削除しています..."
docker rm ${CONTAINER_NAME}

if [ $? -eq 0 ]; then
    echo "✅ コンテナが削除され、完全に初期化されました。"
    echo ""
    echo "🚀 新しいコンテナを開始するには:"
    echo "   ./run_isaac_sim_docker.sh"
else
    echo "❌ コンテナの削除に失敗しました。"
    exit 1
fi
