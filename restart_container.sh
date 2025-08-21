#!/bin/bash
# restart_container.sh
#
# このスクリプトは、Isaac Simコンテナを再起動するためのスクリプトです。
# コンテナは削除されず、再起動のみを行います。

CONTAINER_NAME="isaac-sim-ws"

# コンテナが存在するかチェック
if ! docker ps -a --format "table {{.Names}}" | grep -q "^${CONTAINER_NAME}$"; then
    echo "❌ コンテナ ${CONTAINER_NAME} が見つかりません。"
    echo "まずコンテナを開始してください: ./run_isaac_sim_docker.sh"
    exit 1
fi

# コンテナが実行中かチェック
if docker ps --format "table {{.Names}}" | grep -q "^${CONTAINER_NAME}$"; then
    echo "🔄 コンテナ ${CONTAINER_NAME} を再起動しています..."
    docker restart ${CONTAINER_NAME}
else
    echo "🚀 停止中のコンテナ ${CONTAINER_NAME} を開始しています..."
    docker start ${CONTAINER_NAME}
fi

if [ $? -eq 0 ]; then
    echo "✅ コンテナが再起動されました。"
    echo ""
    echo "🔗 コンテナに接続するには:"
    echo "   ./connect_to_container.sh"
else
    echo "❌ コンテナの再起動に失敗しました。"
    exit 1
fi
