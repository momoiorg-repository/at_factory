#!/bin/bash
# connect_to_container.sh
#
# このスクリプトは、実行中のIsaac Simコンテナに接続するためのスクリプトです。

CONTAINER_NAME="at-factory-jazzy"

# コンテナが存在するかチェック
if ! docker ps -a --format "table {{.Names}}" | grep -q "^${CONTAINER_NAME}$"; then
    echo "❌ コンテナ ${CONTAINER_NAME} が見つかりません。"
    echo "まずコンテナを開始してください: ./run_isaac_sim_docker.sh"
    exit 1
fi

# コンテナが実行中かチェック
if ! docker ps --format "table {{.Names}}" | grep -q "^${CONTAINER_NAME}$"; then
    echo "❌ コンテナ ${CONTAINER_NAME} が停止しています。"
    echo "コンテナを開始してください: ./run_isaac_sim_docker.sh"
    exit 1
fi

echo "🔗 コンテナ ${CONTAINER_NAME} に接続しています..."
echo "💡 コンテナを終了するには 'exit' と入力してください。"
echo "   （コンテナ自体は停止しません）"
echo ""

# コンテナに接続
docker exec -it ${CONTAINER_NAME} /bin/bash
