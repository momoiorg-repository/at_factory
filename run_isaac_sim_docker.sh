#!/bin/bash
# run_isaac_sim_docker.sh
#
# このスクリプトは、Isaac Sim (Jazzy) コンテナをバックグラウンドで実行し続けるためのスクリプトです。
# エディタを閉じてもコンテナは停止しません。

# --- Config ---
# コンテナ名とイメージタグを設定
CONTAINER_NAME="at-factory-jazzy"
IMAGE_TAG="at_factory:jazzy"
# ---

# サーバのローカルXサーバーへのアクセスを許可
xhost +

# 既存のコンテナが存在するかチェック
if docker ps -a --format "table {{.Names}}" | grep -q "^${CONTAINER_NAME}$"; then
    echo "コンテナ ${CONTAINER_NAME} が既に存在します。"
    
    # コンテナが実行中かチェック
    if docker ps --format "table {{.Names}}" | grep -q "^${CONTAINER_NAME}$"; then
        echo "コンテナは既に実行中です。"
        echo "コンテナに接続するには: ./connect_to_container.sh"
        exit 0
    else
        echo "停止中のコンテナ ${CONTAINER_NAME} を開始します..."
        docker start ${CONTAINER_NAME}
        echo "コンテナが開始されました。"
        echo "コンテナに接続するには: ./connect_to_container.sh"
        exit 0
    fi
fi

echo "Isaac Sim (Jazzy) コンテナをバックグラウンドで開始しています..."
echo "イメージ: $IMAGE_TAG"
echo "コンテナ名: $CONTAINER_NAME"

# バックグラウンドでコンテナを実行（-d オプションでデタッチモード）
docker run --name ${CONTAINER_NAME} -d \
  --runtime=nvidia \
  --gpus all \
  -e "ACCEPT_EULA=Y" \
  -e "PRIVACY_CONSENT=Y" \
  --network host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e QT_GRAPHICSSYSTEM=native \
  -e QT_QPA_PLATFORM=xcb \
  -e QT_AUTO_SCREEN_SCALE_FACTOR=1 \
  -e QT_SCALE_FACTOR=1 \
  -e QT_FONT_DPI=96 \
  -e LIBGL_ALWAYS_SOFTWARE=0 \
  -e MESA_GL_VERSION_OVERRIDE=4.5 \
  -e MESA_GLSL_VERSION_OVERRIDE=450 \
  -e __GL_SYNC_TO_VBLANK=0 \
  -e __GL_THREADED_OPTIMIZATIONS=1 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e ROS_DOMAIN_ID=31 \
  -v ./isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
  -v ./isaac-sim/cache/ov:/root/.cache/ov:rw \
  -v ./isaac-sim/cache/pip:/root/.cache/pip:rw \
  -v ./isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
  -v ./isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
  -v ./isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
  -v ./isaac-sim/data:/root/.local/share/ov/data:rw \
  -v ./isaac-sim/documents:/root/isaac-sim/Documents:rw \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $HOME/.Xauthority:/root/.Xauthority:ro \
  -v ~/Documents:/root/Documents:rw \
  --entrypoint /bin/bash \
  "$IMAGE_TAG" \
  -c "tail -f /dev/null"

# コンテナの起動を確認
if [ $? -eq 0 ]; then
    echo "✅ Isaac Sim (Jazzy) コンテナがバックグラウンドで開始されました。"
    echo "📋 コンテナ名: ${CONTAINER_NAME}"
    echo ""
    echo "🔗 コンテナに接続するには:"
    echo "   ./connect_to_container.sh"
    echo ""
    echo "📊 コンテナの状態を確認するには:"
    echo "   docker ps"
    echo ""
    echo "🛑 コンテナを停止するには:"
    echo "   ./stop_container.sh"
else
    echo "❌ コンテナの起動に失敗しました。"
    exit 1
fi