#!/bin/bash
# serve_meshes.sh — 启动 3D 模型 Web 查看器
# 自动同步 web_viewer.html 到 /tmp/meshes/ 并启动 HTTP 服务

MESHES_DIR="/tmp/meshes"
VIEWER_HTML="$HOME/mine_project/src/FAST_LIO/scripts/web_viewer.html"
PORT=${1:-8080}

# 确保 meshes 目录存在
mkdir -p "$MESHES_DIR"

# 同步最新 web_viewer
cp "$VIEWER_HTML" "$MESHES_DIR/index.html"

echo "============================================"
echo "  FAST-LIO2 实时三维查看器"
echo "============================================"
echo "  Mesh 目录: $MESHES_DIR"
echo "  地址:      http://localhost:$PORT"
echo "  快捷键:    1=点云 2=网格 3=混合"
echo "             O=全局 F=漫游 T=俯视"
echo "============================================"

cd "$MESHES_DIR" && python3 -m http.server "$PORT"
