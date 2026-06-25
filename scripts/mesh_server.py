#!/usr/bin/env python3
"""
简单的 3D 模型 HTTP 服务器
用法: python3 mesh_server.py [port] [mesh_dir]
"""

import http.server
import json
import os
import re
import sys
from pathlib import Path

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
MESH_DIR = Path(sys.argv[2]) if len(sys.argv) > 2 else Path("/tmp/meshes")
VIEWER_HTML = Path(__file__).parent / "web_viewer.html"


def build_file_list():
    """返回 {cloud: [...], mesh: [...]}"""
    files = {'cloud': [], 'mesh': []}
    if MESH_DIR.exists():
        for f in sorted(MESH_DIR.iterdir()):
            if not f.is_file():
                continue
            m = re.match(r'(cloud|mesh)_(\d+)\.ply', f.name)
            if m:
                files[m.group(1)].append({
                    'name': f.name,
                    'size': f.stat().st_size,
                })
    return files


class Handler(http.server.BaseHTTPRequestHandler):

    def log_message(self, fmt, *args):
        # 简洁日志
        print(f"  [{self.command}] {self.path} -> {args[0] if args else ''}")

    def do_GET(self):
        path = self.path.split('?')[0]  # 去掉 query string
        print(f"  DEBUG: raw_path={repr(self.path)} cleaned={repr(path)}")

        # ---- /list ----
        if path == '/list':
            data = json.dumps(build_file_list()).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', len(data))
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(data)
            return

        # ---- / 或 /index.html ----
        if path in ('/', '/index.html'):
            if VIEWER_HTML.exists():
                body = VIEWER_HTML.read_bytes()
            else:
                body = b"<h1>FAST-LIO2 Viewer</h1><p>web_viewer.html not found</p>"
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', len(body))
            self.end_headers()
            self.wfile.write(body)
            return

        # ---- 静态文件 (PLY) ----
        filepath = MESH_DIR / path.lstrip('/')
        if filepath.exists() and filepath.is_file():
            body = filepath.read_bytes()
            self.send_response(200)
            if filepath.suffix == '.ply':
                self.send_header('Content-Type', 'application/octet-stream')
            elif filepath.suffix == '.html':
                self.send_header('Content-Type', 'text/html; charset=utf-8')
            else:
                self.send_header('Content-Type', 'application/octet-stream')
            self.send_header('Content-Length', len(body))
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(body)
            return

        # ---- 404 ----
        self.send_response(404)
        self.end_headers()
        self.wfile.write(b'404 Not Found')


if __name__ == '__main__':
    MESH_DIR.mkdir(parents=True, exist_ok=True)

    print(f"============================================")
    print(f"  FAST-LIO2 实时三维查看器")
    print(f"============================================")
    print(f"  Mesh 目录:  {MESH_DIR}")
    print(f"  地址:      http://localhost:{PORT}")
    print(f"  API:       http://localhost:{PORT}/list")
    print(f"  快捷键:    1=点云 2=网格 3=混合")
    print(f"             O=全局 F=漫游 T=俯视")
    print(f"============================================")

    httpd = http.server.HTTPServer(('0.0.0.0', PORT), Handler)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down.")
        httpd.server_close()
