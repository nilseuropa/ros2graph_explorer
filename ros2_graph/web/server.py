from __future__ import annotations

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from typing import Optional, TYPE_CHECKING

if TYPE_CHECKING:  # pragma: no cover
    from ..ros2_graph_node import GraphSnapshot


class _ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class GraphWebServer:
    """Lightweight HTTP server that serves graph data and a simple UI."""

    def __init__(self, host: str, port: int, logger) -> None:
        self._host = host
        self._port = port
        self._logger = logger
        self._lock = threading.Lock()
        self._latest_payload: Optional[str] = None
        self._server = _ThreadingHTTPServer((host, port), self._create_handler())
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._running = False

    def _create_handler(self):
        parent = self

        class GraphRequestHandler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:
                path = self.path.split('?', 1)[0]
                if path in ('/', '/index.html'):
                    parent._serve_index(self)
                elif path == '/graph':
                    parent._serve_graph(self)
                elif path == '/healthz':
                    parent._serve_health(self)
                else:
                    self.send_error(404, 'Not Found')

            def log_message(self, format: str, *args) -> None:  # noqa: A003
                parent._logger.debug(f'web: {format % args}')

        return GraphRequestHandler

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread.start()
        self._logger.info(
            f'web UI available at http://{self._host}:{self._port} '
            '(open in a browser for live updates)'
        )

    def stop(self) -> None:
        if not self._running:
            return
        self._server.shutdown()
        self._server.server_close()
        self._thread.join(timeout=2.0)
        self._running = False

    def publish(self, snapshot: 'GraphSnapshot', fingerprint: str) -> None:
        payload = {
            'fingerprint': fingerprint,
            'generated_at': time.time(),
            'graph': snapshot.to_dict(),
        }
        payload_json = json.dumps(payload, separators=(',', ':'))
        with self._lock:
            self._latest_payload = payload_json

    def _latest(self) -> Optional[str]:
        with self._lock:
            return self._latest_payload

    def _serve_index(self, handler: BaseHTTPRequestHandler) -> None:
        content = INDEX_HTML.encode('utf-8')
        handler.send_response(200)
        handler.send_header('Content-Type', 'text/html; charset=utf-8')
        handler.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
        handler.send_header('Content-Length', str(len(content)))
        handler.end_headers()
        handler.wfile.write(content)

    def _serve_graph(self, handler: BaseHTTPRequestHandler) -> None:
        payload = self._latest()
        if payload is None:
            message = b'{"error":"graph not ready yet"}'
            handler.send_response(503)
            handler.send_header('Content-Type', 'application/json')
            handler.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            handler.send_header('Content-Length', str(len(message)))
            handler.end_headers()
            handler.wfile.write(message)
            return

        data = payload.encode('utf-8')
        handler.send_response(200)
        handler.send_header('Content-Type', 'application/json')
        handler.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
        handler.send_header('Content-Length', str(len(data)))
        handler.end_headers()
        handler.wfile.write(data)

    def _serve_health(self, handler: BaseHTTPRequestHandler) -> None:
        handler.send_response(200)
        handler.send_header('Content-Type', 'text/plain')
        handler.send_header('Cache-Control', 'no-cache')
        handler.end_headers()
        handler.wfile.write(b'ok')


INDEX_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>ros2_graph live view</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #0d1117;
      --panel: #161b22;
      --text: #e6edf3;
      --accent: #3fb950;
      --accent-secondary: #58a6ff;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: 'Segoe UI', system-ui, -apple-system, sans-serif;
      background: var(--bg);
      color: var(--text);
      display: flex;
      flex-direction: column;
      min-height: 100vh;
    }
    header {
      padding: 1rem;
      background: var(--panel);
      border-bottom: 1px solid rgba(255, 255, 255, 0.1);
      display: flex;
      flex-wrap: wrap;
      gap: 1rem;
      align-items: center;
    }
    header h1 {
      font-size: 1.1rem;
      margin: 0;
      letter-spacing: 0.03em;
      text-transform: uppercase;
    }
    #meta {
      flex: 1;
      font-size: 0.9rem;
    }
    #status {
      font-size: 0.85rem;
      opacity: 0.8;
    }
    #graphCanvas {
      flex: 1;
      width: 100%;
      height: calc(100vh - 120px);
      background: var(--bg);
      display: block;
    }
    #footer {
      padding: 0.5rem 1rem;
      font-size: 0.8rem;
      opacity: 0.7;
    }
    button {
      background: var(--accent-secondary);
      border: none;
      color: var(--bg);
      padding: 0.4rem 0.8rem;
      border-radius: 4px;
      cursor: pointer;
      font-weight: 600;
    }
    button:disabled {
      opacity: 0.4;
      cursor: not-allowed;
    }
  </style>
</head>
<body>
  <header>
    <h1>ros2_graph</h1>
    <div id="meta">waiting for data…</div>
    <div id="status">initializing…</div>
    <button id="refreshBtn" type="button">Refresh now</button>
  </header>
  <canvas id="graphCanvas"></canvas>
  <div id="footer">Data source: /graph endpoint (refresh every 2s)</div>
  <script>
    const canvas = document.getElementById('graphCanvas');
    const ctx = canvas.getContext('2d');
    const metaEl = document.getElementById('meta');
    const statusEl = document.getElementById('status');
    const refreshBtn = document.getElementById('refreshBtn');
    let lastGraph = null;

    function clipLabel(text, maxLen = 24) {
      if (!text) {
        return '';
      }
      return text.length > maxLen ? text.slice(0, maxLen - 1) + '…' : text;
    }

    function resizeCanvas() {
      canvas.width = window.innerWidth;
      canvas.height = Math.max(240, window.innerHeight - document.querySelector('header').offsetHeight - 40);
      renderGraph(lastGraph);
    }

    window.addEventListener('resize', resizeCanvas);
    refreshBtn.addEventListener('click', fetchGraph);

    function renderGraph(graph) {
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      if (!graph) {
        return;
      }
      lastGraph = graph;
      const width = canvas.width;
      const height = canvas.height;
      const nodeNames = graph.nodes || [];
      const topicNames = Object.keys(graph.topics || {});

      const positions = {};
      const nodeY = height * 0.3;
      const topicY = height * 0.7;

      function allocate(names, y) {
        const count = names.length;
        names.forEach((name, idx) => {
          const x = count > 1 ? ((idx + 1) / (count + 1)) * width : width / 2;
          positions[name] = { x, y };
        });
      }

      allocate(nodeNames, nodeY);
      allocate(topicNames, topicY);

      function getPos(name) {
        if (!positions[name]) {
          positions[name] = { x: width / 2, y: height / 2 };
        }
        return positions[name];
      }

      ctx.lineWidth = 1.5;
      ctx.strokeStyle = '#8b949e';
      ctx.fillStyle = '#8b949e';
      (graph.edges || []).forEach(edge => {
        const start = getPos(edge.start);
        const end = getPos(edge.end);
        drawArrow(start, end);
        if (edge.qos) {
          const midX = (start.x + end.x) / 2;
          const midY = (start.y + end.y) / 2;
          ctx.save();
          ctx.fillStyle = '#9e6ffe';
          ctx.font = '11px monospace';
          ctx.fillText(edge.qos, midX + 4, midY - 2);
          ctx.restore();
        }
      });

      nodeNames.forEach(name => drawNode(name, positions[name], '#58a6ff'));
      topicNames.forEach(name => drawTopic(name, positions[name], '#3fb950'));
    }

    function drawArrow(start, end) {
      ctx.beginPath();
      ctx.moveTo(start.x, start.y);
      ctx.lineTo(end.x, end.y);
      ctx.stroke();
      const angle = Math.atan2(end.y - start.y, end.x - start.x);
      const headLen = 9;
      ctx.beginPath();
      ctx.moveTo(end.x, end.y);
      ctx.lineTo(end.x - headLen * Math.cos(angle - Math.PI / 6), end.y - headLen * Math.sin(angle - Math.PI / 6));
      ctx.lineTo(end.x - headLen * Math.cos(angle + Math.PI / 6), end.y - headLen * Math.sin(angle + Math.PI / 6));
      ctx.closePath();
      ctx.fill();
    }

    function drawNode(name, pos, color) {
      ctx.save();
      ctx.fillStyle = color;
      ctx.beginPath();
      ctx.arc(pos.x, pos.y, 14, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = '#0d1117';
      ctx.font = '11px sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(clipLabel(name), pos.x, pos.y - 18);
      ctx.restore();
    }

    function drawTopic(name, pos, color) {
      ctx.save();
      ctx.fillStyle = color;
      ctx.fillRect(pos.x - 18, pos.y - 12, 36, 24);
      ctx.fillStyle = '#0d1117';
      ctx.font = '11px sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(clipLabel(name), pos.x, pos.y - 18);
      ctx.restore();
    }

    async function fetchGraph() {
      refreshBtn.disabled = true;
      try {
        const response = await fetch('/graph?ts=' + Date.now(), { cache: 'no-store' });
        if (!response.ok) {
          throw new Error('HTTP ' + response.status);
        }
        const payload = await response.json();
        const graph = payload.graph;
        const nodes = graph.nodes?.length ?? 0;
        const topics = Object.keys(graph.topics || {}).length;
        const edges = graph.edges?.length ?? 0;
        metaEl.textContent =
          'nodes: ' + nodes +
          ' | topics: ' + topics +
          ' | edges: ' + edges +
          ' | fingerprint: ' + payload.fingerprint;
        statusEl.textContent = 'Last update: ' + new Date(payload.generated_at * 1000).toLocaleTimeString();
        renderGraph(graph);
      } catch (err) {
        statusEl.textContent = 'Error fetching data: ' + err.message;
      } finally {
        refreshBtn.disabled = false;
      }
    }

    resizeCanvas();
    fetchGraph();
    setInterval(fetchGraph, 2000);
  </script>
</body>
</html>
"""
