from __future__ import annotations

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from importlib import resources
from socketserver import ThreadingMixIn
from typing import Dict, Optional, Tuple, TYPE_CHECKING

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
                if path in ('/', '/index.html', '/styles.css', '/app.js'):
                    parent._serve_static(self, path)
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

    def _serve_static(self, handler: BaseHTTPRequestHandler, path: str) -> None:
        rel_path, content_type = STATIC_FILES[path]
        try:
            data = resources.files(__package__).joinpath('static', rel_path).read_bytes()
        except FileNotFoundError:
            handler.send_error(404, 'Asset not found')
            return

        handler.send_response(200)
        handler.send_header('Content-Type', content_type)
        handler.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
        handler.send_header('Content-Length', str(len(data)))
        handler.end_headers()
        handler.wfile.write(data)

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


STATIC_FILES: Dict[str, Tuple[str, str]] = {
    '/': ('index.html', 'text/html; charset=utf-8'),
    '/index.html': ('index.html', 'text/html; charset=utf-8'),
    '/styles.css': ('styles.css', 'text/css; charset=utf-8'),
    '/app.js': ('app.js', 'application/javascript; charset=utf-8'),
}
