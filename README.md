# ros2_graph

Headless ROS 2 computation graph visualizer inspired by `rqt_graph`. It exposes the
graph state in multiple textual formats (DOT/JSON/adjacency) while simultaneously
hosting a lightweight web UI for interactive inspection—no Qt or extra Python
packages required.

## Package layout

| Path | Purpose |
| --- | --- |
| `ros2_graph/ros2_graph_node.py` | Main `rclpy` node that samples the ROS graph, prints it to stdout, and optionally serves it over HTTP. |
| `ros2_graph/web/server.py` | Pure-stdlib HTTP server plus the embedded HTML/JS canvas viewer served at `/`. |
| `resource/ros2_graph` & `setup.py` | Standard ament metadata and entry point (`ros2_graph = ros2_graph.ros2_graph_node:main`). |

## How it works

1. `GraphBuilder` (inside `ros2_graph_node.py`) calls
   `Node.get_topic_names_and_types()` along with publisher/subscription discovery
   APIs to construct an immutable `GraphSnapshot` containing nodes, topics, and QoS
   annotated edges.
2. Each timer tick (default: `update_interval = 2.0s`) the node fingerprints the
   snapshot; unchanged graphs are skipped.
3. When the graph changes:
   - The selected textual representation (`output_format = dot|json|adjacency`)
     is printed to stdout (so you can pipe it to Graphviz, parse JSON, etc.).
   - If the embedded HTTP server is enabled, the snapshot is serialized to JSON and
     published to `/graph`, and the browser UI (`/`) renders it on a canvas.

## Parameters

| Name | Default | Description |
| --- | --- | --- |
| `output_format` | `dot` | Format printed to stdout (`dot`, `json`, or `adjacency`). |
| `update_interval` | `2.0` | Seconds between graph refresh attempts (min 0.1). |
| `print_once` | `false` | When true, exit after emitting the first snapshot. |
| `web_enable` | `true` | Start the built-in HTTP server + UI. |
| `web_host` | `0.0.0.0` | Address for the HTTP server (use `127.0.0.1` to bind locally). |
| `web_port` | `8734` | Port for the HTTP server. |

## Usage

```bash
colcon build --packages-select ros2_graph
source install/setup.bash
ros2 run ros2_graph ros2_graph \
  --ros-args -p output_format:=json -p web_port:=9001
```

You'll see the chosen format printed to the terminal. Open
`http://<web_host>:<web_port>/` (default `http://localhost:8734/`) in a browser to
view the live canvas visualization. The `/graph` endpoint returns raw JSON for
external tooling, and `/healthz` exposes a simple readiness probe.

## Extending

- `GraphSnapshot` centralizes serialization; add new renderers there if you need
  alternative textual formats.
- The web UI is self-contained in `ros2_graph/web/server.py`. Replace the embedded
  HTML template with static files or add REST endpoints as needed.
- Additional graph data (e.g., services, actions, node metadata) can flow through
  the same snapshot+web mechanisms without touching the transport layers.
