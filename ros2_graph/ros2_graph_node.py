from __future__ import annotations

import hashlib
import json
from collections import defaultdict
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Set, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from .web import GraphWebServer


@dataclass(frozen=True)
class Edge:
    """Simple directed edge connecting ROS nodes and topics."""

    start: str
    end: str
    qos_label: str = ''


class GraphSnapshot:
    """Immutable snapshot of the ROS graph."""

    def __init__(
        self,
        nodes: Set[str],
        topics: Dict[str, Tuple[str, ...]],
        edges: Iterable[Edge],
    ) -> None:
        self.nodes = frozenset(nodes)
        self.topics = {name: tuple(types) for name, types in topics.items()}
        self.edges = tuple(sorted(set(edges), key=lambda e: (e.start, e.end, e.qos_label)))

    def fingerprint(self) -> str:
        """Stable fingerprint for change detection."""
        payload = {
            'nodes': sorted(self.nodes),
            'topics': {name: list(types) for name, types in sorted(self.topics.items())},
            'edges': [(e.start, e.end, e.qos_label) for e in self.edges],
        }
        serialized = json.dumps(payload, sort_keys=True, separators=(',', ':'))
        return hashlib.sha1(serialized.encode('utf-8')).hexdigest()

    def to_dict(self) -> Dict[str, object]:
        """Return a JSON-serializable dictionary."""
        return {
            'nodes': sorted(self.nodes),
            'topics': {name: list(types) for name, types in sorted(self.topics.items())},
            'edges': [
                {'start': e.start, 'end': e.end, 'qos': e.qos_label}
                for e in self.edges
            ],
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), indent=2, sort_keys=True)

    def to_dot(self) -> str:
        """Return GraphViz DOT source with additional layout hints."""
        node_publish_topics: Dict[str, Set[str]] = defaultdict(set)
        node_subscribe_topics: Dict[str, Set[str]] = defaultdict(set)
        topic_publishers: Dict[str, Set[str]] = defaultdict(set)
        topic_subscribers: Dict[str, Set[str]] = defaultdict(set)

        for edge in self.edges:
            if edge.start in self.nodes and edge.end in self.topics:
                node_publish_topics[edge.start].add(edge.end)
                topic_publishers[edge.end].add(edge.start)
            elif edge.start in self.topics and edge.end in self.nodes:
                node_subscribe_topics[edge.end].add(edge.start)
                topic_subscribers[edge.start].add(edge.end)

        source_nodes = sorted(
            node
            for node in self.nodes
            if node_publish_topics[node] and not node_subscribe_topics[node]
        )
        sink_nodes = sorted(
            node
            for node in self.nodes
            if node_subscribe_topics[node] and not node_publish_topics[node]
        )
        topic_names = sorted(self.topics.keys())

        lines: List[str] = [
            'digraph ros2_graph {',
            '  rankdir=LR;',
            '  graph [splines=ortho, overlap=false, ranksep=1.2, nodesep=0.8];',
            '  node [fontsize=14];',
            '  edge [fontsize=12];',
        ]

        for node in sorted(self.nodes):
            lines.append(f'  "{node}" [shape=ellipse];')

        for topic in topic_names:
            types = self.topics[topic]
            type_label = '\\n'.join(types)
            label = topic if not type_label else f'{topic}\\n{type_label}'
            safe_label = label.replace('"', '\\"')
            lines.append(f'  "{topic}" [shape=box,style=rounded,label="{safe_label}"];')

        if source_nodes:
            quoted = ' '.join(f'"{node}"' for node in source_nodes)
            lines.append(f'  {{ rank = min; {quoted}; }}')

        if sink_nodes:
            quoted = ' '.join(f'"{node}"' for node in sink_nodes)
            lines.append(f'  {{ rank = max; {quoted}; }}')

        if topic_names:
            quoted = ' '.join(f'"{topic}"' for topic in topic_names)
            lines.append(f'  {{ rank = same; {quoted}; }}')

        for topic in topic_names:
            publishers = sorted(topic_publishers.get(topic, []))
            subscribers = sorted(topic_subscribers.get(topic, []))
            for first, second in zip(publishers, publishers[1:]):
                lines.append(
                    f'  "{first}" -> "{second}" [style=invis, weight=1.5, constraint=true];'
                )
            for first, second in zip(subscribers, subscribers[1:]):
                lines.append(
                    f'  "{first}" -> "{second}" [style=invis, weight=1.5, constraint=true];'
                )
            if publishers and subscribers:
                lines.append(
                    f'  "{publishers[-1]}" -> "{subscribers[0]}" '
                    '[style=invis, weight=0.5, constraint=true];'
                )

        for edge in self.edges:
            attributes: List[str] = ['weight=2']
            if edge.qos_label:
                safe_qos = edge.qos_label.replace('"', '\\"')
                attributes.append(f'label="{safe_qos}"')
            attr_str = f" [{', '.join(attributes)}]" if attributes else ''
            lines.append(f'  "{edge.start}" -> "{edge.end}"{attr_str};')

        lines.append('}')
        return '\n'.join(lines)

    def to_adjacency(self) -> str:
        """Return a simple adjacency-list text format."""
        adjacency: Dict[str, List[Tuple[str, str]]] = {}
        for edge in self.edges:
            adjacency.setdefault(edge.start, []).append((edge.end, edge.qos_label))

        lines: List[str] = []
        for start in sorted(adjacency.keys()):
            targets = adjacency[start]
            formatted_targets = []
            for end, qos in sorted(targets, key=lambda t: t[0]):
                if qos:
                    formatted_targets.append(f'{end} [{qos}]')
                else:
                    formatted_targets.append(end)
            lines.append(f'{start} -> {", ".join(formatted_targets)}')

        if not lines:
            return '# graph is empty'
        return '\n'.join(lines)


class GraphBuilder:
    """Build graph snapshots using the node graph API."""

    def __init__(self, node: Node) -> None:
        self._node = node

    def build(self) -> GraphSnapshot:
        node_names: Set[str] = set()
        topics: Dict[str, Tuple[str, ...]] = {}
        edges: List[Edge] = []

        for topic_name, types in self._node.get_topic_names_and_types():
            topics[topic_name] = tuple(types)
            publisher_infos = self._node.get_publishers_info_by_topic(topic_name)
            subscription_infos = self._node.get_subscriptions_info_by_topic(topic_name)

            for info in publisher_infos:
                fq_name = _fully_qualified_node_name(info.node_namespace, info.node_name)
                node_names.add(fq_name)
                edges.append(Edge(fq_name, topic_name, _format_qos(info.qos_profile)))

            for info in subscription_infos:
                fq_name = _fully_qualified_node_name(info.node_namespace, info.node_name)
                node_names.add(fq_name)
                edges.append(Edge(topic_name, fq_name, _format_qos(info.qos_profile)))

        return GraphSnapshot(node_names, topics, edges)


def _fully_qualified_node_name(namespace: str, node_name: str) -> str:
    namespace = namespace or '/'
    if not namespace.startswith('/'):
        namespace = '/' + namespace
    namespace = namespace.rstrip('/')
    if not namespace:
        namespace = '/'
    if namespace == '/':
        return f'/{node_name}'.replace('//', '/')
    return f'{namespace}/{node_name}'.replace('//', '/')


def _format_qos(profile: QoSProfile | None) -> str:
    if profile is None:
        return ''

    parts: List[str] = []
    if profile.reliability is not None:
        parts.append(profile.reliability.name)
    if profile.durability is not None:
        parts.append(profile.durability.name)
    if profile.history is not None:
        parts.append(profile.history.name)
    if profile.depth not in (None, 0):
        parts.append(f'depth={profile.depth}')

    return '/'.join(parts)


class Ros2GraphNode(Node):
    """Minimal ROS node that prints graph snapshots to stdout."""

    def __init__(self) -> None:
        super().__init__('ros2_graph')
        self.declare_parameter('output_format', 'dot')
        self.declare_parameter('update_interval', 2.0)
        self.declare_parameter('print_once', False)
        self.declare_parameter('web_enable', True)
        self.declare_parameter('web_host', '0.0.0.0')
        self.declare_parameter('web_port', 8734)

        interval = max(float(self.get_parameter('update_interval').value), 0.1)
        self._output_format = str(self.get_parameter('output_format').value).lower()
        self._print_once = bool(self.get_parameter('print_once').value)
        self._web_server: Optional[GraphWebServer] = None
        if bool(self.get_parameter('web_enable').value):
            host = str(self.get_parameter('web_host').value or '0.0.0.0')
            port = int(self.get_parameter('web_port').value or 8734)
            try:
                self._web_server = GraphWebServer(host, port, self.get_logger())
                self._web_server.start()
            except OSError as exc:
                self.get_logger().error(f'Failed to start web server on {host}:{port} ({exc})')

        self._builder = GraphBuilder(self)
        self._last_fingerprint: str | None = None

        self._timer = self.create_timer(interval, self._update_graph)
        # emit immediately so users don't wait for the first interval
        self._update_graph()

    def _update_graph(self) -> None:
        snapshot = self._builder.build()
        fingerprint = snapshot.fingerprint()
        if fingerprint == self._last_fingerprint:
            return

        self._emit_snapshot(snapshot)
        self._publish_web(snapshot, fingerprint)
        self._last_fingerprint = fingerprint

        if self._print_once:
            self.get_logger().info('print_once=true, shutting down after first update')
            # allow logs to flush before shutting down
            rclpy.shutdown()

    def _emit_snapshot(self, snapshot: GraphSnapshot) -> None:
        formatter = {
            'dot': snapshot.to_dot,
            'json': snapshot.to_json,
            'adjacency': snapshot.to_adjacency,
        }.get(self._output_format, snapshot.to_dot)

        if formatter is snapshot.to_dot and self._output_format not in ('dot', 'json', 'adjacency'):
            self.get_logger().warning(
                f"Unknown output_format '{self._output_format}', falling back to DOT"
            )

        print(formatter(), flush=True)
        self.get_logger().info(
            f'graph updated ({len(snapshot.nodes)} nodes, '
            f'{len(snapshot.topics)} topics, {len(snapshot.edges)} edges)'
        )

    def _publish_web(self, snapshot: GraphSnapshot, fingerprint: str) -> None:
        if not self._web_server:
            return
        try:
            self._web_server.publish(snapshot, fingerprint)
        except Exception:  # pragma: no cover - defensive
            self.get_logger().exception('Failed to push graph update to web clients')

    def destroy_node(self) -> bool:
        if self._web_server:
            self._web_server.stop()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = Ros2GraphNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
