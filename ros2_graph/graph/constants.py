INTERNAL_NODE_NAMES = {
    "/ros2_graph_metrics_probe",
    "ros2_graph_metrics_probe",
    "/ros2_graph",
    "ros2_graph",
    "/ros2cli_daemon",
    "ros2cli_daemon",
}

INTERNAL_NODE_PREFIXES = (
    "/tf_listener",
    "/tf2_buffer",
    "/tf_static_listener",
    "/transform_listener",
    "/_ros2cli_daemon",
    "_ros2cli_daemon",
    "ros2cli_daemon_",
)

__all__ = ["INTERNAL_NODE_NAMES", "INTERNAL_NODE_PREFIXES"]
