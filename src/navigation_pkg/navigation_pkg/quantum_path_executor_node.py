#!/usr/bin/env python3
"""
Backward-compatible wrapper for the Phase-9 Path Executor Node.
Defaults to listening on /path/quantum.
"""

from navigation_pkg.path_executor_node import PathExecutorNode, main


class QuantumPathExecutor(PathExecutorNode):
    """Alias for PathExecutorNode targeting /path/quantum by default."""
    pass


if __name__ == "__main__":
    main()