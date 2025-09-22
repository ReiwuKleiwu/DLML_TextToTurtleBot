"""Application entrypoint wiring the ROS node and orchestration."""
from __future__ import annotations

import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parent
REPO_ROOT = PACKAGE_ROOT.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import rclpy

from code.application.runtime.control_loop import ControlLoop
from code.infrastructure.event.threaded_event_bus import ThreadedEventBus
from code.infrastructure.ros.node import TurtleBotNode


def main(args=None) -> None:
    rclpy.init(args=args)

    event_bus = ThreadedEventBus()
    event_bus.start()

    node = TurtleBotNode(event_bus=event_bus, namespace="", use_turtlebot_sim=True)

    control_loop = ControlLoop(rate_hz=2.0, tick_callback=node.orchestrator.perform_control_step)
    control_loop.start()

    try:
        rclpy.spin(node)
    finally:
        control_loop.stop()
        control_loop.join()
        node.destroy_node()
        event_bus.stop()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
