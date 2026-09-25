#!/usr/bin/env python3
"""Line-based direct Gazebo controller for the collision-test body."""

import time

from gz.msgs.twist_pb2 import Twist
from gz.transport import Node

TOPIC = "/model/collision_test_body/cmd_vel"
HELP = "Commands: forward [seconds], reverse [seconds], left [seconds], right [seconds], stop, quit"

def publish_for(publisher, linear, angular, seconds):
    msg = Twist()
    msg.linear.x = linear
    msg.angular.z = angular
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        publisher.publish(msg)
        time.sleep(0.1)
    publisher.publish(Twist())

def main():
    gz_node = Node()
    publisher = gz_node.advertise(TOPIC, Twist)
    if not publisher:
        raise RuntimeError(f"Could not advertise Gazebo topic: {TOPIC}")
    print(f"Publishing directly to {TOPIC}")
    print(HELP)
    while True:
        try:
            raw = input("> ").strip().lower().split()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not raw:
            continue
        command = raw[0]
        if command in {"quit", "q", "exit"}:
            break
        if command == "stop":
            publisher.publish(Twist())
            print("stopped")
            continue
        try:
            seconds = float(raw[1]) if len(raw) > 1 else 1.0
        except ValueError:
            print("Duration must be a number of seconds")
            continue
        seconds = max(0.1, min(seconds, 30.0))
        motions = {
            "forward": (0.8, 0.0),
            "f": (0.8, 0.0),
            "reverse": (-0.8, 0.0),
            "backward": (-0.8, 0.0),
            "b": (-0.8, 0.0),
            "left": (0.0, 1.0),
            "l": (0.0, 1.0),
            "right": (0.0, -1.0),
            "r": (0.0, -1.0),
        }
        if command not in motions:
            print(HELP)
            continue
        linear, angular = motions[command]
        print(f"{command} for {seconds:.1f} seconds")
        publish_for(publisher, linear, angular, seconds)
    publisher.publish(Twist())
    print("controller stopped")

if __name__ == "__main__":
    main()
