#!/usr/bin/env python3
import argparse
import json
import sys
import time
from pathlib import Path

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Publish POIs to /mapping/cmd_pois")
    parser.add_argument("--tinynav_map_path", required=True)
    parser.add_argument("--pois", default=None, help="Comma-separated POI ids, for example 2,1,0")
    return parser.parse_args()


def parse_pois_arg(pois: str) -> list[str]:
    values = [value.strip() for value in pois.split(",") if value.strip()]
    if not values:
        raise ValueError("--pois must be a comma-separated list like 2,1,0")
    return values


def load_selected_pois(tinynav_map_path: Path, pois: str | None) -> dict[str, object]:
    pois_path = tinynav_map_path / "pois.json"
    if not pois_path.exists():
        raise FileNotFoundError(f"POI file not found: {pois_path}")
    data = json.loads(pois_path.read_text())
    if not isinstance(data, dict):
        raise ValueError("pois.json must be a JSON object")
    if pois is None:
        return data
    selected = {}
    for index, poi_key in enumerate(parse_pois_arg(pois)):
        if poi_key not in data:
            raise KeyError(f"POI {poi_key} not found in {pois_path}")
        selected[str(index)] = data[poi_key]
    return selected


def publish(payload: dict[str, object]) -> None:
    rclpy.init()
    node = rclpy.create_node("pub_pois")
    # Must match map_node's TRANSIENT_LOCAL subscription. DDS requires the writer's
    # durability to be at least the reader's, so a plain volatile publisher here never
    # matches at all -- and the failure surfaces as the timeout below, which reads like
    # map_node being slow to start rather than the two ends being incompatible.
    # Latching is also what this topic needs on its own merits: it carries the current
    # nav target, published once, to a subscriber that may not exist yet.
    publisher = node.create_publisher(
        String, "/mapping/cmd_pois",
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    msg = String()
    msg.data = json.dumps(payload, separators=(",", ":"))
    # monotonic, not wall clock: the board has no RTC, so time.time() jumps when the
    # clock is first set and a forward jump fires this timeout instantly.
    deadline = time.monotonic() + 5.0
    while publisher.get_subscription_count() == 0:
        if time.monotonic() >= deadline:
            node.destroy_node()
            rclpy.shutdown()
            raise RuntimeError("Timed out waiting for /mapping/cmd_pois subscribers")
        rclpy.spin_once(node, timeout_sec=0.1)
    publisher.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


def main() -> int:
    args = parse_args()
    try:
        tinynav_map_path = Path(args.tinynav_map_path)
        payload = load_selected_pois(tinynav_map_path, args.pois)
        publish(payload)
    except (FileNotFoundError, ValueError, KeyError, RuntimeError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    print(f"Published POIs for map: {tinynav_map_path}")
    print(f"POIs: {args.pois or 'all'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
