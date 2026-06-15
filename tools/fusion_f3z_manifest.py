#!/usr/bin/env python3
"""Extract a readable file map from a Fusion .f3z DesignDescription.json."""

import csv
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DESCRIPTION = ROOT / "cad" / "fusion" / "extracted" / "DesignDescription.json"
OUTPUT = ROOT / "cad" / "fusion" / "extracted" / "fusion_file_map.csv"


def main() -> None:
    data = json.loads(DESCRIPTION.read_text())
    graphs = data["designDescription"]["designGraphs"]
    rows = []
    for graph in graphs:
        root_ids = set(graph.get("rootIds", []))
        for obj in graph.get("designObjects", []):
            rows.append(
                {
                    "id": obj.get("id", ""),
                    "is_root": "yes" if obj.get("id") in root_ids else "",
                    "friendly_name": obj.get("friendlyName", ""),
                    "display_name": obj.get("displayName", ""),
                    "relative_path": obj.get("relativePath", ""),
                    "content_type": obj.get("contentType", ""),
                    "root_file_name": obj.get("metadata", {}).get("rootFileName", ""),
                    "description": obj.get("metadata", {}).get("description", ""),
                    "created_at": obj.get("createdAt", ""),
                }
            )

    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    with OUTPUT.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    print(OUTPUT)


if __name__ == "__main__":
    main()
