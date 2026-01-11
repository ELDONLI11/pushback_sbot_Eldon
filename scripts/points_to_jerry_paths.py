#!/usr/bin/env python3
"""Convert JerryIO point-list logs into a JerryIO file.

Input can be a raw PROS log that contains:
- '#PATH-POINTS-START Path'
- lines like 'x,y,120' (optionally 'x,y,120,heading')
- optionally '#PATH.JERRYIO-DATA {...}'

Output is a text file suitable for pasting into jerryio.com:
- '#PATH-POINTS-START Path'
- the same point lines (normalized)
- '#PATH.JERRYIO-DATA {...}' with either:
    - points-only mode (default): 'paths': []
    - with-paths mode: a generated 'paths' array.

NOTE:
JerryIO validation is picky about the exact 'paths' schema. If you see validation errors,
use the default points-only mode (no segments/controls).

When using --with-paths, this script uses a minimal polyline representation:
each segment contains only 2 end-point controls (start/end). This matches the approach
in reversejerry.py and tends to validate more reliably than Bezier control points.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional


POINT_RE = re.compile(
    r"(?P<x>-?\d+(?:\.\d+)?)\s*,\s*(?P<y>-?\d+(?:\.\d+)?)\s*,\s*(?P<speed>-?\d+(?:\.\d+)?)(?:\s*,\s*(?P<h>-?\d+(?:\.\d+)?))?"
)


@dataclass(frozen=True)
class Pt:
    x: float
    y: float
    heading: Optional[float] = None


DEFAULT_GC: dict[str, Any] = {
    "robotWidth": 30,
    "robotHeight": 30,
    "robotIsHolonomic": False,
    "showRobot": False,
    # Your coordinate dumps are in inches; JerryIO uses 'uol' as a scale-to-cm factor.
    # Existing working files in this repo use uol=2.54 for inch-based coordinates.
    "uol": 2.54,
    "pointDensity": 2,
    "controlMagnetDistance": 5,
    "fieldImage": {
        "displayName": "V5RC 2026 - Push Back",
        "signature": "V5RC-PushBack-H2H-TopDownHighlighted-TileColor66_71",
        "origin": {"__type": "built-in"},
    },
    "coordinateSystem": "VEX Gaming Positioning System",
}

DEFAULT_PC: dict[str, Any] = {
    "speedLimit": {
        "minLimit": {"value": 0, "label": "0"},
        "maxLimit": {"value": 600, "label": "600"},
        "step": 1,
        "from": 40,
        "to": 120,
    },
    "bentRateApplicableRange": {
        "minLimit": {"value": 0, "label": "0"},
        "maxLimit": {"value": 1, "label": "1"},
        "step": 0.001,
        "from": 0,
        "to": 0.1,
    },
}


def _heading_from_delta(p0: Pt, p1: Pt) -> float:
    dx = p1.x - p0.x
    dy = p1.y - p0.y
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return 0.0
    # Best-effort: degrees of motion direction.
    return math.degrees(math.atan2(dy, dx))


def _pt_heading_for_legacy(pts: list[Pt], i: int) -> float:
    """Best-effort per-point heading for legacy point dumps (x,y,heading).

    Legacy files in pushback/static/ store heading (deg) as the 3rd column.
    We derive it from motion direction to the next point; for the last point
    we fall back to the previous segment.
    """
    if not pts:
        return 0.0
    if pts[i].heading is not None:
        return _norm_heading_deg(pts[i].heading)
    if i + 1 < len(pts):
        return _norm_heading_deg(_heading_from_delta(pts[i], pts[i + 1]))
    if i - 1 >= 0:
        return _norm_heading_deg(_heading_from_delta(pts[i - 1], pts[i]))
    return 0.0


def _norm_heading_deg(deg: float) -> float:
    deg = deg % 360.0
    if deg < 0:
        deg += 360.0
    return deg


def parse_footer_json(lines: list[str]) -> tuple[Optional[dict[str, Any]], int]:
    """Return (parsed_json, line_index) for the first '#PATH.JERRYIO-DATA ' line."""
    for idx, line in enumerate(lines):
        if line.startswith("#PATH.JERRYIO-DATA "):
            raw = line[len("#PATH.JERRYIO-DATA ") :].strip()
            try:
                return json.loads(raw), idx
            except json.JSONDecodeError:
                return None, idx
    return None, -1


def extract_points(lines: list[str], *, allow_anywhere: bool) -> list[Pt]:
    pts: list[Pt] = []
    in_points = False
    for line in lines:
        if line.startswith("#PATH-POINTS-START"):
            in_points = True
            continue
        # Footer marker is '#PATH.JERRYIO-DATA'. Stop scanning points at the footer.
        if line.startswith("#PATH.JERRYIO-DATA"):
            break
        if not in_points and not allow_anywhere:
            continue
        # Be tolerant of extra log text on the same line. We still only accept the first
        # x,y,speed[,heading] numeric tuple found.
        m = POINT_RE.search(line)
        if not m:
            continue
        x = float(m.group("x"))
        y = float(m.group("y"))
        h = m.group("h")
        pt = Pt(x=x, y=y, heading=float(h) if h is not None else None)
        # Drop consecutive duplicates (common in logs).
        if pts and abs(pts[-1].x - pt.x) < 1e-6 and abs(pts[-1].y - pt.y) < 1e-6:
            continue
        pts.append(pt)
    return pts


def build_paths(pts: list[Pt], stride: int, name: str) -> list[dict[str, Any]]:
    if len(pts) < 2:
        return []

    stride = max(1, stride)

    segments: list[dict[str, Any]] = []
    seg_idx = 0

    i0 = 0
    while i0 + stride < len(pts):
        i1 = min(i0 + stride, len(pts) - 1)
        p0 = pts[i0]
        p1 = pts[i1]

        # Skip zero-length segments.
        if abs(p1.x - p0.x) < 1e-9 and abs(p1.y - p0.y) < 1e-9:
            i0 = i1
            continue

        h0_raw = p0.heading if p0.heading is not None else _heading_from_delta(p0, p1)
        h1_raw = p1.heading if p1.heading is not None else _heading_from_delta(p0, p1)
        h0 = _norm_heading_deg(h0_raw)
        h1 = _norm_heading_deg(h1_raw)

        # JerryIO segments are typically cubic Beziers with 4 controls:
        # end-point, control, control, end-point.
        # For a simple polyline-like path, we place control points on the straight
        # line between endpoints (at 1/3 and 2/3). This matches the schema used by
        # known-good files in this repo and renders on jerryio.com.
        c1x = p0.x + (p1.x - p0.x) / 3.0
        c1y = p0.y + (p1.y - p0.y) / 3.0
        c2x = p0.x + 2.0 * (p1.x - p0.x) / 3.0
        c2y = p0.y + 2.0 * (p1.y - p0.y) / 3.0

        segments.append(
            {
                "controls": [
                    {
                        "uid": f"SBOT_EP_{seg_idx}_A",
                        "x": p0.x,
                        "y": p0.y,
                        "lock": False,
                        "visible": True,
                        "heading": round(h0, 2),
                        "__type": "end-point",
                    },
                    {
                        "uid": f"SBOT_C_{seg_idx}_A",
                        "x": c1x,
                        "y": c1y,
                        "lock": False,
                        "visible": True,
                        "__type": "control",
                    },
                    {
                        "uid": f"SBOT_C_{seg_idx}_B",
                        "x": c2x,
                        "y": c2y,
                        "lock": False,
                        "visible": True,
                        "__type": "control",
                    },
                    {
                        "uid": f"SBOT_EP_{seg_idx}_B",
                        "x": p1.x,
                        "y": p1.y,
                        "lock": False,
                        "visible": True,
                        "heading": round(h1, 2),
                        "__type": "end-point",
                    },
                ],
                "speedProfiles": [],
                "lookaheadKeyframes": [],
                "uid": f"SBOT_SEG_{seg_idx}",
            }
        )

        seg_idx += 1
        i0 = i1

    return [
        {
            "segments": segments,
            "pc": DEFAULT_PC,
            "name": name,
            "uid": "SBOT_PATH",
            "lock": False,
            "visible": True,
        }
    ]


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("input", type=Path)
    ap.add_argument("output", type=Path)
    ap.add_argument(
        "--anywhere",
        action="store_true",
        help="Extract point lines from anywhere in the file (no need for #PATH-POINTS-START).",
    )
    ap.add_argument(
        "--with-paths",
        action="store_true",
        help="Generate a 'paths' array as a polyline (segments with 2 end-point controls).",
    )
    ap.add_argument(
        "--just-points",
        action="store_true",
        help="Write only normalized point lines (x,y,120), one per line (no header/footer).",
    )
    ap.add_argument(
        "--legacy",
        action="store_true",
        help=(
            "Write legacy file format that matches pushback/static/*.txt (points as x,y,heading + endData block + footer). "
            "This is the most compatible with jerryio.com 'Open file'."
        ),
    )
    ap.add_argument("--stride", type=int, default=5, help="points per segment when using --with-paths")
    ap.add_argument("--name", type=str, default="Path")
    ap.add_argument("--max-decel", type=float, default=50.0, help="Legacy: maxDecelerationRate")
    ap.add_argument("--speed-from", type=float, default=0.0, help="Legacy: speedLimit.from")
    ap.add_argument("--speed-to", type=float, default=127.0, help="Legacy: speedLimit.to")
    ap.add_argument("--legacy-k", type=float, default=200.0, help="Legacy: third numeric line after endData (kept for compatibility)")
    args = ap.parse_args(argv)

    raw = args.input.read_text(encoding="utf-8", errors="replace").splitlines()

    footer_json, _footer_idx = parse_footer_json(raw)
    gc = DEFAULT_GC
    if isinstance(footer_json, dict) and isinstance(footer_json.get("gc"), dict):
        gc = footer_json["gc"]

    pts = extract_points(raw, allow_anywhere=args.anywhere)
    if len(pts) < 2:
        if args.anywhere:
            print("No point lines found. Expected lines like: x,y,120", file=sys.stderr)
        else:
            print("No points found between '#PATH-POINTS-START' and footer.", file=sys.stderr)
        return 2

    # In legacy mode we always emit a single cubic segment path.
    paths = build_paths(pts, stride=args.stride, name=args.name) if (args.with_paths or args.legacy) else []

    # Emit output.
    out_lines: list[str] = []
    if args.just_points:
        for p in pts:
            out_lines.append(f"{p.x:.3f},{p.y:.3f},120")
    elif args.legacy:
        # Match the pre-footer layout seen in pushback/static/*.txt.
        # 1) points as: x, y, heading
        for i, p in enumerate(pts):
            h = _pt_heading_for_legacy(pts, i)
            out_lines.append(f"{p.x:.3f}, {p.y:.3f}, {h:.3f}")
        # 2) endData + 3 numeric lines
        out_lines.append("endData")
        out_lines.append(f"{args.max_decel:g}")
        out_lines.append(f"{args.speed_from:g}")
        out_lines.append(f"{args.legacy_k:g}")

        # 3) control line: start, c1, c2, end (8 numbers)
        p0 = pts[0]
        p1 = pts[-1]
        c1x = p0.x + (p1.x - p0.x) / 3.0
        c1y = p0.y + (p1.y - p0.y) / 3.0
        c2x = p0.x + 2.0 * (p1.x - p0.x) / 3.0
        c2y = p0.y + 2.0 * (p1.y - p0.y) / 3.0
        out_lines.append(
            f"{p0.x:.3f}, {p0.y:.3f}, {c1x:.3f}, {c1y:.3f}, {c2x:.3f}, {c2y:.3f}, {p1.x:.3f}, {p1.y:.3f}"
        )

        # 4) Footer JSON. Also align pc to legacy numeric lines.
        legacy_pc = {
            **DEFAULT_PC,
            "speedLimit": {
                **DEFAULT_PC["speedLimit"],
                "from": args.speed_from,
                "to": args.speed_to,
                "maxLimit": {"value": 127, "label": "127"},
            },
            "maxDecelerationRate": args.max_decel,
            "bentRateApplicableRange": {
                **DEFAULT_PC["bentRateApplicableRange"],
                "from": 0,
                "to": 0.158,
            },
        }

        # Ensure the exported path uses the same pc structure.
        if paths:
            paths[0]["pc"] = legacy_pc

        out_footer = {"appVersion": "0.10.0", "format": "LemLib v0.5", "gc": {**gc, "showRobot": True}, "paths": paths}
        out_lines.append("#PATH.JERRYIO-DATA " + json.dumps(out_footer, separators=(",", ":")))
    else:
        # Use the same 'format' label as known-good exports in this repo.
        out_footer = {"appVersion": "0.10.0", "format": "LemLib v0.5", "gc": gc, "paths": paths}
        out_lines.append("#PATH-POINTS-START Path")
        for p in pts:
            out_lines.append(f"{p.x:.3f},{p.y:.3f},120")
        out_lines.append("#PATH.JERRYIO-DATA " + json.dumps(out_footer, separators=(",", ":")))

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("\n".join(out_lines) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
