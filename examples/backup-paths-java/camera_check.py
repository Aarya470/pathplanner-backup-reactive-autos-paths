"""Check the detection camera numbers with a tape measure. Same math as the robot code.

Uses src/main/deploy/detection-camera.json. The lens numbers (fx, fy, cx, cy) come from the
camera's calibration in PhotonVision: open the PhotonVision page, Cameras tab, and read them for
the resolution the detection pipeline uses. Pass them with --lens, or put fx, fy, cx, cy in the
json file.

1. Work out the camera tilt (pitchDownDegrees) from one measurement:
     put a game piece on the floor straight out from the camera (behind the robot for a camera on
     the back), measure the distance in meters from the robot center to where the piece touches
     the floor, and read the bottom-center pixel of its
     box in PhotonVision. Then run:
       python camera_check.py --solve-pitch <distance_m> <pixel_y> --lens fx fy cx cy
     Put the printed pitchDownDegrees in detection-camera.json.

2. Check a floor position:
       python camera_check.py <pixel_x> <pixel_y> --lens fx fy cx cy
     It prints how far forward and left of the robot center the floor point is (negative forward
     means behind the robot). Compare it with
     a tape measure at a few spots (near, far, left, right).

Add --config <path> to use a json file somewhere else.
"""
import argparse
import json
import math
import sys
from pathlib import Path


def load_config(path_arg):
    if path_arg:
        path = Path(path_arg)
    else:
        here = Path(__file__).resolve().parent
        candidates = [
            here.parent / "src" / "main" / "deploy" / "detection-camera.json",
            here / "detection-camera.json",
            Path.cwd() / "src" / "main" / "deploy" / "detection-camera.json",
        ]
        path = next((p for p in candidates if p.exists()), None)
        if path is None:
            sys.exit("Can't find detection-camera.json. Pass it with --config <path>.")
    return json.loads(path.read_text()), path


def length(cfg, name):
    """A length given as name + "Inches" or name + "Meters", in meters."""
    if name + "Inches" in cfg:
        return cfg[name + "Inches"] * 0.0254
    return cfg[name + "Meters"]


def lens_numbers(cfg, lens):
    if lens:
        return lens
    if all(k in cfg for k in ("fx", "fy", "cx", "cy")):
        return [cfg["fx"], cfg["fy"], cfg["cx"], cfg["cy"]]
    sys.exit("Need the lens numbers. Add --lens fx fy cx cy (from PhotonVision's calibration).")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("pixel", nargs="*", type=float, help="pixel_x pixel_y of the box's bottom center")
    ap.add_argument("--solve-pitch", nargs=2, type=float, metavar=("DISTANCE_M", "PIXEL_Y"))
    ap.add_argument("--lens", nargs=4, type=float, metavar=("FX", "FY", "CX", "CY"))
    ap.add_argument("--config")
    args = ap.parse_args()

    cfg, path = load_config(args.config)
    fx, fy, cx, cy = lens_numbers(cfg, args.lens)
    height = length(cfg, "height")

    if args.solve_pitch:
        distance, v = args.solve_pitch
        # Straight out from the lens along the way it points (yaw 180 = straight behind the robot)
        forward = length(cfg, "forward")
        lens_distance = distance - abs(forward) if abs(cfg["yawDegrees"]) > 90 else distance - forward
        if lens_distance <= 0:
            sys.exit("The piece must be in front of the camera lens.")
        depression = math.atan2(height, lens_distance)
        pitch = depression - math.atan2(v - cy, fy)
        print(f"Using {path}")
        print(f"pitchDownDegrees: {math.degrees(pitch):.1f}")
        return

    if len(args.pixel) != 2:
        ap.error("give pixel_x pixel_y, or use --solve-pitch")
    u, v = args.pixel

    pitch = math.radians(cfg["pitchDownDegrees"])
    yaw = math.radians(cfg["yawDegrees"])
    depression = pitch + math.atan2(v - cy, fy)
    if depression <= 1e-6:
        sys.exit("This pixel points at or above the horizon, so it can't be on the floor.")

    ground = height / math.tan(depression)
    ray = yaw + math.atan2(cx - u, fx)
    forward = length(cfg, "forward") + ground * math.cos(ray)
    left = length(cfg, "left") + ground * math.sin(ray)
    print(f"Using {path}")
    print(f"Floor point: {forward:.2f} m forward, {left:.2f} m left of robot center")


if __name__ == "__main__":
    main()
