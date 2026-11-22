"""Check a camera's mountedAt numbers with a tape measure. Same math as the robot code.

--mount takes the same 5 numbers as mountedAt(...) in RobotContainer.java:
  forward_m left_m height_m tilt_down_deg facing_deg

Limelight (or any camera that gives tx/ty angles):
  1. Find the tilt: put a game piece on the floor straight out from the camera, measure from the
     robot's center to where it touches the floor, and read its ty. Then:
       python camera_check.py --mount -0.35 0 0.45 0 180 --solve-tilt 1.50 --ty -12.3
  2. Check where something is:
       python camera_check.py --mount -0.35 0 0.45 25 180 --tx 4.0 --ty -10.0
     Add --target-height 0.4 for a robot (default 0.08, a game piece).

PhotonVision with calibration (uses the bottom middle pixel of the box, on the floor):
  Read fx fy cx cy from the camera's calibration (PhotonVision page, Cameras tab), then:
       python camera_check.py --mount -0.35 0 0.45 0 180 --lens 700 700 640 360 --solve-tilt 1.50 --pixel-y 500
       python camera_check.py --mount -0.35 0 0.45 25 180 --lens 700 700 640 360 --pixel 610 520

It prints how far forward and left of the robot's center the point is (negative forward means
behind the robot). Compare with a tape measure at a few spots: near, far, left and right.
"""
import argparse
import math
import sys


def to_robot(mount, tx_deg, ty_deg, target_height):
    """Follow the ray from the lens until it reaches target_height. Matches Camera.toRobotRelative."""
    fwd, left, height, tilt_deg, facing_deg = mount
    x, y, z = 1.0, -math.tan(math.radians(tx_deg)), math.tan(math.radians(ty_deg))
    p = math.radians(tilt_deg)
    xt = x * math.cos(p) + z * math.sin(p)
    zt = -x * math.sin(p) + z * math.cos(p)
    if abs(zt) < 1e-9:
        return None
    t = (target_height - height) / zt
    if t <= 0 or abs(zt) / math.hypot(xt, y) < math.tan(math.radians(0.5)):
        return None
    f = math.radians(facing_deg)
    rx, ry = t * xt, t * y
    return fwd + rx * math.cos(f) - ry * math.sin(f), left + rx * math.sin(f) + ry * math.cos(f)


def pixel_to_angles(u, v, lens):
    fx, fy, cx, cy = lens
    return math.degrees(math.atan((u - cx) / fx)), math.degrees(math.atan((cy - v) / fy))


def solve_tilt(mount, distance, ty_deg, target_height):
    """Tilt that puts a target straight out from the camera at this distance from the robot center."""
    fwd, left, height, _, facing_deg = mount
    # The target is straight out from the lens along the way the camera faces, and `distance` from
    # the robot's center: solve |lens + d * facing| = distance for d
    f = math.radians(facing_deg)
    b = fwd * math.cos(f) + left * math.sin(f)
    disc = b * b - (fwd * fwd + left * left - distance * distance)
    lens_to_target = -b + math.sqrt(disc) if disc >= 0 else -1
    if lens_to_target <= 0:
        sys.exit("The distance must be measured from the robot's center and be past the camera.")
    below = math.degrees(math.atan2(height - target_height, lens_to_target))
    return below + ty_deg


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mount", nargs=5, type=float, required=True,
                    metavar=("FORWARD_M", "LEFT_M", "HEIGHT_M", "TILT_DOWN_DEG", "FACING_DEG"))
    ap.add_argument("--tx", type=float, help="degrees right of center (Limelight tx)")
    ap.add_argument("--ty", type=float, help="degrees above center (Limelight ty)")
    ap.add_argument("--lens", nargs=4, type=float, metavar=("FX", "FY", "CX", "CY"))
    ap.add_argument("--pixel", nargs=2, type=float, metavar=("PIXEL_X", "PIXEL_Y"),
                    help="bottom middle of the box, with --lens")
    ap.add_argument("--pixel-y", type=float, help="bottom of the box, for --solve-tilt with --lens")
    ap.add_argument("--solve-tilt", type=float, metavar="DISTANCE_M",
                    help="distance from the robot center to a target straight out from the camera")
    ap.add_argument("--target-height", type=float,
                    help="height of what the angles point at: default 0.08 (game piece middle), or 0 with --lens")
    args = ap.parse_args()

    use_lens = args.lens is not None
    target_height = args.target_height if args.target_height is not None else (0.0 if use_lens else 0.08)

    if args.solve_tilt is not None:
        if use_lens:
            if args.pixel_y is None:
                sys.exit("With --lens, give --pixel-y of the bottom of the box.")
            ty = pixel_to_angles(args.lens[2], args.pixel_y, args.lens)[1]
        elif args.ty is not None:
            ty = args.ty
        else:
            sys.exit("Give --ty (or --lens and --pixel-y).")
        tilt = solve_tilt(args.mount, args.solve_tilt, ty, target_height)
        print(f"tilt down = {tilt:.1f} degrees. Put it in mountedAt(...) as the 4th number.")
        return

    if use_lens:
        if args.pixel is None:
            sys.exit("With --lens, give --pixel X Y.")
        tx, ty = pixel_to_angles(args.pixel[0], args.pixel[1], args.lens)
    else:
        if args.tx is None or args.ty is None:
            sys.exit("Give --tx and --ty (or --lens and --pixel).")
        tx, ty = args.tx, args.ty

    point = to_robot(args.mount, tx, ty, target_height)
    if point is None:
        sys.exit("That ray never reaches the target height: check the height and tilt.")
    print(f"forward {point[0]:.2f} m, left {point[1]:.2f} m, "
          f"{math.hypot(point[0], point[1]):.2f} m from the robot's center")


if __name__ == "__main__":
    main()
