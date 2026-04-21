#!/usr/bin/env python3
import math
import argparse
from textwrap import dedent

def main():
    ap = argparse.ArgumentParser(
        description="Calcula fx, fy, cx, cy a partir del FOV y la resolución."
    )
    ap.add_argument("--width",  "-W", type=int, required=True, help="Ancho de imagen en píxeles.")
    ap.add_argument("--height", "-H", type=int, required=True, help="Alto de imagen en píxeles.")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--hfov", type=float, help="FOV horizontal (por defecto en radianes).")
    g.add_argument("--vfov", type=float, help="FOV vertical (por defecto en radianes).")
    ap.add_argument("--deg", action="store_true", help="Indica que el FOV se pasa en grados (no radianes).")
    ap.add_argument("--frame-id", default="camera_gnd_link", help="frame_id para CameraInfo (opcional).")
    ap.add_argument("--print-camera-info", action="store_true",
                    help="Imprime además un bloque sensor_msgs/CameraInfo coherente (modelo pinhole sin distorsión).")
    args = ap.parse_args()

    W = float(args.width)
    H = float(args.height)

    if args.hfov is not None:
        hfov = math.radians(args.hfov) if args.deg else float(args.hfov)
        if not (0.0 < hfov < math.pi):
            raise ValueError("hfov debe estar en (0, π) rad.")
        # vFOV derivado por relación de aspecto (píxeles cuadrados)
        vfov = 2.0 * math.atan(math.tan(hfov/2.0) * (H / W))
    else:
        vfov = math.radians(args.vfov) if args.deg else float(args.vfov)
        if not (0.0 < vfov < math.pi):
            raise ValueError("vfov debe estar en (0, π) rad.")
        # hFOV derivado
        hfov = 2.0 * math.atan(math.tan(vfov/2.0) * (W / H))

    # Focales en píxeles (modelo pinhole)
    fx = W / (2.0 * math.tan(hfov / 2.0))
    fy = H / (2.0 * math.tan(vfov / 2.0))

    # Centro óptico (asumiendo centro de imagen)
    cx = W / 2.0
    cy = H / 2.0

    def fmt(x):  # imprime con 6 decimales
        return f"{x:.6f}"

    print("== Intrínsecos ==")
    print(f"width:  {int(W)} px, height: {int(H)} px")
    print(f"hfov:   {hfov:.6f} rad ({math.degrees(hfov):.3f} deg)")
    print(f"vfov:   {vfov:.6f} rad ({math.degrees(vfov):.3f} deg)")
    print(f"fx:     {fmt(fx)}")
    print(f"fy:     {fmt(fy)}")
    print(f"cx:     {fmt(cx)}")
    print(f"cy:     {fmt(cy)}")

    if args.print_camera_info:
        K = [fx, 0.0, cx,
             0.0, fy, cy,
             0.0, 0.0, 1.0]
        P = [fx, 0.0, cx, 0.0,
             0.0, fy, cy, 0.0,
             0.0, 0.0, 1.0, 0.0]
        R = [1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0]
        D = [0.0, 0.0, 0.0, 0.0, 0.0]  # sin distorsión (pinhole ideal)

        yaml = f"""
        header:
          frame_id: {args.frame_id}
        height: {int(H)}
        width: {int(W)}
        distortion_model: plumb_bob
        d: [{", ".join(fmt(x) for x in D)}]
        k: [{", ".join(fmt(x) for x in K)}]
        r: [{", ".join(fmt(x) for x in R)}]
        p: [{", ".join(fmt(x) for x in P)}]
        binning_x: 0
        binning_y: 0
        roi:
          x_offset: 0
          y_offset: 0
          height: 0
          width: 0
          do_rectify: false
        """
        print("\n== CameraInfo (pinhole, sin distorsión) ==")
        print(dedent(yaml).strip())

if __name__ == "__main__":
    main()
