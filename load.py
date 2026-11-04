"""Interactive Habitat-Sim data collector (load.py).

    Full module guide migrated to docs/load.md. Thin driver over
    packages/simulator; see that file for run modes, keybindings,
    outputs, and GL ordering.
"""

import argparse
import json
import os
import time

import numpy as np

DEFAULT_CONFIG = os.path.join(os.path.dirname(__file__), "configs", "second_floor.yaml")


def parse_args():
    parser = argparse.ArgumentParser(
        description="hw1 data collector — thin driver over packages/simulator")
    parser.add_argument("--config", default=DEFAULT_CONFIG, help="Path to YAML config")
    parser.add_argument("--trajectory", default=None,
                        help="optional .npy (N,7) pose trajectory to replay-preview "
                             "instead of interactive keyboard collection")
    parser.add_argument("--fps", type=float, default=30.0,
                        help="preview frame rate (0 = as fast as possible)")
    parser.add_argument("--output-root", default=None,
                        help="override output.root from the config")
    parser.add_argument("--clean", action="store_true",
                        help="force uncertainties.enabled=false — no zone fires "
                             "anywhere (uncorrupted collection)")
    parser.add_argument("--auto-capture", action="store_true",
                        help="save a frame after each movement/turn that changes the camera pose")
    return parser.parse_args()


def run_replay(engine, viewer, preview, traj_path, fps, data_root, out_cfg):
    """Replay a .npy pose trajectory, previewing AND saving every frame.

    Deterministic time base (t = i / engine.fps_nominal) lives inside
    replay_poses; this callback only does output I/O, preview, abort, pacing."""
    from simulator import load_trajectory, replay_poses, save_frame

    pygame = viewer.pygame
    poses = load_trajectory(traj_path)
    delay_ms = int(1000.0 / fps) if fps and fps > 0 else 0
    print(f"replaying {len(poses)} poses — q / ESC to abort")

    def on_frame(frame, sensor_state, idx):
        save_frame(frame, sensor_state, data_root, out_cfg, idx)
        preview.draw(frame, idx)
        for event in pygame.event.get():          # let the user abort mid-replay
            if event.type == pygame.QUIT or (
                    event.type == pygame.KEYDOWN
                    and event.key in (pygame.K_q, pygame.K_ESCAPE)):
                return False
            if event.type in viewer.EXPOSE_EVENTS:
                preview.invalidate()
        if delay_ms:
            pygame.time.wait(delay_ms)
        return True

    captured = replay_poses(engine, poses, on_frame)   # caller saves GT_pose.npy
    np.save(os.path.join(data_root, "GT_pose.npy"), captured)
    print(f"replay: saved {len(captured)} poses to "
          f"{os.path.join(data_root, 'GT_pose.npy')}")


def run_interactive(engine, viewer, preview, fps, data_root, out_cfg, auto_capture=False):
    """Keyboard-driven collection; zones fire wherever the agent walks."""
    from simulator import save_frame

    pygame = viewer.pygame
    KEY_ACTION = {
        pygame.K_w: "move_forward",
        pygame.K_s: "move_backward",
        pygame.K_a: "turn_left",
        pygame.K_d: "turn_right",
    }
    CAPTURE_KEYS = (pygame.K_c, pygame.K_SPACE)
    QUIT_KEYS = (pygame.K_q, pygame.K_ESCAPE)

    print("#############################")
    if auto_capture:
        print("AUTO-CAPTURE ON: move/turn to save views; q / ESC finishes.", flush=True)
        pygame.key.set_repeat(250, 150)
    print("use the keyboard to control the agent")
    print("  w / s : forward / backward")
    print("  a / d : turn left / right")
    print("  c or SPACE : capture the current frame")
    print("  q or ESC   : finish and quit")
    print("#############################")

    cam_extr = []   # captured GT poses [x, y, z, qw, qx, qy, qz]
    count = 0
    clock = pygame.time.Clock()
    t0 = time.monotonic()   # session start: flicker phase runs on wall-clock delta

    def observe_now():
        """Re-process the CURRENT sensors at the CURRENT wall-clock t, so the
        flicker oscillation keeps advancing even while the agent is standing
        still (the ZONE it stands in only changes when the agent moves). The
        raw readout itself is cached by the Engine until the agent moves."""
        frame = engine.observe(time.monotonic() - t0)
        sensor_state = engine.agent.get_state().sensor_states["color_sensor"]
        return frame, sensor_state

    frame, sensor_state = observe_now()
    running = True
    while running:
        capture_requested = False
        for event in pygame.event.get():   # non-blocking: keep rendering between inputs
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key in QUIT_KEYS:
                    running = False
                elif event.key in CAPTURE_KEYS:
                    capture_requested = True
                elif event.key in KEY_ACTION:
                    before = engine.agent.get_state().sensor_states["color_sensor"]
                    q = before.rotation
                    old_pose = (np.array(before.position).copy(), np.array([q.w, q.x, q.y, q.z]))
                    engine.step(KEY_ACTION[event.key])
                    after = engine.agent.get_state().sensor_states["color_sensor"]
                    q = after.rotation
                    changed = (not np.allclose(old_pose[0], after.position, atol=1e-7)
                               or abs(float(np.dot(old_pose[1], [q.w, q.x, q.y, q.z]))) < 1.0 - 1e-7)
                    capture_requested |= auto_capture and changed
                # any other key is ignored
            elif event.type in viewer.EXPOSE_EVENTS:
                preview.invalidate()       # window (re)exposed: full repaint
        frame, sensor_state = observe_now()   # continuous time-driven render
        if capture_requested:
            count += 1
            pose = save_frame(frame, sensor_state, data_root, out_cfg, count)
            cam_extr.append(pose)
            print(f"captured frame {count} @ pose "
                  f"({pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f}) "
                  f"({pose[3]:.3f}, {pose[4]:.3f}, {pose[5]:.3f}, {pose[6]:.3f})", flush=True)
        preview.draw(frame, count, fps=clock.get_fps())
        clock.tick(fps)

    np.save(os.path.join(data_root, "GT_pose.npy"),
            np.asarray(cam_extr, dtype=np.float32))
    print(f"saved {len(cam_extr)} poses to {os.path.join(data_root, 'GT_pose.npy')}")


def report_zone_coverage(config, scheduler, data_root):
    """Print how many captured frames landed in each uncertainty zone.

    This is the HW1 specification §3.1 step-3 check made visible at collection time: a
    zone the capture never enters fires on nothing, and the run silently
    degenerates to a clean baseline."""
    from simulator import OUTSIDE, zone_frame_counts

    if scheduler is None or not scheduler.zones:
        return
    gt_path = os.path.join(data_root, "GT_pose.npy")
    if not os.path.exists(gt_path):
        return
    counts = zone_frame_counts(config, np.load(gt_path))
    print("frames per uncertainty zone: "
          + ", ".join(f"{name}={n}" for name, n in counts.items()))
    empty = [name for name, n in counts.items() if n == 0 and name != OUTSIDE]
    if empty:
        print(f"  WARNING: never entered {', '.join(empty)} — those zones "
              "contributed nothing to this capture")


def main():
    args = parse_args()   # argparse first: --help never touches habitat or pygame

    from simulator import Engine, ZoneScheduler, load_config, prepare_capture_dirs

    config = load_config(args.config)
    out_cfg = config["output"]
    if args.output_root:
        out_cfg["root"] = args.output_root
    data_root = out_cfg["root"]
    # Shared capture path: makes rgb/ depth/ [semantic/] AND writes
    # intrinsics.json, so every capture ships its own camera parameters (D5).
    print(f"capture dir: {data_root} (+ intrinsics.json)")
    prepare_capture_dirs(config, data_root)

    unc = config["uncertainties"]
    if args.clean:
        unc["enabled"] = False        # --clean IS uncertainties.enabled: false
    scheduler = None
    if not unc.get("enabled", True):
        print("uncertainty zones disabled (clean collection)")
    else:
        scheduler = ZoneScheduler(unc)
        names = ", ".join(z["name"] for z in scheduler.zones) or "NONE"
        print(f"{len(scheduler.zones)} uncertainty zone(s) armed: {names}")
        if not scheduler.zones:
            print("  WARNING: uncertainties.enabled is true but `zones:` is "
                  "empty — this collection degenerates to a clean baseline")

    fps = args.fps if args.fps and args.fps > 0 else 30.0
    engine = Engine(config, scheduler=scheduler, fps_nominal=fps)

    # Engine constructed -> only now may the viewer set SDL vars + import pygame.
    from simulator import viewer
    pygame = viewer.pygame

    pygame.init()
    font = pygame.font.SysFont(None, 28)
    display_cfg = config["display"]
    # Size the window from a real first frame (true start_position, t=0).
    canvas = viewer.build_canvas(engine.observe(0.0), display_cfg)
    scale = float(display_cfg["scale"])
    screen = pygame.display.set_mode(
        (int(canvas.shape[1] * scale), int(canvas.shape[0] * scale)))
    pygame.display.set_caption(
        "Habitat — AUTO capture: hold W/S/A/D to explore; Q saves and exits"
        if args.auto_capture else "Habitat data collector")
    preview = viewer.Preview(screen, display_cfg, font)   # incremental repaints

    collection_started = time.perf_counter()
    try:
        if args.trajectory:
            run_replay(engine, viewer, preview, args.trajectory, args.fps,
                       data_root, out_cfg)
        else:
            run_interactive(engine, viewer, preview, fps, data_root, out_cfg,
                            auto_capture=args.auto_capture)
        poses = np.load(os.path.join(data_root, "GT_pose.npy"), allow_pickle=False).reshape(-1, 7)
        positions = poses[:, :3]
        metadata = {
            "capture_path": os.path.abspath(data_root),
            "config_path": os.path.abspath(args.config),
            "configuration": config,
            "mode": "replay" if args.trajectory else "interactive",
            "auto_capture": args.auto_capture,
            "trajectory_source": args.trajectory,
            "frame_count": len(poses),
            "collection_seconds": time.perf_counter() - collection_started,
            "sampled_camera_path_length_m": float(np.linalg.norm(
                np.diff(positions, axis=0), axis=1).sum()),
            "route_poses": "GT_pose.npy",
            "route_xz_bounds_m": (
                {"min": positions[:, [0, 2]].min(axis=0).tolist(),
                 "max": positions[:, [0, 2]].max(axis=0).tolist()}
                if len(positions) else None),
        }
        with open(os.path.join(data_root, "capture_metadata.json"), "w", encoding="utf-8") as handle:
            json.dump(metadata, handle, indent=2)
            handle.write("\n")
        report_zone_coverage(config, scheduler, data_root)
    finally:
        pygame.quit()
        engine.close()


if __name__ == "__main__":
    main()
