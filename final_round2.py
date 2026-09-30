"""Replay the Round 1 map and revisit marked targets; default is offline planning.

No blaster commands are issued. Start physically at Round 1 start_cell with the
same heading. This entrypoint performs navigation and camera inspection only.
"""
from __future__ import annotations

import argparse
from dataclasses import fields
import json
import math
from pathlib import Path
import time

from classwork8.round2 import VECTORS, plan_round2, read_json


def execute(plan, speed, connection, report_path):
    # Keep --help and offline planning usable without RoboMaster/OpenCV installed.
    from robomaster import robot
    from classwork8.config import Classwork8Config
    from classwork8.camera_service import CameraService
    from classwork8.target_detection import TargetDetector
    from classwork8.tof_camera_round1_v05 import (
        V05PoseTracker, GimbalTracker, ToFOnlySensorManager, stop_chassis,
        _point_gimbal, _set_camera_observation_pitch, _map_xy_from_raw,
        _basic_motion_command, _align_chassis_after_scan,
    )

    summary = read_json(Path(plan["source_run"]) / "summary.json")
    names = {f.name for f in fields(Classwork8Config)}
    config = Classwork8Config(**{k: v for k, v in summary["config"].items() if k in names})
    if not math.isclose(config.cell_size_m, plan["cell_size_m"], abs_tol=1e-6):
        raise ValueError("summary.json and topology.json cell sizes disagree")
    config.travel_speed_mps = speed
    config.heading_hold_enabled = True
    config.yaw_isolation_mode = False
    config.gimbal_scan_pitch_deg = 0.0
    config.validate()
    if any(not math.isfinite(v) or v <= 0 for v in
           (config.odom_scale_x, config.odom_scale_y)):
        raise ValueError("Invalid saved odometry calibration")

    class FreshPose(V05PoseTracker):
        position_time = 0.0

        def position_callback(self, data):
            if data is not None and len(data) >= 3 and all(math.isfinite(float(v)) for v in data[:3]):
                super().position_callback(data)
                self.position_time = time.monotonic()

    ep = robot.Robot()
    pose, tracker, sensors = FreshPose(), GimbalTracker(), ToFOnlySensorManager()
    camera = None
    chassis = None
    subscriptions = []
    report = {"plan": plan, "status": "RUNNING", "events": [],
              "last_completed_cell": plan["start_cell"]}

    def save():
        report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")

    def fresh():
        now = time.monotonic()
        age = pose.attitude_age_sec()
        if now - pose.position_time > 0.5 or age is None or age > 0.5:
            raise RuntimeError("STALE_POSE: movement stopped")
        if pose.get_yaw() is None or not math.isfinite(pose.get_yaw()):
            raise RuntimeError("INVALID_YAW")

    def point(direction):
        stop_chassis(chassis)
        if not _point_gimbal(ep.gimbal, sensors, tracker, direction, config, None):
            raise RuntimeError("GIMBAL_UNAVAILABLE")

    def follow(route):
        nonlocal current
        if not route:
            return
        if tuple(route[0]) != current:
            raise RuntimeError("Route does not start at current cell")
        for destination in route[1:]:
            destination = tuple(destination)
            delta = (destination[0] - current[0], destination[1] - current[1])
            direction = VECTORS.index(delta)
            point(direction)
            ok, reason = _align_chassis_after_scan(chassis, pose, config, yaw0, None)
            if not ok:
                raise RuntimeError(reason)
            sensors.reset_filters()
            deadline = time.monotonic() + 1.5
            while sensors.get_front_cm() is None and time.monotonic() < deadline:
                time.sleep(0.02)
            target = ((destination[0] - origin[0]) * config.cell_size_m,
                      (destination[1] - origin[1]) * config.cell_size_m)
            deadline = time.monotonic() + max(5.0, config.cell_size_m / speed * 2.5)
            try:
                while True:
                    fresh()
                    x, y = pose.get_xy()
                    mx, my = _map_xy_from_raw(x, y, x0, y0, yaw0,
                                              config.odom_scale_x, config.odom_scale_y)
                    dx, dy = target[0] - mx, target[1] - my
                    remaining = dx * delta[0] + dy * delta[1]
                    cross = abs(dx * delta[1] - dy * delta[0])
                    if cross > 0.10:
                        raise RuntimeError("CROSS_TRACK_ERROR: reposition robot before retry")
                    if remaining < -0.05:
                        raise RuntimeError("CELL_OVERSHOOT")
                    if remaining <= min(0.015, config.cell_size_m * 0.05):
                        break
                    distance = sensors.get_front_cm()
                    if distance is None or distance < 18.0:
                        raise RuntimeError("TOF_MISSING_OR_BLOCKED: route halted")
                    if time.monotonic() > deadline:
                        raise RuntimeError("MOVE_TIMEOUT")
                    vx, vy, vz, error = _basic_motion_command(config, direction, yaw0, pose.get_yaw())
                    if error is None or abs(error) > 10.0:
                        raise RuntimeError("HEADING_ERROR")
                    # SDK watchdog prevents a persistent last command on interruption.
                    chassis.drive_speed(x=vx, y=vy, z=vz, timeout=0.3)
                    time.sleep(0.04)
            finally:
                stop_chassis(chassis)
            current = destination
            report["last_completed_cell"] = current
            report["events"].append({"status": "ARRIVED", "cell": current})
            save()

    try:
        save()
        if not ep.initialize(conn_type=connection):
            raise RuntimeError("RoboMaster connection failed")
        chassis = ep.chassis
        if not ep.set_robot_mode(mode=robot.FREE):
            raise RuntimeError("FREE mode failed")
        stop_chassis(chassis)
        for subscribe, unsubscribe, kwargs in (
            (chassis.sub_position, chassis.unsub_position, dict(cs=1, freq=20, callback=pose.position_callback)),
            (chassis.sub_attitude, chassis.unsub_attitude, dict(freq=20, callback=pose.attitude_callback)),
            (ep.sensor.sub_distance, ep.sensor.unsub_distance, dict(freq=20, callback=sensors.tof_callback)),
            (ep.gimbal.sub_angle, ep.gimbal.unsub_angle, dict(freq=20, callback=tracker.callback)),
        ):
            if not subscribe(**kwargs):
                raise RuntimeError("Sensor subscription failed")
            subscriptions.append(unsubscribe)
        deadline = time.monotonic() + 5.0
        while not pose.has_position() or pose.get_yaw() is None:
            if time.monotonic() > deadline:
                raise RuntimeError("Initial pose timeout")
            time.sleep(0.02)
        fresh()
        x0, y0 = pose.get_xy()
        yaw0 = pose.get_yaw()
        origin = current = tuple(plan["start_cell"])
        camera = CameraService(ep, resolution=config.target_camera_resolution)
        if not camera.start():
            raise RuntimeError("Camera unavailable")
        detector = TargetDetector(config)
        for stop in plan["stops"]:
            follow(stop["route"])
            point(stop["view_direction"])
            if not _set_camera_observation_pitch(ep.gimbal, tracker, config,
                                                  config.target_camera_pitch_deg, None):
                raise RuntimeError("Camera pitch unavailable")
            time.sleep(config.target_camera_settle_sec)
            verified, _ = detector.verify_latest(camera, not_before=time.monotonic())
            matches = [v for v in verified if v.detection.color == stop["color"]
                       and v.detection.shape == stop["shape"]]
            # Colour/shape agreement is evidence of a sighting, not physical identity.
            event = {"target_id": stop["target_id"], "cell": current,
                     "status": "MATCHING_SIGN_SEEN" if matches else "NOT_FOUND",
                     "matching_sign_count": len(matches), "fired": False}
            report["events"].append(event)
            print(json.dumps(event), flush=True)
            save()
        follow(plan["return_route"])
        report["status"] = "VISITS_COMPLETED"
    except BaseException as exc:
        report["status"] = "ABORTED"
        report["error"] = str(exc) or type(exc).__name__
        raise
    finally:
        cleanup_errors = []
        actions = [lambda: stop_chassis(chassis)]
        if chassis is not None:
            actions.append(lambda: ep.gimbal.drive_speed(pitch_speed=0, yaw_speed=0))
        if camera is not None:
            actions.append(camera.stop)
        actions.extend(reversed(subscriptions))
        actions.append(ep.close)
        for action in actions:
            try:
                action()
            except Exception as exc:
                cleanup_errors.append(str(exc))
        if cleanup_errors:
            report["cleanup_errors"] = cleanup_errors
            report["status"] = "CLEANUP_FAILED"
        save()
        if cleanup_errors:
            raise RuntimeError("Cleanup failed: " + "; ".join(cleanup_errors))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path, help="Round 1 directory containing topology.json and targets.json")
    parser.add_argument("--execute", action="store_true", help="Connect and physically visit targets (no firing)")
    parser.add_argument("--target", action="append", dest="targets", help="Visit only this ID; repeat to select more")
    parser.add_argument("--return-home", action="store_true")
    parser.add_argument("--speed", type=float, default=0.10)
    parser.add_argument("--connection", choices=("ap", "sta"), default="ap")
    parser.add_argument("--output", type=Path, default=Path("round2_plan.json"))
    parser.add_argument("--report", type=Path, default=Path("round2_report.json"))
    args = parser.parse_args()
    if not math.isfinite(args.speed) or not 0.03 <= args.speed <= 0.20:
        parser.error("--speed must be between 0.03 and 0.20 m/s")
    sources = { (args.run_dir / name).resolve() for name in
                ("topology.json", "targets.json", "summary.json") }
    if args.output.resolve() in sources or args.report.resolve() in sources or args.output.resolve() == args.report.resolve():
        parser.error("Output/report must be distinct and must not overwrite Round 1 inputs")
    try:
        plan = plan_round2(args.run_dir, args.targets, args.return_home)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(plan, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"Plan: {args.output} | targets: {len(plan['stops'])} | skipped: {len(plan['skipped'])}")
        if args.execute:
            if not plan["stops"]:
                raise ValueError("No reachable target views; robot was not connected")
            args.report.parent.mkdir(parents=True, exist_ok=True)
            execute(plan, args.speed, args.connection, args.report)
    except (ValueError, KeyError, OSError, RuntimeError) as exc:
        parser.exit(1, f"Round 2 failed: {exc}\n")
    except KeyboardInterrupt:
        parser.exit(130, "Round 2 stopped by operator.\n")


if __name__ == "__main__":
    main()
