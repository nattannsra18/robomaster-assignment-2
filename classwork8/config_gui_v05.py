"""Pre-mission configuration GUI for Final Round 1 V05.

The user can tune the important robot/mapping parameters without editing source
code.  Values are applied to the provided Classwork8Config instance only for
the current run.
"""

from __future__ import annotations

from typing import Dict, List, Tuple

from .target_mission import VALID_COLORS, VALID_SHAPES, parse_target_specs


def configure_before_run(config) -> bool:
    import tkinter as tk
    from tkinter import messagebox, ttk

    root = tk.Tk()
    root.title("Classwork 8 V05 - Mission Configuration")
    # Fit short laptop screens and Windows display scaling. The action bar
    # stays anchored below a scrollable parameter notebook.
    screen_w = root.winfo_screenwidth()
    screen_h = root.winfo_screenheight()
    window_w = max(560, min(920, screen_w - 64))
    window_h = max(460, min(720, screen_h - 100))
    root.geometry("{}x{}".format(window_w, window_h))
    root.minsize(min(650, window_w), min(460, window_h))

    accepted = {"value": False}
    variables: Dict[str, object] = {}

    outer = ttk.Frame(root, padding=12)
    outer.pack(fill="both", expand=True)

    ttk.Label(
        outer,
        text="Classwork 8 V05 - Mission Configuration",
        font=("Segoe UI", 17, "bold"),
    ).pack(anchor="w")

    ttk.Label(
        outer,
        text=(
            "Planner: Nearest-Frontier BFS + closed-maze auto completion. "
            "Configure the run here; no source-code editing is required."
        ),
        wraplength=800,
    ).pack(anchor="w", pady=(2, 10))

    notebook = ttk.Notebook(outer)

    # Each tab has its own vertical scrollbar. A long Target Detection tab
    # must never push the bottom Start/Cancel buttons outside the screen.
    tabs = {}
    tab_canvases = {}
    tab_names = (
        "Mission Settings", "Motion", "ToF / Mapping", "Mapping",
        "Target Detection", "Completion / Export",
    )
    for name in tab_names:
        tab_container = ttk.Frame(notebook)
        notebook.add(tab_container, text=name)
        canvas = tk.Canvas(tab_container, highlightthickness=0, borderwidth=0)
        scrollbar = ttk.Scrollbar(
            tab_container, orient="vertical", command=canvas.yview
        )
        canvas.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)

        content = ttk.Frame(canvas, padding=12)
        content_id = canvas.create_window((0, 0), window=content, anchor="nw")

        def _update_scrollregion(_event, active_canvas=canvas):
            active_canvas.configure(scrollregion=active_canvas.bbox("all"))

        def _fit_content(event, active_canvas=canvas, item_id=content_id):
            active_canvas.itemconfigure(item_id, width=event.width)

        content.bind("<Configure>", _update_scrollregion)
        canvas.bind("<Configure>", _fit_content)
        tabs[name] = content
        tab_canvases[name] = canvas

    def _scroll_current_tab(event):
        try:
            name = tab_names[notebook.index(notebook.select())]
            delta = getattr(event, "delta", 0)
            if delta:
                steps = -max(1, abs(int(delta / 120))) if delta > 0 else max(1, abs(int(delta / 120)))
            else:
                steps = -1 if getattr(event, "num", 0) == 4 else 1
            tab_canvases[name].yview_scroll(steps, "units")
        except Exception:
            pass

    root.bind("<MouseWheel>", _scroll_current_tab, add="+")
    root.bind("<Button-4>", _scroll_current_tab, add="+")
    root.bind("<Button-5>", _scroll_current_tab, add="+")

    field_specs: Dict[str, List[Tuple[str, str, str, str]]] = {
        # Quick settings appear FIRST. Advanced tabs reuse the same Tk
        # variables, so changing one control updates its duplicate instantly.
        "Mission Settings": [
            ("unsafe_disable_motion_guards", "UNSAFE: disable all motion guards", "bool", "Operator-supervised foam-maze test only. Bypasses preflight, live ToF/Gimbal holds, ToF braking, yaw abort and cross-track abort. Odometry endpoint and manual Stop remain active."),
            ("wall_clearance_enabled", "Enable 4-direction wall clearance adjustment", "bool", "At THIS direction, shift away immediately when the opposite route is verified; hold for a fresh floor-sign camera check BEFORE advancing. No extra Gimbal yaw scans."),
            ("wall_clearance_front_cm", "FRONT minimum wall range (cm)", "float", "Measured horizontal ToF reading; if too close, cautiously reverse."),
            ("wall_clearance_right_cm", "RIGHT minimum wall range (cm)", "float", "If the right wall is closer than this, cautiously strafe LEFT."),
            ("wall_clearance_back_cm", "BACK minimum wall range (cm)", "float", "If the back wall is closer than this, cautiously move forward."),
            ("wall_clearance_left_cm", "LEFT minimum wall range (cm)", "float", "If the left wall is closer than this, cautiously strafe RIGHT."),
            ("wall_clearance_camera_dwell_sec", "Pause after adjusting, before checking sign (s)", "float", "Keep Gimbal on this same direction at camera pitch and collect fresh camera frames; default 0.70 s."),
            ("travel_speed_mps", "Robot cruise speed (m/s)", "float", "Maximum longitudinal SDK request; live ToF may brake before a wall"),
            ("moving_gimbal_check_enabled", "Moving Gimbal Check (diagnostic)", "bool", "Default ON. Turning it off bypasses only in-motion angle/age checks; initial aim, fresh ToF, hard stop, heading guard and wheel-stop ACK stay ON."),
            ("gimbal_yaw_speed_dps", "Gimbal yaw max speed (deg/s)", "float", "Faster 170 max, Kp 3.6; final pitch and yaw must settle before ToF"),
            ("target_detection_enabled", "Camera target survey", "bool", "Observe targets in each newly scanned cell"),
            ("stationary_target_test", "Stationary target/aim test", "bool", "One four-direction scan and auto-aim cycle, then stop without chassis translation."),
            ("target_fire_mode", "Target firing mode", "choice", "off = observe only; selected = fire checked color/shape targets; all = fire every verified target ID."),
            ("target_fire_type", "Blaster type", "choice", "IR is recommended; water requires correctly loaded gel beads."),
            ("target_fire_times", "Shots per target", "int", "Number of IR/water commands for each target, 1-5."),
            ("target_aim_offset_x_ratio", "Blaster aim X offset (-0.25 to 0.25)", "float", "Calibrated desired centroid offset; start at 0.0 and tune only from stationary tests."),
            ("target_aim_offset_y_ratio", "Blaster aim Y offset (-0.25 to 0.25)", "float", "Calibrated desired centroid offset; positive moves the desired point down in the image."),
            ("target_survey_open_directions", "Detect targets along open corridors", "bool", "Distant signs become unlocalized camera sightings, not false target positions"),
            ("target_camera_pitch_deg", "Camera look-down pitch (deg)", "float", "Default -20 deg for ground signs; camera only while stopped"),
            ("target_roi_bottom_ratio", "Target ROI bottom (0-1)", "float", "Default 0.96 for lower signs; reduce if floor reflections are detected"),
            ("closed_maze_auto_stop", "Exact 6x6 auto completion", "bool", "Stops only after all 36 cells in the declared 6x6 map are visited"),
            ("gui_auto_save_map", "Auto-export GUI map PNG", "bool", "Writes gui_map.png alongside mission logs"),
        ],
        "Motion": [
            ("cell_size_m", "Cell size (m)", "float", "Physical maze cell; assignment default = 0.60"),
            ("step_tolerance_m", "Cell stop tolerance (m)", "float", "Stable V1 default 0.02 means stop at about 58 cm, then tune from measured trials"),
            ("cell_center_tolerance_m", "Lateral cell tolerance (m)", "float", "Default 0.06 tolerates mecanum slip without demanding exact centring"),
            ("travel_speed_mps", "Cruise speed (m/s)", "float", "Maximum speed outside the ToF brake zone"),
            ("slow_front_cm", "ToF brake start (cm)", "float", "Reduce translation speed continuously below this range"),
            ("stop_front_cm", "ToF hard stop (cm)", "float", "Always stop at or below this range; never treated as cell arrival"),
            ("movement_brake_min_speed_mps", "Minimum braking speed (m/s)", "float", "Lowest commanded approach speed before the independent hard stop"),
            ("movement_endpoint_brake_distance_m", "Odometry endpoint brake distance (m)", "float", "Begin tapering speed this far before the 60 cm odometry endpoint"),
            ("movement_preflight_margin_cm", "Preflight margin (cm)", "float", "Extra range required after travel-to-tolerance plus hard-stop reserve"),
            ("odom_scale_x", "Odometry scale X", "float", "Start at 1.00; tune with a measured 60 cm forward test"),
            ("odom_scale_y", "Odometry scale Y", "float", "Start at 1.00; tune with a measured 60 cm strafe test"),
            ("wall_clearance_enabled", "Adjust clearance after full scan", "bool", "No opposite probe or late correction: use same-sweep opposite range or short proven reverse of last traversed cell; otherwise skip unsafe movement."),
            ("wall_clearance_front_cm", "Front minimum range (cm)", "float", "Horizontal ToF distance to front wall"),
            ("wall_clearance_right_cm", "Right minimum range (cm)", "float", "Horizontal ToF distance to right wall"),
            ("wall_clearance_back_cm", "Back minimum range (cm)", "float", "Horizontal ToF distance to rear wall"),
            ("wall_clearance_left_cm", "Left minimum range (cm)", "float", "Horizontal ToF distance to left wall"),
            ("wall_clearance_deadband_cm", "Clearance tolerance (cm)", "float", "Avoid tiny repeated correction near target; default 0.5 cm"),
            ("wall_clearance_max_step_cm", "Maximum shift per scan (cm)", "float", "Hard limit for one corrective move, default 4 cm"),
            ("wall_clearance_speed_mps", "Clearance adjustment speed (m/s)", "float", "Slow translation with z=0, default 0.035 m/s"),
            ("wall_clearance_camera_dwell_sec", "Post-shift camera dwell (s)", "float", "Stationary dwell at current scan direction before target verification; default 0.70 s."),
            ("heading_kp_z", "Heading Kp", "float", "Yaw correction gain"),
            ("heading_deadband_deg", "Heading deadband (deg)", "float", "Ignore tiny yaw noise"),
            ("heading_max_z_dps", "Max yaw correction speed", "float", "Normal heading correction limit"),
            ("heading_drive_sign", "Yaw correction sign (+1 or -1)", "float", "Validate with chassis attitude before reversing"),
            ("heading_align_tolerance_deg", "Post-scan yaw tolerance (deg)", "float", "Align only when yaw error exceeds this"),
            ("heading_align_max_z_dps", "Post-scan turn speed (deg/s)", "float", "Bounded yaw-only checkpoint correction"),
        ],
        "ToF / Mapping": [
            ("tof_open_cm", "Open direction threshold (cm)", "float", ">= this value is an open candidate"),
            ("scan_hard_wall_cm", "Hard-wall threshold (cm)", "float", "<= this is confidently a wall"),
            ("scan_samples", "Scan samples", "int", "Median samples per gimbal direction"),
            ("scan_sample_interval_sec", "Scan sample interval (s)", "float", "Delay between ToF samples"),
            ("scan_cell_budget_sec", "New-cell scan budget (s)", "float", "Target 6-8 seconds; optional camera work is skipped after this budget, but required topology safety may finish"),
            ("gimbal_yaw_speed_dps", "Yaw max speed (deg/s)", "float", "Faster default 170 max, Kp 3.6, SDK cap in config 180"),
            ("gimbal_min_yaw_speed_dps", "Yaw minimum speed (deg/s)", "float", "Low-speed correction near a requested scan direction"),
            ("gimbal_yaw_kp", "Yaw correction Kp", "float", "Smooth proportional yaw-only controller"),
            ("gimbal_tolerance_deg", "Yaw settle tolerance (deg)", "float", "Default 2.5; avoids stopping on harmless +2.2 deg end-settle noise"),
            ("gimbal_turn_timeout_sec", "Gimbal phase timeout (s)", "float", "Each PRE pitch, yaw and POST pitch gets its own timeout"),
            ("gimbal_yaw_pitch_guard_deg", "Pitch warning during yaw (deg)", "float", "Log a transient warning; mapping still requires pitch/yaw level before ToF"),
            ("gimbal_scan_pitch_deg", "Scan pitch target (deg)", "float", "Horizontal relative gimbal pitch; start with 0 degrees"),
            ("gimbal_pitch_kp", "Pitch correction Kp", "float", "Use a gentle gain to avoid nodding during yaw sweeps"),
            ("gimbal_pitch_min_speed_dps", "Minimum pitch speed (deg/s)", "float", "Lower than the previous 4 deg/s to reduce overshoot near level"),
            ("gimbal_pitch_max_speed_dps", "Maximum pitch speed (deg/s)", "float", "Limit visible pitch movement"),
            ("gimbal_pitch_drive_sign", "Pitch direction sign (+1/-1)", "float", "Faster default pitch max 38 dps, only reverse after stationary sign test"),
            ("gimbal_pitch_tolerance_deg", "Pitch tolerance (deg)", "float", "0.8 deg default; earlier 2 deg allowed noticeable nodding"),
            ("gimbal_pitch_unsafe_deg", "Legacy pitch threshold (deg)", "float", "Retained for compatibility; Stable V1 uses the separate moving pitch tolerance"),
            ("moving_gimbal_feedback_max_age_sec", "Moving feedback max age (s)", "float", "Pause movement when Gimbal or ToF feedback is older than this"),
            ("moving_gimbal_pitch_tolerance_deg", "Moving pitch tolerance (deg)", "float", "Diagnostic in-motion limit; stationary scan checks remain stricter"),
            ("moving_gimbal_yaw_tolerance_deg", "Moving yaw tolerance (deg)", "float", "Diagnostic in-motion limit relative to the travel direction"),
            ("moving_gimbal_bad_samples", "Bad samples before hold", "int", "Debounce: one transient sample does not stop the chassis"),
            ("moving_feedback_recovery_samples", "Fresh samples to resume", "int", "Consecutive distinct ToF/Gimbal updates required while wheel-stopped"),
            ("moving_feedback_recovery_timeout_sec", "Feedback recovery timeout (s)", "float", "After this timeout the mission reports an error; it does not backtrack"),
        ],
        "Mapping": [
            ("resolution_m", "Occupancy resolution (m)", "float", "Assignment map resolution; default = 0.05"),
            ("map_width_m", "Working canvas width (m)", "float", "Internal export canvas, not prior field knowledge"),
            ("map_height_m", "Working canvas height (m)", "float", "Internal export canvas, not prior field knowledge"),
            ("max_moves", "Maximum cell moves", "int", "Safety cap"),
            ("free_delta", "Free evidence delta", "int", "Occupancy evidence update"),
            ("occupied_delta", "Occupied evidence delta", "int", "Occupancy evidence update"),
        ],
        "Completion / Export": [
            ("closed_maze_auto_stop", "Exact 6x6 auto stop", "bool", "Stop when all 36 logical cells forming the declared 6x6 arena have been visited"),
            ("closed_maze_perimeter_wall_ratio", "Perimeter wall diagnostic ratio", "float", "Reported for map quality; no longer blocks completion after all 36 cells are visited"),
            ("assignment_maze_rows", "Required maze rows", "int", "Assignment-fixed value: 6"),
            ("assignment_maze_cols", "Required maze columns", "int", "Assignment-fixed value: 6"),
            ("mission_warning_sec", "Mission warning time (s)", "float", "Default 420 seconds = 7 minutes"),
            ("mission_soft_deadline_sec", "Mission soft deadline (s)", "float", "Default 525 seconds = urgency warning; exploration continues"),
            ("gui_auto_save_map", "Auto-save GUI map PNG", "bool", "Save gui_map.png in the same run output folder when the mission finishes"),
            ("gui_export_width_px", "GUI export width (px)", "int", "PNG export width"),
            ("gui_export_height_px", "GUI export height (px)", "int", "PNG export height"),
        ],
        "Target Detection": [
            ("target_detection_enabled", "Enable camera target survey", "bool", "Round 1 detects color + shape while the gimbal already scans ToF"),
            ("stationary_target_test", "Stationary target/aim test", "bool", "Never enters the movement planner; useful before any live driving test"),
            ("target_camera_resolution", "Camera resolution", "choice", "360p is recommended for low latency"),
            ("target_camera_pitch_deg", "Target observation pitch (deg)", "float", "Default -20 for ground signs; horizontal ToF remains 0"),
            ("target_preview_fps", "Live preview FPS", "float", "Independent annotated camera preview; default 8"),
            ("target_survey_open_directions", "Survey OPEN directions too", "bool", "Find low signs even if horizontal ToF says the path ahead is open (range marked unconfirmed)"),
            ("target_min_confidence", "Candidate confidence", "float", "Reject weak single-frame detections below this value"),
            ("target_save_confidence", "Save confidence", "float", "Temporal track must exceed this value before entering targets.json"),
            ("target_quick_gate_frames", "Quick candidate frames", "int", "Use 1-2 fresh frames; full verification runs only after a candidate appears"),
            ("target_verify_frames", "Required matching frames", "int", "Minimum repeated detections before a target is verified"),
            ("target_frame_interval_sec", "Frame interval (s)", "float", "Small delay between temporal verification samples"),
            ("target_verify_max_jump_px", "Max centroid jump (px)", "float", "Keeps temporal verification on the same object"),
            ("target_merge_centroid_px", "Same-view merge threshold (px)", "float", "Keep adjacent same-color signs separate; default 18px"),
            ("target_fire_mode", "Target firing mode", "choice", "Choose off, selected targets, or every verified target"),
            ("target_fire_type", "Blaster fire type", "choice", "Choose ir or water; default ir"),
            ("target_fire_times", "Shots per selected target", "int", "Default 1; command acknowledgement is logged, not physical-hit confirmation"),
            ("target_max_fire_distance_cells", "Maximum firing range (cells)", "float", "Assignment rule: no more than 2 cells"),
            ("target_aim_offset_x_ratio", "Camera-to-blaster X offset ratio", "float", "Desired target centroid relative to image centre; stationary calibration only"),
            ("target_aim_offset_y_ratio", "Camera-to-blaster Y offset ratio", "float", "Desired target centroid relative to image centre; stationary calibration only"),
            ("target_camera_above_blaster_m", "Camera above blaster (m)", "float", "Physical vertical separation; 0.05 m for this robot, used by FRONT-only parallax correction"),
            ("target_camera_horizontal_fov_deg", "Camera horizontal FOV (deg)", "float", "DJI specification is 120 deg; used with frame aspect ratio for parallax correction"),
            ("target_aim_tolerance_ratio", "Aim tolerance (frame ratio)", "float", "Must remain inside this gate for consecutive fresh frames; default 0.05"),
            ("target_auto_aim_stable_frames", "Fresh centered frames before fire", "int", "Default 2; cached frames never count twice"),
            ("target_auto_aim_timeout_sec", "Auto-aim timeout (s)", "float", "Failure stops aiming and suppresses the shot"),
            ("target_auto_aim_max_yaw_delta_deg", "Maximum aim yaw travel (deg)", "float", "Bounded deviation from the scanned wall direction"),
            ("target_auto_aim_max_pitch_delta_deg", "Maximum aim pitch travel (deg)", "float", "Bounded deviation from the selected camera pitch"),
            ("target_auto_aim_yaw_drive_sign", "Auto-aim yaw sign (+1/-1)", "float", "Reverse only after a stationary dry-run proves image error grows"),
            ("target_auto_aim_pitch_drive_sign", "Auto-aim pitch sign (+1/-1)", "float", "Reverse only after a stationary dry-run proves image error grows"),
            ("target_clahe_clip_limit", "CLAHE clip limit", "float", "Lighting normalization strength on Lab-L"),
            ("target_roi_top_ratio", "Target ROI top (0-1)", "float", "Exclude non-target ceiling/background; tune using the camera debug frame"),
            ("target_roi_bottom_ratio", "Target ROI bottom (0-1)", "float", "Ground-level targets may be below 0.82; default 0.94, then tune using live ROI slider"),
            ("target_roi_border_margin_px", "ROI border margin (px)", "int", "Reject cropped contours touching the detection region"),
            ("target_min_contour_area_px", "Minimum contour area (px)", "float", "Reject small tape and floor noise"),
            ("target_rectangularity_min", "Rectangle fill minimum", "float", "Square/rectangle geometry threshold"),
            ("target_square_aspect_min", "Square aspect min", "float", "Lower W/H bound for square"),
            ("target_square_aspect_max", "Square aspect max", "float", "Upper W/H bound for square"),
            ("target_circle_circularity_min", "Circle circularity min", "float", "Circle geometry threshold"),
        ],
    }

    help_labels = []

    def add_field(parent, attr, label, kind, help_text, row):
        ttk.Label(parent, text=label).grid(
            row=row, column=0, sticky="w", padx=(0, 10), pady=5
        )

        current = getattr(config, attr)
        if attr in variables:
            var, previous_kind = variables[attr]
            if previous_kind != kind:
                raise ValueError("Conflicting GUI field type for {}".format(attr))
        else:
            var = (
                tk.BooleanVar(value=bool(current))
                if kind == "bool"
                else tk.StringVar(value=str(current))
            )
            variables[attr] = (var, kind)

        if kind == "bool":
            widget = ttk.Checkbutton(parent, variable=var)
        elif kind == "choice":
            choice_values = {
                "target_fire_mode": ("off", "selected", "all"),
                "target_fire_type": ("ir", "water"),
            }
            widget = ttk.Combobox(
                parent,
                textvariable=var,
                values=choice_values.get(attr, ("360p", "540p", "720p")),
                state="readonly",
                width=18,
            )
        else:
            widget = ttk.Entry(parent, textvariable=var, width=20)
        widget.grid(row=row, column=1, sticky="ew", pady=5)

        help_label = ttk.Label(
            parent,
            text=help_text,
            foreground="#64748b",
            wraplength=390,
        )
        help_label.grid(row=row, column=2, sticky="w", padx=(12, 0), pady=5)
        help_labels.append(help_label)

    for tab_name, specs in field_specs.items():
        parent = tabs[tab_name]
        parent.columnconfigure(1, weight=0)
        parent.columnconfigure(2, weight=1)

        for row, spec in enumerate(specs):
            add_field(parent, *spec, row=row)

    selected_specs = parse_target_specs(config.target_required_specs)
    target_spec_vars = {}
    target_frame = ttk.LabelFrame(
        tabs["Mission Settings"],
        text="Selected color + shape targets",
        padding=8,
    )
    target_frame.grid(
        row=len(field_specs["Mission Settings"]),
        column=0,
        columnspan=3,
        sticky="ew",
        pady=(10, 4),
    )
    for column, shape in enumerate(sorted(VALID_SHAPES), start=1):
        ttk.Label(target_frame, text=shape.title()).grid(
            row=0, column=column, padx=8, pady=2
        )
    for row, color in enumerate(sorted(VALID_COLORS), start=1):
        ttk.Label(target_frame, text=color.title()).grid(
            row=row, column=0, sticky="w", padx=(0, 8), pady=2
        )
        for column, shape in enumerate(sorted(VALID_SHAPES), start=1):
            key = "{}:{}".format(color, shape)
            var = tk.BooleanVar(
                value=any(spec.key == key for spec in selected_specs)
            )
            target_spec_vars[key] = var
            ttk.Checkbutton(target_frame, variable=var).grid(
                row=row, column=column, padx=8, pady=2
            )

    info = ttk.LabelFrame(outer, text="60 cm calibration", padding=10)
    # Pack the footer after the action bar is created so the Start button
    # remains on the bottom edge of the window.

    calibration_var = tk.StringVar()
    ttk.Label(
        info,
        textvariable=calibration_var,
        wraplength=790,
        justify="left",
    ).pack(anchor="w")

    def refresh_calibration_text(*_args):
        try:
            cell = float(variables["cell_size_m"][0].get())
            tol = float(variables["step_tolerance_m"][0].get())
            sx = float(variables["odom_scale_x"][0].get())
            sy = float(variables["odom_scale_y"][0].get())
            calibration_var.set(
                "Controller target: {:.1f} cm before tolerance; "
                "completion threshold ≈ {:.1f} cm in calibrated map coordinates. "
                "Current odom scale X/Y = {:.3f}/{:.3f}. "
                "For a tape-measure test, if a commanded 60 cm move physically "
                "travels D cm, adjust scale approximately to old_scale × D/60.".format(
                    cell * 100.0,
                    max(0.0, cell - tol) * 100.0,
                    sx,
                    sy,
                )
            )
        except Exception:
            calibration_var.set("Enter valid numeric motion values to see calibration guidance.")

    for attr in ("cell_size_m", "step_tolerance_m", "odom_scale_x", "odom_scale_y"):
        variables[attr][0].trace_add("write", refresh_calibration_text)

    refresh_calibration_text()

    button_row = ttk.Frame(outer)
    button_row.pack(side="bottom", fill="x", pady=(8, 0))
    info.pack(side="bottom", fill="x", pady=(8, 0))

    def set_v05_defaults():
        defaults = {
            "unsafe_disable_motion_guards": True,
            "cell_size_m": 0.60,
            "step_tolerance_m": 0.02,
            "travel_speed_mps": 0.30,
            "slow_front_cm": 35.0,
            "stop_front_cm": 18.0,
            "movement_brake_min_speed_mps": 0.06,
            "movement_endpoint_brake_distance_m": 0.18,
            "movement_preflight_margin_cm": 0.0,
            "cell_center_tolerance_m": 0.060,
            "moving_gimbal_check_enabled": True,
            "moving_gimbal_feedback_max_age_sec": 0.35,
            "moving_gimbal_pitch_tolerance_deg": 3.0,
            "moving_gimbal_yaw_tolerance_deg": 5.0,
            "moving_gimbal_bad_samples": 3,
            "moving_feedback_recovery_samples": 3,
            "moving_feedback_recovery_timeout_sec": 2.50,
            "odom_scale_x": 1.00,
            "odom_scale_y": 1.00,
            "wall_clearance_enabled": False,
            "wall_clearance_front_cm": 15.0,
            "wall_clearance_right_cm": 15.0,
            "wall_clearance_back_cm": 15.0,
            "wall_clearance_left_cm": 15.0,
            "wall_clearance_deadband_cm": 0.5,
            "wall_clearance_max_step_cm": 4.0,
            "wall_clearance_speed_mps": 0.035,
            "wall_clearance_camera_dwell_sec": 0.70,
            "heading_kp_z": 2.4,
            "heading_deadband_deg": 0.35,
            "heading_max_z_dps": 18.0,
            "heading_drive_sign": 1.0,
            "heading_align_tolerance_deg": 1.5,
            "heading_align_max_z_dps": 10.0,
            "tof_open_cm": 55.0,
            "scan_hard_wall_cm": 25.0,
            "scan_samples": 5,
            "scan_sample_interval_sec": 0.06,
            "scan_cell_budget_sec": 8.0,
            "skip_scanned_visited_cells": True,
            "gimbal_yaw_speed_dps": 170.0,
            "gimbal_min_yaw_speed_dps": 9.0,
            "gimbal_yaw_kp": 3.6,
            "gimbal_tolerance_deg": 2.5,
            "gimbal_turn_timeout_sec": 8.0,
            "gimbal_yaw_pitch_guard_deg": 6.0,
            "gimbal_scan_pitch_deg": 0.0,
            "gimbal_pitch_kp": 2.2,
            "gimbal_pitch_min_speed_dps": 4.0,
            "gimbal_pitch_max_speed_dps": 38.0,
            "gimbal_pitch_drive_sign": 1.0,
            "gimbal_pitch_tolerance_deg": 0.8,
            "gimbal_pitch_unsafe_deg": 6.0,
            "resolution_m": 0.05,
            "map_width_m": 8.0,
            "map_height_m": 8.0,
            "max_moves": 500,
            "free_delta": -2,
            "occupied_delta": 5,
            "closed_maze_auto_stop": True,
            "closed_maze_perimeter_wall_ratio": 0.70,
            "closed_maze_min_rows": 2,
            "closed_maze_min_cols": 2,
            "assignment_maze_rows": 6,
            "assignment_maze_cols": 6,
            "mission_warning_sec": 420.0,
            "mission_soft_deadline_sec": 525.0,
            "gui_auto_save_map": True,
            "gui_export_width_px": 1200,
            "gui_export_height_px": 900,
            "target_detection_enabled": True,
            "stationary_target_test": False,
            "target_fire_mode": "off",
            "target_required_specs": "",
            "target_fire_type": "ir",
            "target_fire_times": 1,
            "target_max_fire_distance_cells": 2.0,
            "target_aim_offset_x_ratio": 0.0,
            "target_aim_offset_y_ratio": 0.0,
            "target_camera_above_blaster_m": 0.05,
            "target_camera_horizontal_fov_deg": 120.0,
            "target_aim_tolerance_ratio": 0.05,
            "target_auto_aim_stable_frames": 2,
            "target_auto_aim_timeout_sec": 4.0,
            "target_auto_aim_feedback_max_age_sec": 0.35,
            "target_auto_aim_max_lost_frames": 5,
            "target_auto_aim_max_jump_px": 100.0,
            "target_auto_aim_min_speed_dps": 9.0,
            "target_auto_aim_max_speed_dps": 25.0,
            "target_auto_aim_gain_dps_per_ratio": 120.0,
            "target_auto_aim_pulse_sec": 0.06,
            "target_auto_aim_settle_sec": 0.06,
            "target_auto_aim_max_yaw_delta_deg": 12.0,
            "target_auto_aim_max_pitch_delta_deg": 10.0,
            "target_auto_aim_yaw_drive_sign": 1.0,
            "target_auto_aim_pitch_drive_sign": 1.0,
            "target_auto_aim_divergence_ratio": 0.02,
            "target_camera_resolution": "360p",
            "target_camera_pitch_deg": -20.0,
            "target_preview_fps": 10.0,
            "target_survey_open_directions": True,
            "target_min_confidence": 0.50,
            "target_save_confidence": 0.60,
            "target_quick_gate_frames": 2,
            "target_verify_frames": 4,
            "target_frame_interval_sec": 0.040,
            "target_verify_max_jump_px": 50.0,
            "target_merge_centroid_px": 18.0,
            "target_merge_distance_m": 0.40,
            "target_clahe_clip_limit": 2.0,
            "target_roi_top_ratio": 0.18,
            "target_roi_bottom_ratio": 0.96,
            "target_roi_border_margin_px": 3,
            "target_min_contour_area_px": 300.0,
            "target_rectangularity_min": 0.58,
            "target_square_aspect_min": 0.85,
            "target_square_aspect_max": 1.16,
            "target_circle_circularity_min": 0.70,
        }

        for attr, value in defaults.items():
            if attr not in variables:
                continue
            var, kind = variables[attr]
            if kind == "bool":
                var.set(bool(value))
            else:
                var.set(str(value))
        for var in target_spec_vars.values():
            var.set(False)

    def apply_and_start():
        try:
            for attr, (var, kind) in variables.items():
                if kind == "bool":
                    value = bool(var.get())
                elif kind == "int":
                    value = int(var.get())
                elif kind == "float":
                    value = float(var.get())
                else:
                    value = str(var.get())

                setattr(config, attr, value)

            config.target_required_specs = ",".join(
                key for key, var in sorted(target_spec_vars.items())
                if bool(var.get())
            )
            config.target_fire_enabled = config.target_fire_mode != "off"

            # One logical step is always exactly one physical cell.
            config.exploration_step_m = float(config.cell_size_m)
            config.validate()

        except Exception as exc:
            messagebox.showerror(
                "Invalid configuration",
                str(exc),
                parent=root,
            )
            return

        accepted["value"] = True
        root.destroy()

    def cancel():
        accepted["value"] = False
        root.destroy()

    ttk.Button(
        button_row,
        text="Reset V05 defaults",
        command=set_v05_defaults,
    ).pack(side="left")

    ttk.Button(
        button_row,
        text="Cancel",
        command=cancel,
    ).pack(side="right", padx=(8, 0))

    ttk.Button(
        button_row,
        text="Apply & Connect",
        command=apply_and_start,
    ).pack(side="right")

    # Packing this last leaves the button and calibration footer permanently
    # visible, while the selected tab receives the remaining height.
    notebook.pack(side="top", fill="both", expand=True)
    root.protocol("WM_DELETE_WINDOW", cancel)
    root.mainloop()

    return bool(accepted["value"])
