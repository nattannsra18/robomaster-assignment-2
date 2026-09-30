"""Small operator GUI for choosing a saved-map Round-2 start pose."""

from pathlib import Path
from typing import Dict, Optional, Tuple

from .round2_mission import (
    DIR_NAME,
    build_round2_plan,
    load_round1_artifacts,
    save_round2_plan,
)


Cell = Tuple[int, int]


def run_round2_plan_gui(
    run_dir: Path,
    output_path: Optional[Path] = None,
) -> Optional[Path]:
    """Let the operator select start cell, chassis facing, and target set."""
    import tkinter as tk
    from tkinter import messagebox, ttk

    run_dir = Path(run_dir)
    topology, target_payload = load_round1_artifacts(run_dir)
    visited = {
        (int(value[0]), int(value[1]))
        for value in topology.get("visited_cells", [])
    }
    if not visited:
        raise ValueError("saved topology has no visited cells")

    saved_start = tuple(int(v) for v in topology.get("start_cell", [0, 0]))
    saved_final = tuple(int(v) for v in topology.get("final_cell", saved_start))
    custom_cell = [saved_start]
    result = {"path": None}

    root = tk.Tk()
    root.title("Assignment 2 - Round 2 Start Pose and Targets")
    root.geometry("980x720")

    main = ttk.Frame(root, padding=12)
    main.pack(fill="both", expand=True)
    main.columnconfigure(0, weight=3)
    main.columnconfigure(1, weight=2)
    main.rowconfigure(0, weight=1)

    canvas = tk.Canvas(main, background="#111827", highlightthickness=0)
    canvas.grid(row=0, column=0, sticky="nsew", padx=(0, 12))
    controls = ttk.Frame(main)
    controls.grid(row=0, column=1, sticky="nsew")

    source_var = tk.StringVar(value="saved_start")
    facing_var = tk.StringVar(value="FRONT")
    selected_label = tk.StringVar()
    status_var = tk.StringVar(value="Choose the real robot start pose, then save.")
    target_vars: Dict[str, tk.BooleanVar] = {}
    hit_cells: Dict[int, Cell] = {}

    ttk.Label(
        controls,
        text="Round 2 start pose",
        font=("TkDefaultFont", 13, "bold"),
    ).pack(anchor="w", pady=(0, 8))

    ttk.Radiobutton(
        controls,
        text="Round-1 start_cell {}".format(saved_start),
        variable=source_var,
        value="saved_start",
    ).pack(anchor="w", pady=2)
    ttk.Radiobutton(
        controls,
        text="Round-1 final_cell {}".format(saved_final),
        variable=source_var,
        value="saved_final",
    ).pack(anchor="w", pady=2)
    ttk.Radiobutton(
        controls,
        text="Custom cell (click map)",
        variable=source_var,
        value="custom",
    ).pack(anchor="w", pady=2)
    ttk.Label(controls, textvariable=selected_label).pack(anchor="w", pady=(3, 10))

    ttk.Label(controls, text="Robot front points toward map:").pack(anchor="w")
    facing_box = ttk.Combobox(
        controls,
        textvariable=facing_var,
        values=("FRONT", "RIGHT", "BACK", "LEFT"),
        state="readonly",
        width=14,
    )
    facing_box.pack(anchor="w", pady=(3, 12))

    ttk.Separator(controls).pack(fill="x", pady=6)
    ttk.Label(
        controls,
        text="Required targets",
        font=("TkDefaultFont", 12, "bold"),
    ).pack(anchor="w", pady=(4, 6))

    targets_by_spec = {}
    for target in target_payload.get("targets", []):
        if not target.get("round2_position_ready"):
            continue
        key = "{}:{}".format(
            str(target.get("color", "")).lower(),
            str(target.get("shape", "")).lower(),
        )
        targets_by_spec.setdefault(key, []).append(str(target.get("target_id", "?")))
    for spec in sorted(targets_by_spec):
        ids = targets_by_spec[spec]
        variable = tk.BooleanVar(value=False)
        target_vars[spec] = variable
        text = "{}  ({})".format(spec, ", ".join(ids))
        if len(ids) > 1:
            text += " - ambiguous"
        ttk.Checkbutton(controls, text=text, variable=variable).pack(
            anchor="w", pady=2
        )

    if not target_vars:
        ttk.Label(
            controls,
            text="No Round-2-ready target poses in targets.json",
            foreground="#b91c1c",
            wraplength=320,
        ).pack(anchor="w", pady=4)

    ttk.Separator(controls).pack(fill="x", pady=10)
    ttk.Label(
        controls,
        text=(
            "Directions are relative to the Round-1 map: FRONT is up and "
            "RIGHT is right. The saved plan converts every map step into a "
            "body-relative command for the chosen robot facing."
        ),
        wraplength=330,
        foreground="#4b5563",
    ).pack(anchor="w")
    ttk.Label(
        controls,
        textvariable=status_var,
        wraplength=330,
    ).pack(anchor="w", pady=(12, 8))

    def chosen_cell() -> Cell:
        source = source_var.get()
        if source == "saved_final":
            return saved_final
        if source == "custom":
            return custom_cell[0]
        return saved_start

    def draw_map(*_args):
        canvas.delete("all")
        hit_cells.clear()
        width = max(500, canvas.winfo_width())
        height = max(500, canvas.winfo_height())
        min_x = min(cell[0] for cell in visited)
        max_x = max(cell[0] for cell in visited)
        min_y = min(cell[1] for cell in visited)
        max_y = max(cell[1] for cell in visited)
        size = min(
            (width - 90) / max(1, max_y - min_y + 1),
            (height - 90) / max(1, max_x - min_x + 1),
        )
        ox = (width - (max_y - min_y + 1) * size) / 2.0
        oy = (height - (max_x - min_x + 1) * size) / 2.0

        def rect(cell):
            x, y = cell
            col = max_y - y
            row = max_x - x
            x0 = ox + col * size
            y0 = oy + row * size
            return x0, y0, x0 + size, y0 + size

        def center(cell):
            x0, y0, x1, y1 = rect(cell)
            return (x0 + x1) / 2.0, (y0 + y1) / 2.0

        for index, cell in enumerate(sorted(visited)):
            x0, y0, x1, y1 = rect(cell)
            canvas.create_rectangle(
                x0, y0, x1, y1, fill="#f8fafc", outline="#94a3b8"
            )
            canvas.create_text(
                x0 + 5, y0 + 5, text="{},{}".format(*cell),
                anchor="nw", fill="#64748b",
            )
            hit_cells[index] = cell
            canvas.create_rectangle(
                x0, y0, x1, y1, outline="", fill="", tags=("cell_{}".format(index),)
            )
            canvas.tag_bind(
                "cell_{}".format(index),
                "<Button-1>",
                lambda _event, picked=cell: select_custom(picked),
            )

        for x, y, direction in topology.get("wall_edges", []):
            cell = int(x), int(y)
            if cell not in visited:
                continue
            x0, y0, x1, y1 = rect(cell)
            lines = {
                0: (x0, y0, x1, y0),
                1: (x1, y0, x1, y1),
                2: (x0, y1, x1, y1),
                3: (x0, y0, x0, y1),
            }
            canvas.create_line(*lines[int(direction) % 4], fill="#111827", width=5)

        for target in target_payload.get("targets", []):
            pose = target.get("round2_approach_pose") or {}
            raw_cell = pose.get("cell")
            if not target.get("round2_position_ready") or not raw_cell:
                continue
            cell = int(raw_cell[0]), int(raw_cell[1])
            if cell not in visited:
                continue
            cx, cy = center(cell)
            canvas.create_oval(
                cx - 13, cy - 13, cx + 13, cy + 13,
                fill=str(target.get("color", "purple")), outline="white", width=2,
            )
            canvas.create_text(
                cx, cy, text=str(target.get("target_id", "T")), fill="white"
            )

        start = chosen_cell()
        sx, sy = center(start)
        canvas.create_oval(
            sx - 18, sy - 18, sx + 18, sy + 18,
            fill="#7c3aed", outline="white", width=2,
        )
        direction = {name: number for number, name in DIR_NAME.items()}[
            facing_var.get()
        ]
        dx, dy = {
            0: (0, -1), 1: (1, 0), 2: (0, 1), 3: (-1, 0)
        }[direction]
        canvas.create_line(
            sx, sy, sx + dx * size * 0.38, sy + dy * size * 0.38,
            fill="#f59e0b", width=4, arrow="last",
        )
        canvas.create_text(
            ox, max(12, oy - 24), text="MAP FRONT ↑",
            anchor="w", fill="white", font=("TkDefaultFont", 11, "bold"),
        )
        canvas.create_text(
            ox + (max_y - min_y + 1) * size, max(12, oy - 24),
            text="MAP RIGHT →", anchor="e", fill="white",
            font=("TkDefaultFont", 11, "bold"),
        )
        selected_label.set(
            "Selected cell: {} | facing: {}".format(start, facing_var.get())
        )

    def select_custom(cell: Cell):
        custom_cell[0] = cell
        source_var.set("custom")
        draw_map()

    def save_plan():
        required = ",".join(
            spec for spec, variable in sorted(target_vars.items())
            if variable.get()
        )
        facing = {name: number for number, name in DIR_NAME.items()}[
            facing_var.get()
        ]
        try:
            plan = build_round2_plan(
                topology,
                target_payload,
                required,
                start_cell=chosen_cell(),
                start_facing_direction=facing,
            )
            destination = output_path or run_dir / "round2_plan.json"
            save_round2_plan(plan, destination)
        except (OSError, ValueError) as exc:
            messagebox.showerror("Plan rejected", str(exc), parent=root)
            status_var.set("Plan rejected: {}".format(exc))
            return
        result["path"] = Path(destination)
        messagebox.showinfo(
            "Round 2 plan ready",
            "Saved {}\nTargets: {}\nRoute cells: {}".format(
                destination, len(plan["actions"]), len(plan["full_route"])
            ),
            parent=root,
        )
        root.destroy()

    source_var.trace_add("write", draw_map)
    facing_var.trace_add("write", draw_map)
    canvas.bind("<Configure>", draw_map)

    button_row = ttk.Frame(controls)
    button_row.pack(side="bottom", fill="x", pady=(12, 0))
    ttk.Button(button_row, text="Cancel", command=root.destroy).pack(side="right")
    ttk.Button(button_row, text="Save Round 2 plan", command=save_plan).pack(
        side="right", padx=(0, 8)
    )

    root.after_idle(draw_map)
    root.mainloop()
    return result["path"]
