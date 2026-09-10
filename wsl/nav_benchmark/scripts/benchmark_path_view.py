#!/usr/bin/env python3
"""Focused, truthful Path View for benchmark results.

The viewer deliberately avoids inferring Sub-goal and recovery events from
geometry. It displays recorded facts: actual trajectory, canonical /plan(map)
messages, the latest plan for each Nav2 goal, fixed case obstacles, and the
requested final goal.
"""

from __future__ import annotations

import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

import benchmark_path_tools as path_tools


PLAN_COLORS = (
    "tab:blue",
    "tab:purple",
    "tab:green",
    "tab:brown",
    "tab:pink",
    "tab:cyan",
)


def _color(index: int) -> str:
    return PLAN_COLORS[(max(1, int(index)) - 1) % len(PLAN_COLORS)]


def show_path_view(parent, run_dir: Path, cases_root: Path) -> None:
    run_dir = Path(run_dir)
    case_id = run_dir.parent.name
    scenario = case_id.split("_", 1)[0]
    case_path = Path(cases_root) / scenario / f"{case_id}.yaml"
    if not case_path.exists():
        messagebox.showerror(
            "Path View", f"Case YAML not found:\n{case_path}", parent=parent
        )
        return

    try:
        from matplotlib.backends.backend_tkagg import (
            FigureCanvasTkAgg,
            NavigationToolbar2Tk,
        )
        from matplotlib.figure import Figure
        from matplotlib.patches import Circle, Polygon
    except ImportError as exc:
        messagebox.showerror(
            "Path View",
            "Matplotlib is required for Path View.\n"
            "Install it with: sudo apt install python3-matplotlib\n\n"
            f"{exc}",
            parent=parent,
        )
        return

    try:
        case = path_tools.load_case(case_path)
        summary = path_tools.load_summary(run_dir)
        actual = path_tools.load_actual_trajectory(run_dir)
        parsed_plans = path_tools.load_plans(run_dir)
        plans = [
            plan
            for plan in parsed_plans
            if path_tools.is_canonical_global_plan(plan)
        ]
        first_segments = path_tools.first_plan_per_nav_goal(plans)
        segments = path_tools.latest_plan_per_nav_goal(plans)
        plan_counts = path_tools.plan_counts_per_nav_goal(plans)
        goal = path_tools.requested_goal_from_result(run_dir, summary)
        transform = path_tools.derive_world_to_map(
            case, actual, requested_goal=goal
        )
        obstacles = path_tools.obstacle_outlines_in_map(case, transform)
        if goal is None:
            goal = path_tools.case_goal_in_map(case, transform)
    except Exception as exc:
        messagebox.showerror(
            "Path View",
            f"Failed to load path data:\n{exc}",
            parent=parent,
        )
        return

    if not actual and not parsed_plans:
        messagebox.showinfo(
            "Path View",
            "This run contains neither trajectory.csv nor planned_paths.csv.",
            parent=parent,
        )
        return

    window = tk.Toplevel(parent)
    window.title(f"Path View - {case_id} / {run_dir.name}")
    window.geometry("1050x760")
    window.minsize(720, 600)

    controls = ttk.Frame(window, padding=(10, 8))
    controls.pack(fill="x")
    footer = ttk.Frame(window)
    footer.pack(side="bottom", fill="x")
    plot_host = ttk.Frame(window)
    plot_host.pack(fill="both", expand=True)

    show_actual = tk.BooleanVar(value=bool(actual))
    # 첫 경로가 기본값이다. 마지막 경로는 구간 끝자락 조각이라
    # 출발점부터의 경로가 통째로 빠진 것처럼 보인다.
    show_first = tk.BooleanVar(value=bool(first_segments))
    show_segments = tk.BooleanVar(value=False)
    show_all_plans = tk.BooleanVar(value=False)
    show_obstacles = tk.BooleanVar(value=bool(obstacles))
    show_direct = tk.BooleanVar(value=False)

    figure = Figure(figsize=(9.5, 6.5), dpi=100)
    axis = figure.add_subplot(111)
    canvas = FigureCanvasTkAgg(figure, master=plot_host)
    toolbar = NavigationToolbar2Tk(canvas, plot_host, pack_toolbar=False)
    toolbar.update()
    toolbar.pack(side="bottom", fill="x")
    canvas.get_tk_widget().pack(fill="both", expand=True)

    def draw():
        axis.clear()

        if show_obstacles.get():
            used_label = False
            for obstacle in obstacles:
                label = "Obstacle" if not used_label else None
                if obstacle["kind"] == "circle":
                    patch = Circle(
                        obstacle["center"],
                        obstacle["radius"],
                        fill=False,
                        linewidth=1.2,
                        label=label,
                    )
                else:
                    patch = Polygon(
                        obstacle["points"],
                        closed=True,
                        fill=False,
                        linewidth=1.2,
                        label=label,
                    )
                axis.add_patch(patch)
                used_label = True

        if show_all_plans.get():
            for index, plan in enumerate(plans):
                axis.plot(
                    [pose.x for pose in plan.points],
                    [pose.y for pose in plan.points],
                    linestyle=":",
                    linewidth=0.8,
                    alpha=0.25,
                    color=_color(plan.nav_goal_index),
                    label="All recorded /plan" if index == 0 else None,
                )

        if show_first.get():
            for position, plan in enumerate(first_segments, start=1):
                goal_index = max(position, int(plan.nav_goal_index))
                axis.plot(
                    [pose.x for pose in plan.points],
                    [pose.y for pose in plan.points],
                    linestyle="--",
                    linewidth=2.0,
                    color=_color(goal_index),
                    label=f"First plan - Nav goal {goal_index}",
                )

        if show_segments.get():
            for position, plan in enumerate(segments, start=1):
                goal_index = max(position, int(plan.nav_goal_index))
                axis.plot(
                    [pose.x for pose in plan.points],
                    [pose.y for pose in plan.points],
                    linestyle="-.",
                    linewidth=1.2,
                    alpha=0.7,
                    color=_color(goal_index),
                    label=f"Latest plan - Nav goal {goal_index}",
                )

        if show_actual.get() and actual:
            axis.plot(
                [pose.x for pose in actual],
                [pose.y for pose in actual],
                linestyle="-",
                linewidth=2.2,
                color="tab:orange",
                label="Actual trajectory",
            )

        if actual:
            axis.plot(
                [actual[0].x],
                [actual[0].y],
                marker="o",
                linestyle="None",
                markersize=8,
                label="Start",
            )
        if goal is not None:
            axis.plot(
                [goal.x],
                [goal.y],
                marker="*",
                linestyle="None",
                markersize=12,
                label="Goal",
            )
        if show_direct.get() and actual and goal is not None:
            axis.plot(
                [actual[0].x, goal.x],
                [actual[0].y, goal.y],
                linestyle=":",
                linewidth=1.0,
                label="Direct Start-Goal",
            )

        result = summary.get(
            "benchmark_result", summary.get("result", "-")
        )
        axis.set_title(f"{case_id} / {run_dir.name} / {result}")
        axis.set_xlabel("Map X (m)")
        axis.set_ylabel("Map Y (m)")
        axis.set_aspect("equal", adjustable="datalim")
        axis.grid(True, alpha=0.25)
        _handles, labels = axis.get_legend_handles_labels()
        if labels:
            axis.legend(loc="best")
        figure.tight_layout()
        canvas.draw_idle()

    controls_spec = (
        ("Actual trajectory", show_actual, bool(actual)),
        ("First plan per Nav goal", show_first, bool(first_segments)),
        ("Latest plan per Nav goal", show_segments, bool(segments)),
        ("All recorded plans", show_all_plans, bool(plans)),
        ("Obstacles", show_obstacles, bool(obstacles)),
        ("Direct line", show_direct, bool(actual) and goal is not None),
    )
    for column, (text, variable, enabled) in enumerate(controls_spec):
        check = ttk.Checkbutton(
            controls, text=text, variable=variable, command=draw
        )
        check.grid(row=0, column=column, sticky="w", padx=(0, 14))
        if not enabled:
            check.configure(state="disabled")

    excluded_count = len(parsed_plans) - len(plans)
    # Nav goal별 계획 개수는 "왜 이 구간만 경로가 하나뿐인가" 같은 질문에
    # 추측 대신 숫자로 답한다. 재계획이 실제로 몇 번 돌았는지가 보인다.
    per_goal = (
        ", ".join(f"#{key}:{plan_counts[key]}" for key in sorted(plan_counts))
        or "-"
    )
    info = (
        f"Actual samples: {len(actual)}   |   "
        f"Recorded plans: {len(parsed_plans)}   |   "
        f"Valid /plan(map): {len(plans)}   |   "
        f"Plans per Nav goal: {per_goal}   |   "
        f"Excluded mixed-frame plans: {excluded_count}"
    )
    ttk.Label(
        footer,
        text=info,
        padding=(10, 6),
        justify="left",
    ).pack(fill="x")

    if not plans:
        ttk.Label(
            footer,
            text=(
                "No canonical /plan(map) was recorded. The actual trajectory "
                "remains usable; rerun with benchmark_logger suite 2.0."
            ),
            padding=(10, 2, 10, 8),
            justify="left",
        ).pack(fill="x")

    draw()