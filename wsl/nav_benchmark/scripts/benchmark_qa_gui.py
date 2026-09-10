#!/usr/bin/env python3
"""
Nav Benchmark QA GUI - benchmark suite 2.0
=========================

Thin GUI wrapper around the already-validated benchmark_runner.py.

Runs on: PC/WSL Ubuntu
Prerequisites:
  - ~/nav_benchmark/scripts/benchmark_runner.py
  - cases under ~/nav_benchmark/cases/S1 ... S6
  - passwordless SSH to Raspberry Pi
  - Pi hil_router already running

The GUI itself does NOT reimplement benchmark logic.
It launches benchmark_runner.py and displays its output/results.
"""

from __future__ import annotations

import ipaddress
import os
import re
import signal
import subprocess
import sys
import threading
import tkinter as tk
from pathlib import Path
from queue import Empty, Queue
from tkinter import filedialog, messagebox, ttk

from ui_theme import apply_theme
from benchmark_common import SUITE_VERSION
from benchmark_gui_components import (
    CaseSelectionDialog,
    _case_sort_key,
    _scenario_sort_key,
)
from benchmark_path_view import show_path_view

try:
    import yaml
except ImportError:
    yaml = None


HOME = Path.home()
ROOT = HOME / "nav_benchmark"
RUNNER = ROOT / "scripts" / "benchmark_runner.py"
CASES_ROOT = ROOT / "cases"
RESULTS_ROOT = ROOT / "results"

POLL_MS = 100

# ROS 2 / Zenoh may emit ANSI color/control sequences even when stdout is
# redirected into the GUI.  They are useful in a terminal, but become raw ESC
# characters when copied from a Tk Text widget.
# 이전 패턴의 결함 두 가지:
#  1) OSC 종료자 ST를 백슬래시 두 개(\x1B\\\\)로 적어 ST로 끝나는 시퀀스를
#     못 잡았고, 대신 뒤 분기가 멀쩡한 글자를 갉아먹었다.
#  2) [^\x07]* 가 탐욕적이라 한 줄에 ESC]가 있고 뒤늦게 BEL이 나오면
#     그 사이 로그 본문이 통째로 삭제됐다.
# 진단 콘솔에서 로그가 조용히 사라지는 것이 최악이므로, 종료자가 확실할
# 때만 지우고 애매하면 원문을 남긴다.
_ANSI_ESCAPE_RE = re.compile(
    r"\x1B\[[0-?]*[ -/]*[@-~]"                      # CSI (색상 등)
    r"|\x1B\][^\x07\x1B]{0,128}(?:\x07|\x1B\\)"   # OSC (종료자 필수)
    r"|\x1B[@-Z\\^_]"                               # 기타 단일문자 이스케이프
)


def _detect_wsl() -> bool:
    if os.environ.get("WSL_DISTRO_NAME"):
        return True
    try:
        with open("/proc/version", "r", encoding="utf-8", errors="replace") as fh:
            return "microsoft" in fh.read().lower()
    except OSError:
        return False


IS_WSL = _detect_wsl()


def _copy_to_windows_clipboard(text: str) -> bool:
    """Write text straight into the Windows clipboard from WSL.

    X11 keeps clipboard data inside the owning process: nothing is handed to
    a clipboard manager, so Tk's clipboard dies the moment this window closes
    and does not reliably cross the WSLg bridge for large text.  clip.exe
    writes into the Windows clipboard itself, which survives both.
    """
    windows_text = text.replace("\r\n", "\n").replace("\n", "\r\n")
    attempts = (
        b"\xff\xfe" + windows_text.encode("utf-16-le", "replace"),
        windows_text.encode("utf-8", "replace"),
    )
    for payload in attempts:
        try:
            completed = subprocess.run(
                ["clip.exe"],
                input=payload,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=5,
            )
        except (OSError, subprocess.SubprocessError):
            return False
        if completed.returncode == 0:
            return True
    return False


def _clean_console_text(text: str) -> str:
    clean = _ANSI_ESCAPE_RE.sub("", text)
    clean = clean.replace("\r\n", "\n").replace("\r", "\n")
    # Drop remaining C0 controls except newline/tab. They have no useful
    # representation in the runner console and make clipboard text confusing.
    clean = "".join(
        ch for ch in clean
        if ch in "\n\t" or ord(ch) >= 32
    )
    return clean

class BenchmarkQAGui(tk.Tk):
    def __init__(self):
        super().__init__()
        apply_theme(self)

        self.title(f"Nav Benchmark QA {SUITE_VERSION}")
        self.geometry("1320x800")
        self.minsize(900, 650)

        self.proc: subprocess.Popen | None = None
        self.output_queue: Queue[str] = Queue()
        self.close_requested = False

        self.scenario_var = tk.StringVar()
        self.case_var = tk.StringVar()
        self.pi_host_var = tk.StringVar(
            value=os.environ.get("NAV_BENCH_PI_HOST", "").strip()
        )
        self.status_var = tk.StringVar(value="IDLE")
        self.command_var = tk.StringVar(value="-")
        self.latest_result_var = tk.StringVar(value="-")
        self.console_status_var = tk.StringVar(
            value="Copy All / Save Console to File 로 러너 출력을 꺼낼 수 있습니다."
        )

        self._build_ui()
        self._load_scenarios()
        self._refresh_results()
        self.after(POLL_MS, self._drain_output_queue)

        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ------------------------------------------------------------------
    # UI
    # ------------------------------------------------------------------
    def _build_ui(self):
        top = ttk.Frame(self, padding=(14, 12, 14, 8))
        top.pack(fill="x")
        top.columnconfigure(0, weight=1)

        ttk.Label(
            top,
            text="Nav Benchmark QA",
            style="Title.TLabel",
        ).grid(row=0, column=0, sticky="w", pady=(0, 10))

        # Selector and action groups reflow instead of forcing a wide window.
        # Wide windows use one toolbar row; narrower windows move the action
        # group below the selectors while preserving the exact same controls.
        controls = ttk.Frame(top)
        controls.grid(row=1, column=0, sticky="ew")
        controls.columnconfigure(0, weight=1)

        selector_bar = ttk.Frame(controls)
        ttk.Label(selector_bar, text="Scenario").grid(row=0, column=0, sticky="w")
        self.scenario_combo = ttk.Combobox(
            selector_bar,
            textvariable=self.scenario_var,
            state="readonly",
            width=9,
        )
        self.scenario_combo.grid(row=0, column=1, padx=(6, 16), sticky="w")
        self.scenario_combo.bind("<<ComboboxSelected>>", self._on_scenario_change)

        ttk.Label(selector_bar, text="Case").grid(row=0, column=2, sticky="w")
        self.case_combo = ttk.Combobox(
            selector_bar,
            textvariable=self.case_var,
            state="readonly",
            width=11,
        )
        self.case_combo.grid(row=0, column=3, padx=(6, 16), sticky="w")

        ttk.Label(selector_bar, text="Pi IP").grid(row=0, column=4, sticky="w")
        self.pi_host_entry = ttk.Entry(
            selector_bar,
            textvariable=self.pi_host_var,
            width=16,
        )
        self.pi_host_entry.grid(row=0, column=5, padx=(6, 0), sticky="w")

        action_bar = ttk.Frame(controls)
        self.run_case_btn = ttk.Button(
            action_bar,
            text="Run Case",
            command=self._run_selected_case,
            style="Primary.TButton",
        )
        self.run_case_btn.pack(side="left", padx=(0, 6))

        self.run_selected_btn = ttk.Button(
            action_bar,
            text="Run Selected...",
            command=self._run_selected_cases,
        )
        self.run_selected_btn.pack(side="left", padx=3)

        self.run_scenario_btn = ttk.Button(
            action_bar,
            text="Run Scenario",
            command=self._run_selected_scenario,
        )
        self.run_scenario_btn.pack(side="left", padx=3)

        self.run_all_btn = ttk.Button(
            action_bar,
            text="Run All",
            command=self._run_all,
        )
        self.run_all_btn.pack(side="left", padx=3)

        self.stop_btn = ttk.Button(
            action_bar,
            text="STOP",
            command=self._stop_runner,
            state="disabled",
            style="Danger.TButton",
        )
        self.stop_btn.pack(side="left", padx=(10, 0))

        toolbar_mode = [None]

        def layout_toolbar(event=None):
            width = event.width if event is not None else controls.winfo_width()
            mode = "single" if width >= 1080 else "stacked"
            if toolbar_mode[0] == mode:
                return
            toolbar_mode[0] = mode

            selector_bar.grid_forget()
            action_bar.grid_forget()
            if mode == "single":
                controls.columnconfigure(0, weight=1)
                controls.columnconfigure(1, weight=0)
                selector_bar.grid(row=0, column=0, sticky="w")
                action_bar.grid(row=0, column=1, sticky="e")
            else:
                controls.columnconfigure(0, weight=1)
                controls.columnconfigure(1, weight=0)
                selector_bar.grid(row=0, column=0, columnspan=2, sticky="w")
                action_bar.grid(
                    row=1,
                    column=0,
                    columnspan=2,
                    sticky="w",
                    pady=(8, 0),
                )

        controls.bind("<Configure>", layout_toolbar)
        layout_toolbar()

        info = ttk.Frame(self, padding=(14, 0, 14, 8))
        info.pack(fill="x")
        # Command / Latest deliberately get flexible cells. width=1 prevents a
        # long value from dictating the whole window's requested width.
        info.columnconfigure(3, weight=3)
        info.columnconfigure(5, weight=2)

        ttk.Label(info, text="Status:").grid(row=0, column=0, sticky="w")
        ttk.Label(
            info,
            textvariable=self.status_var,
            style="Status.TLabel",
        ).grid(row=0, column=1, padx=(6, 22), sticky="w")

        ttk.Label(info, text="Command:").grid(row=0, column=2, sticky="w")
        ttk.Label(
            info,
            textvariable=self.command_var,
            width=1,
            anchor="w",
        ).grid(row=0, column=3, padx=(6, 22), sticky="ew")

        ttk.Label(info, text="Latest:").grid(row=0, column=4, sticky="w")
        ttk.Label(
            info,
            textvariable=self.latest_result_var,
            width=1,
            anchor="w",
        ).grid(row=0, column=5, padx=(6, 0), sticky="ew")

        main_pane = ttk.Panedwindow(self, orient="vertical")
        main_pane.pack(fill="both", expand=True, padx=14, pady=(0, 14))

        # Live console
        console_frame = ttk.Labelframe(
            main_pane,
            text="Live Runner Output",
            padding=8,
        )
        self.console = tk.Text(
            console_frame,
            wrap="none",
            height=22,
            font="TkFixedFont",
        )
        console_y = ttk.Scrollbar(
            console_frame,
            orient="vertical",
            command=self.console.yview,
        )
        console_x = ttk.Scrollbar(
            console_frame,
            orient="horizontal",
            command=self.console.xview,
        )
        self.console.configure(
            yscrollcommand=console_y.set,
            xscrollcommand=console_x.set,
        )

        # Explicit clipboard handling keeps copy behavior deterministic under
        # WSL/X11 and provides a fallback when desktop key bindings differ.
        self.console.bind("<Control-c>", self._copy_console_selection)
        self.console.bind("<Control-C>", self._copy_console_selection)
        self.console.bind("<Button-3>", self._show_console_menu)
        self.console_menu = tk.Menu(self, tearoff=0)
        self.console_menu.add_command(
            label="Copy",
            command=lambda: self._copy_console_selection(),
        )
        self.console_menu.add_command(
            label="Copy All",
            command=self._copy_console_all,
        )
        self.console_menu.add_separator()
        self.console_menu.add_command(
            label="Select All",
            command=self._select_console_all,
        )
        self.console_menu.add_separator()
        self.console_menu.add_command(
            label="Save Console to File...",
            command=self._save_console_to_file,
        )

        self.console.grid(row=0, column=0, sticky="nsew")
        console_y.grid(row=0, column=1, sticky="ns")
        console_x.grid(row=1, column=0, sticky="ew")
        console_frame.rowconfigure(0, weight=1)
        console_frame.columnconfigure(0, weight=1)

        # 클립보드를 아예 거치지 않는 경로를 항상 제공한다. X11 클립보드는
        # 이 창을 닫으면 내용이 사라지고 WSLg 브리지에서 잘리기도 한다.
        console_buttons = ttk.Frame(console_frame)
        console_buttons.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        ttk.Button(
            console_buttons,
            text="Copy All",
            command=self._copy_console_all,
        ).pack(side="left")
        ttk.Button(
            console_buttons,
            text="Save Console to File",
            command=self._save_console_to_file,
        ).pack(side="left", padx=(8, 0))
        ttk.Button(
            console_buttons,
            text="Open Runner Logs",
            command=self._open_runner_logs,
        ).pack(side="left", padx=(8, 0))
        ttk.Label(
            console_buttons,
            textvariable=self.console_status_var,
            style="Muted.TLabel",
        ).pack(side="left", padx=(12, 0))

        # Results table
        results_frame = ttk.Labelframe(
            main_pane,
            text="Latest Case Results",
            padding=8,
        )

        columns = (
            "case",
            "benchmark",
            "nav2",
            "reason",
            "sim_sec",
            "eff",
            "xy",
            "yaw",
            "adaptive",
            "clear",
            "pi_logs",
            "run",
        )
        self._result_columns = columns
        self._result_min_widths = {
            "case": 75,
            "benchmark": 105,
            "nav2": 90,
            "reason": 240,
            "sim_sec": 75,
            "eff": 85,
            "xy": 90,
            "yaw": 105,
            "adaptive": 80,
            "clear": 60,
            "pi_logs": 70,
            "run": 125,
        }
        self._result_flex = {
            "case": 0.6,
            "benchmark": 0.9,
            "nav2": 0.8,
            "reason": 4.0,
            "sim_sec": 0.6,
            "eff": 0.8,
            "xy": 0.8,
            "yaw": 0.9,
            "adaptive": 0.7,
            "clear": 0.5,
            "pi_logs": 0.6,
            "run": 1.3,
        }
        self._result_resize_job = None

        self.results = ttk.Treeview(
            results_frame,
            columns=columns,
            show="headings",
            height=10,
        )

        headings = {
            "case": "Case",
            "benchmark": "Benchmark",
            "nav2": "Nav2",
            "reason": "Failure / Result Reason",
            "sim_sec": "Sim(s)",
            "eff": "Efficiency",
            "xy": "XY Error(m)",
            "yaw": "Yaw Error(deg)",
            "adaptive": "Adaptive",
            "clear": "Clear",
            "pi_logs": "Pi Logs",
            "run": "Run",
        }

        for col in columns:
            self.results.heading(col, text=headings[col])
            self.results.column(
                col,
                width=self._result_min_widths[col],
                minwidth=self._result_min_widths[col],
                anchor="center",
                stretch=False,
            )
        self.results.column("reason", anchor="w")

        results_y = ttk.Scrollbar(
            results_frame,
            orient="vertical",
            command=self.results.yview,
        )
        results_x = ttk.Scrollbar(
            results_frame,
            orient="horizontal",
            command=self.results.xview,
        )
        self.results.configure(
            yscrollcommand=results_y.set,
            xscrollcommand=results_x.set,
        )
        self.results.bind("<<TreeviewSelect>>", self._on_result_select)
        self.results.bind("<Double-1>", self._show_selected_details)

        self.results.grid(row=0, column=0, sticky="nsew")
        results_y.grid(row=0, column=1, sticky="ns")
        results_x.grid(row=1, column=0, sticky="ew")
        results_frame.rowconfigure(0, weight=1)
        results_frame.columnconfigure(0, weight=1)
        results_frame.bind("<Configure>", self._on_results_frame_resize)

        buttons = ttk.Frame(results_frame)
        buttons.grid(row=2, column=0, sticky="w", pady=(8, 0))

        ttk.Button(
            buttons,
            text="Refresh Results",
            command=self._refresh_results,
        ).pack(side="left")

        ttk.Button(
            buttons,
            text="Open Result Folder",
            command=self._open_selected_result_folder,
        ).pack(side="left", padx=(8, 0))

        ttk.Button(
            buttons,
            text="Open Pi Logs",
            command=self._open_selected_pi_logs,
        ).pack(side="left", padx=(8, 0))

        ttk.Button(
            buttons,
            text="Show Details",
            command=self._show_selected_details,
        ).pack(side="left", padx=(8, 0))

        ttk.Button(
            buttons,
            text="Path View",
            command=self._show_selected_path_view,
        ).pack(side="left", padx=(8, 0))

        self.result_detail_var = tk.StringVar(
            value="Select a result row to see details."
        )
        ttk.Label(
            results_frame,
            textvariable=self.result_detail_var,
            justify="left",
            width=1,
            anchor="w",
            style="Muted.TLabel",
        ).grid(row=3, column=0, sticky="ew", pady=(8, 0))

        main_pane.add(console_frame, weight=3)
        main_pane.add(results_frame, weight=2)

    def _on_results_frame_resize(self, event):
        """Debounce result-column layout while the window is being resized."""
        width = max(1, int(event.width))
        if self._result_resize_job is not None:
            try:
                self.after_cancel(self._result_resize_job)
            except tk.TclError:
                pass
        self._result_resize_job = self.after_idle(
            lambda w=width: self._resize_result_columns(w)
        )

    def _resize_result_columns(self, frame_width: int):
        """Fill available width, then fall back to horizontal scrolling."""
        self._result_resize_job = None
        if not hasattr(self, "results"):
            return

        # Leave room for frame padding and the vertical scrollbar.
        available = max(1, frame_width - 28)
        minimum_total = sum(self._result_min_widths.values())

        if available <= minimum_total:
            for col in self._result_columns:
                self.results.column(col, width=self._result_min_widths[col])
            return

        extra = available - minimum_total
        flex_total = sum(self._result_flex.values())
        assigned = 0
        for index, col in enumerate(self._result_columns):
            if index == len(self._result_columns) - 1:
                width = available - assigned
            else:
                width = self._result_min_widths[col] + round(
                    extra * self._result_flex[col] / flex_total
                )
                assigned += width
            self.results.column(col, width=max(self._result_min_widths[col], width))


    # ------------------------------------------------------------------
    # Case discovery
    # ------------------------------------------------------------------
    def _load_scenarios(self):
        scenarios = sorted(
            p.name
            for p in CASES_ROOT.glob("S*")
            if p.is_dir()
        )

        self.scenario_combo["values"] = scenarios

        if scenarios:
            self.scenario_var.set(scenarios[0])
            self._load_cases(scenarios[0])

    def _load_cases(self, scenario: str):
        folder = CASES_ROOT / scenario
        cases = sorted(p.stem for p in folder.glob(f"{scenario}_*.yaml"))
        self.case_combo["values"] = cases

        if cases:
            self.case_var.set(cases[0])
        else:
            self.case_var.set("")

    def _on_scenario_change(self, _event=None):
        self._load_cases(self.scenario_var.get())

    # ------------------------------------------------------------------
    # Runner process
    # ------------------------------------------------------------------
    def _ensure_ready_to_run(self) -> bool:
        if self.proc is not None and self.proc.poll() is None:
            messagebox.showwarning(
                "Benchmark running",
                "A benchmark is already running.",
            )
            return False

        if not RUNNER.exists():
            messagebox.showerror(
                "Runner missing",
                f"Cannot find:\n{RUNNER}",
            )
            return False

        pi_host = self.pi_host_var.get().strip()
        if not pi_host:
            messagebox.showwarning(
                "Pi IP required",
                "Enter the Raspberry Pi's current IPv4 address.",
            )
            self.pi_host_entry.focus_set()
            return False

        try:
            parsed = ipaddress.ip_address(pi_host)
        except ValueError:
            parsed = None

        if parsed is None or parsed.version != 4:
            messagebox.showwarning(
                "Invalid Pi IP",
                f"'{pi_host}' is not a valid IPv4 address.",
            )
            self.pi_host_entry.focus_set()
            return False

        return True

    def _run_selected_case(self):
        if not self._ensure_ready_to_run():
            return

        case_id = self.case_var.get().strip()
        if not case_id:
            messagebox.showwarning("No case", "Select a case first.")
            return

        self._start_runner([case_id])

    def _cases_by_scenario(self):
        result = {}
        for scenario_dir in sorted(CASES_ROOT.glob("S*")):
            if not scenario_dir.is_dir():
                continue
            cases = sorted(
                (p.stem for p in scenario_dir.glob(f"{scenario_dir.name}_*.yaml")),
                key=_case_sort_key,
            )
            if cases:
                result[scenario_dir.name] = cases
        return result

    def _run_selected_cases(self):
        if not self._ensure_ready_to_run():
            return

        dialog = CaseSelectionDialog(
            self,
            self._cases_by_scenario(),
            initial_scenario=self.scenario_var.get().strip(),
        )
        self.wait_window(dialog)

        if not dialog.result:
            return

        self._start_runner(["--cases", *dialog.result])

    def _run_selected_scenario(self):
        if not self._ensure_ready_to_run():
            return

        scenario = self.scenario_var.get().strip()
        if not scenario:
            messagebox.showwarning("No scenario", "Select a scenario first.")
            return

        if not messagebox.askyesno(
            "Run scenario",
            f"Run every case in {scenario}?",
        ):
            return

        self._start_runner(["--scenario", scenario])

    def _run_all(self):
        if not self._ensure_ready_to_run():
            return

        if not messagebox.askyesno(
            "Run all benchmarks",
            "Run every benchmark case?",
        ):
            return

        self._start_runner(["--all"])

    def _start_runner(self, runner_args: list[str]):
        command = [sys.executable, "-u", str(RUNNER), *runner_args]
        pi_host = self.pi_host_var.get().strip()
        runner_env = os.environ.copy()
        runner_env["NAV_BENCH_PI_HOST"] = pi_host
        # A stale endpoint would override NAV_BENCH_PI_HOST inside the runner.
        # The GUI owns the current Pi address, so remove that ambiguity.
        runner_env.pop("NAV_BENCH_ZENOH_ENDPOINT", None)

        self.console.delete("1.0", "end")
        self._append_console(f"[GUI] Pi IP: {pi_host}\n")
        self._append_console("$ " + " ".join(command) + "\n\n")

        try:
            self.proc = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
                env=runner_env,
            )
        except OSError as e:
            messagebox.showerror("Start failed", str(e))
            self.proc = None
            return

        self.status_var.set("RUNNING")
        self.command_var.set(
            f"Pi={pi_host}  " + " ".join(runner_args)
        )
        self._set_run_buttons(False)
        self.stop_btn.configure(state="normal")

        thread = threading.Thread(
            target=self._reader_thread,
            daemon=True,
        )
        thread.start()

    def _reader_thread(self):
        assert self.proc is not None
        proc = self.proc

        if proc.stdout is not None:
            for line in proc.stdout:
                self.output_queue.put(line)

        rc = proc.wait()
        self.output_queue.put(f"\n[GUI] benchmark_runner exited rc={rc}\n")
        self.output_queue.put("__PROCESS_FINISHED__")

    def _drain_output_queue(self):
        try:
            while True:
                item = self.output_queue.get_nowait()

                if item == "__PROCESS_FINISHED__":
                    rc = None if self.proc is None else self.proc.poll()
                    self.status_var.set(
                        "DONE" if rc == 0 else f"EXIT {rc}"
                    )
                    self._set_run_buttons(True)
                    self.stop_btn.configure(state="disabled")
                    self._refresh_results()
                    self.proc = None
                    if self.close_requested:
                        self.destroy()
                        return
                    continue

                self._append_console(item)

        except Empty:
            pass

        if self.winfo_exists():
            self.after(POLL_MS, self._drain_output_queue)

    def _append_console(self, text: str):
        clean = _clean_console_text(text)
        if not clean:
            return

        # 매 줄마다 see("end")를 부르면 실행 중에는 드래그 선택이
        # 계속 끌려 내려가 사실상 복사가 불가능하다. 사용자가 선택 중이거나
        # 위로 스크롤해 둔 상태에서는 따라가지 않는다.
        try:
            has_selection = bool(self.console.tag_ranges("sel"))
            at_bottom = self.console.yview()[1] >= 0.999
        except tk.TclError:
            has_selection, at_bottom = False, True

        self.console.insert("end", clean)
        if at_bottom and not has_selection:
            self.console.see("end")

    def _set_clipboard(self, text: str, label: str) -> bool:
        """Copy text and say plainly where it ended up."""
        if not text.strip():
            self.console_status_var.set(f"{label}: 복사할 내용이 없습니다.")
            return False

        # WSL: push to the Windows clipboard FIRST and, if that worked, do
        # not take the X11 CLIPBOARD selection at all.  X11 keeps the data in
        # the owning process, so Tk has to serve it when another app pastes -
        # and a crash while serving takes this GUI (and the terminal that
        # launched it) down with it.  Owning nothing means serving nothing.
        windows_ok = _copy_to_windows_clipboard(text) if IS_WSL else False

        tk_ok = False
        try:
            if windows_ok:
                raise tk.TclError("windows clipboard already holds the text")
            self.clipboard_clear()
            self.clipboard_append(text)
            # 여기서 update()를 부르면 안 된다. 이벤트 핸들러 안에서
            # update()를 호출하면 Tk 이벤트 루프에 재진입하게 되고,
            # 키를 누르고 있거나 러너 출력이 쏟아지는 중이면 재귀 호출로
            # GUI가 죽는다. 선택 전달은 이미 돌고 있는 mainloop가 처리한다.
            self.update_idletasks()
            tk_ok = True
        except tk.TclError:
            pass

        if windows_ok:
            where = "Windows 클립보드"
        elif tk_ok and not IS_WSL:
            where = "클립보드"
        elif tk_ok:
            where = "X11 클립보드만 (이 창을 닫으면 사라집니다)"
        else:
            where = "실패 - Save Console to File 을 쓰세요"

        self.console_status_var.set(f"{label}: {len(text):,}자 -> {where}")
        return windows_ok or tk_ok

    def _copy_console_selection(self, _event=None):
        try:
            selected = self.console.get("sel.first", "sel.last")
        except tk.TclError:
            self.console_status_var.set(
                "선택된 텍스트가 없습니다. 드래그 후 Ctrl+C, 또는 Copy All."
            )
            return "break"
        try:
            self._set_clipboard(selected, "선택 복사")
        except Exception as exc:  # 복사가 GUI를 죽이는 일은 없어야 한다
            self.console_status_var.set(f"복사 실패: {exc}")
        return "break"

    def _copy_console_all(self):
        try:
            self._set_clipboard(self.console.get("1.0", "end-1c"), "전체 복사")
        except Exception as exc:
            self.console_status_var.set(f"복사 실패: {exc}")

    def _save_console_to_file(self):
        text = self.console.get("1.0", "end-1c")
        if not text.strip():
            messagebox.showinfo("Save console", "콘솔이 비어 있습니다.")
            return

        path = filedialog.asksaveasfilename(
            parent=self,
            title="Save runner console output",
            defaultextension=".log",
            initialfile="runner_console.log",
            filetypes=[("Log files", "*.log"), ("All files", "*.*")],
        )
        if not path:
            return

        try:
            Path(path).write_text(text, encoding="utf-8")
        except OSError as exc:
            messagebox.showerror("Save console", f"파일을 쓸 수 없습니다:\n{exc}")
            return

        self.console_status_var.set(f"저장됨: {path}")
        messagebox.showinfo("Save console", f"저장했습니다:\n{path}")

    def _open_runner_logs(self):
        """Open the newest runner_logs batch folder: every process log lives there."""
        root = ROOT / "runner_logs"
        if not root.exists():
            messagebox.showinfo("Runner logs", f"러너 로그가 아직 없습니다:\n{root}")
            return

        batches = sorted(
            (p for p in root.glob("*") if p.is_dir()),
            key=lambda p: p.stat().st_mtime,
            reverse=True,
        )
        self._open_path(batches[0] if batches else root)

    def _select_console_all(self):
        self.console.tag_add("sel", "1.0", "end-1c")
        self.console.mark_set("insert", "1.0")
        self.console.see("1.0")
        return "break"

    def _show_console_menu(self, event):
        try:
            self.console_menu.tk_popup(event.x_root, event.y_root)
        finally:
            self.console_menu.grab_release()
        return "break"

    def _stop_runner(self):
        if self.proc is None or self.proc.poll() is not None:
            return

        if not messagebox.askyesno(
            "Stop benchmark",
            "Stop the current benchmark?\n\n"
            "The runner will perform its normal cleanup.",
        ):
            return

        self.status_var.set("STOPPING")
        self._append_console("\n[GUI] Sending SIGINT to benchmark runner...\n")

        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
        except ProcessLookupError:
            pass

    def _set_run_buttons(self, enabled: bool):
        state = "normal" if enabled else "disabled"
        self.run_case_btn.configure(state=state)
        self.run_selected_btn.configure(state=state)
        self.run_scenario_btn.configure(state=state)
        self.run_all_btn.configure(state=state)
        self.pi_host_entry.configure(state=state)

    # ------------------------------------------------------------------
    # Results
    # ------------------------------------------------------------------
    @staticmethod
    def _read_yaml_file(path: Path) -> dict | None:
        if yaml is None or not path.exists():
            return None
        try:
            with path.open("r", encoding="utf-8") as handle:
                value = yaml.safe_load(handle) or {}
            return value if isinstance(value, dict) else None
        except (OSError, yaml.YAMLError):
            return None

    def _latest_result_payload(self, case_dir: Path):
        """Return the newest completed or diagnosed run for one case.

        A newly-created run without summary.yaml is not allowed to hide the
        previous result.  A runner_status.yaml INFRA_ERROR is a real result row.
        """
        runs = sorted(
            (path for path in case_dir.glob("run_*") if path.is_dir()),
            key=lambda path: path.stat().st_mtime,
            reverse=True,
        )
        for run_dir in runs:
            summary = self._read_yaml_file(run_dir / "summary.yaml")
            if summary is not None:
                return run_dir, summary
            status = self._read_yaml_file(run_dir / "runner_status.yaml")
            if status is not None:
                payload = dict(status)
                payload.setdefault(
                    "benchmark_result",
                    payload.get("runner_result", "INFRA_ERROR"),
                )
                payload.setdefault("nav2_result", "-")
                payload.setdefault(
                    "benchmark_result_reason",
                    payload.get("failure_reason", "Runner did not produce summary.yaml"),
                )
                payload.setdefault("run_id", run_dir.name)
                return run_dir, payload
        return None, None

    def _refresh_results(self):
        for item in self.results.get_children():
            self.results.delete(item)

        rows = []

        for scenario_dir in sorted(RESULTS_ROOT.glob("S*")):
            if not scenario_dir.is_dir():
                continue

            for case_dir in sorted(scenario_dir.glob("S*_*")):
                if not case_dir.is_dir():
                    continue

                run_dir, s = self._latest_result_payload(case_dir)
                if run_dir is None or s is None:
                    continue

                benchmark_result = s.get("benchmark_result", "-")
                reason = s.get("benchmark_result_reason", "-")

                # Path efficiency is meaningful only when the run actually
                # reaches (or practically reaches) the requested destination.
                if benchmark_result in ("PASS", "NEAR_SUCCESS"):
                    efficiency = self._pct(
                        s.get("path_efficiency_direct_over_travel")
                    )
                else:
                    efficiency = "N/A"

                pi_log_dir = run_dir / "process_logs" / "pi"
                pi_logs_ok = (
                    pi_log_dir.exists()
                    and any(pi_log_dir.glob("*.log"))
                )

                rows.append(
                    (
                        case_dir.name,
                        benchmark_result,
                        s.get("nav2_result", s.get("result", "-")),
                        reason,
                        self._fmt(s.get("duration_sim_sec")),
                        efficiency,
                        self._fmt(s.get("final_xy_error_m"), 4),
                        self._fmt(s.get("final_yaw_error_deg"), 3),
                        s.get("adaptive_escape_count", "-"),
                        s.get("costmap_clear_count", "-"),
                        "YES" if pi_logs_ok else "NO",
                        run_dir.name,
                        run_dir,
                        s,
                    )
                )

        rows.sort(key=lambda row: row[12].stat().st_mtime, reverse=True)

        for row in rows:
            values = row[:12]
            iid = self.results.insert("", "end", values=values)
            self.results.item(iid, tags=(str(row[12]),))

        if rows:
            latest = rows[0]
            self.latest_result_var.set(
                f"{latest[0]} / {latest[1]} / {latest[11]}"
            )
        else:
            self.latest_result_var.set("No results")

    @staticmethod
    def _fmt(value, digits=3):
        if value is None:
            return "-"
        if isinstance(value, (float, int)):
            return f"{value:.{digits}f}"
        return str(value)

    @staticmethod
    def _pct(value):
        if value is None:
            return "-"
        if isinstance(value, (float, int)):
            return f"{value * 100.0:.2f}%"
        return str(value)

    def _selected_result_dir(self) -> Path | None:
        selected = self.results.selection()
        if selected:
            item = selected[0]
            values = self.results.item(item, "values")
            if values and len(values) >= 12:
                case_id = values[0]
                run_id = values[11]
                scenario = case_id.split("_", 1)[0]
                return RESULTS_ROOT / scenario / case_id / run_id

        # No row selected: use newest completed or diagnosed run.
        candidates = []
        for pattern in (
            "S*/S*_*/run_*/summary.yaml",
            "S*/S*_*/run_*/runner_status.yaml",
        ):
            for result_file in RESULTS_ROOT.glob(pattern):
                try:
                    candidates.append(
                        (result_file.stat().st_mtime, result_file.parent)
                    )
                except OSError:
                    pass
        if candidates:
            candidates.sort(reverse=True)
            return candidates[0][1]
        return None

    def _load_selected_summary(self):
        path = self._selected_result_dir()
        if path is None:
            return None, None

        summary = self._read_yaml_file(path / "summary.yaml")
        if summary is not None:
            return path, summary
        return path, self._read_yaml_file(path / "runner_status.yaml")

    def _on_result_select(self, _event=None):
        path, s = self._load_selected_summary()
        if path is None or s is None:
            self.result_detail_var.set(
                "Select a result row to see details."
            )
            return

        result = s.get("benchmark_result", "-")
        reason = s.get("benchmark_result_reason", "-")
        self.result_detail_var.set(
            f"{s.get('case_id', path.parent.name)} | "
            f"{result} | {reason}"
        )

    def _show_selected_details(self, _event=None):
        path, s = self._load_selected_summary()
        if path is None or s is None:
            messagebox.showinfo(
                "Result details",
                "No readable summary.yaml found.",
            )
            return

        pi_dir = path / "process_logs" / "pi"
        pi_logs = "YES" if pi_dir.exists() and any(pi_dir.glob("*.log")) else "NO"

        lines = [
            f"Case: {s.get('case_id', '-')}",
            f"Run: {s.get('run_id', path.name)}",
            f"Benchmark: {s.get('benchmark_result', '-')}",
            f"Nav2: {s.get('nav2_result', s.get('result', '-'))}",
            f"Reason: {s.get('benchmark_result_reason', '-')}",
            "",
            f"Sim time: {s.get('duration_sim_sec', '-')} s",
            f"Travel distance: {s.get('travel_distance_odom_m', '-')} m",
            f"Final XY error: {s.get('final_xy_error_m', '-')} m",
            f"Final Yaw error: {s.get('final_yaw_error_deg', '-')} deg",
            f"Min LiDAR range: {s.get('min_lidar_range_m', '-')} m",
            "",
            f"NavigateToPose goals: {s.get('navigate_to_pose_goal_count', '-')}",
            f"Adaptive Escape: {s.get('adaptive_escape_count', '-')}",
            f"Spin / Backup / Wait: "
            f"{s.get('spin_count', '-')} / "
            f"{s.get('backup_count', '-')} / "
            f"{s.get('wait_count', '-')}",
            f"Costmap clears: {s.get('costmap_clear_count', '-')}",
            f"Pi logs captured: {pi_logs}",
            "",
            f"Result directory:",
            str(path),
        ]

        messagebox.showinfo(
            "Benchmark Result Details",
            "\n".join(lines),
        )

    def _show_selected_path_view(self):
        run_dir = self._selected_result_dir()
        if run_dir is None or not run_dir.exists():
            messagebox.showinfo("Path View", "No benchmark result selected.")
            return
        show_path_view(self, run_dir, CASES_ROOT)


    def _open_selected_pi_logs(self):
        path = self._selected_result_dir()
        if path is None:
            messagebox.showinfo("Pi logs", "No result selected.")
            return

        pi_path = path / "process_logs" / "pi"
        if not pi_path.exists():
            messagebox.showinfo(
                "Pi logs",
                "No Pi logs were captured for this run.\n"
                "Runs made with benchmark_runner v1.5+ should contain them.",
            )
            return

        self._open_path(pi_path)

    @staticmethod
    def _open_path(path: Path):
        try:
            subprocess.Popen(
                ["explorer.exe", str(path)],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            return
        except OSError:
            pass

        try:
            subprocess.Popen(
                ["xdg-open", str(path)],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except OSError:
            messagebox.showinfo(
                "Folder",
                str(path),
            )

    def _open_selected_result_folder(self):
        path = self._selected_result_dir()
        if path is None or not path.exists():
            messagebox.showinfo("Result folder", "No result folder found.")
            return

        self._open_path(path)

    # ------------------------------------------------------------------
    # Close
    # ------------------------------------------------------------------
    def _on_close(self):
        if self.close_requested:
            return
        if self.proc is not None and self.proc.poll() is None:
            if not messagebox.askyesno(
                "Benchmark running",
                "A benchmark is running.\n"
                "Stop it and close the QA program?",
            ):
                return

            self.close_requested = True
            self.status_var.set("STOPPING / CLEANUP")
            self._set_run_buttons(False)
            self.stop_btn.configure(state="disabled")
            self._append_console(
                "\n[GUI] Waiting for benchmark cleanup before closing...\n"
            )
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            except ProcessLookupError:
                pass
            return

        self.destroy()


def main():
    if yaml is None:
        print(
            "WARNING: PyYAML is not installed. "
            "The runner can still launch, but the result table cannot load."
        )

    app = BenchmarkQAGui()
    app.mainloop()


if __name__ == "__main__":
    main()