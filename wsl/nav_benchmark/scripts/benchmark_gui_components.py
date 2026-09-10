#!/usr/bin/env python3
"""Reusable Tk components for the benchmark QA GUI."""

from __future__ import annotations

import tkinter as tk
from tkinter import messagebox, ttk


def scenario_sort_key(scenario: str):
    try:
        return int(str(scenario).upper().removeprefix("S"))
    except ValueError:
        return 9999


def case_sort_key(case_id: str):
    try:
        scenario, case_no = case_id.upper().split("_", 1)
        return scenario_sort_key(scenario), int(case_no)
    except (ValueError, IndexError):
        return 9999, case_id


class CaseSelectionDialog(tk.Toplevel):
    """Modal selector where ordinary clicks toggle multiple cases."""

    def __init__(self, parent, cases_by_scenario, initial_scenario=None):
        super().__init__(parent)
        self.title("Select Benchmark Cases")
        self.geometry("520x610")
        self.minsize(440, 480)
        self.transient(parent)
        self.grab_set()

        self.cases_by_scenario = {
            key: tuple(sorted(values, key=case_sort_key))
            for key, values in cases_by_scenario.items()
        }
        self.selected_cases = set()
        self.result = None
        self.visible_cases = []

        scenario_values = ["All"] + sorted(
            self.cases_by_scenario,
            key=scenario_sort_key,
        )
        self.filter_var = tk.StringVar(
            value=(
                initial_scenario
                if initial_scenario in self.cases_by_scenario
                else "All"
            )
        )
        self.count_var = tk.StringVar(value="0 selected")

        top = ttk.Frame(self, padding=12)
        top.pack(fill="x")
        ttk.Label(top, text="Scenario filter").pack(side="left")
        filter_combo = ttk.Combobox(
            top,
            textvariable=self.filter_var,
            values=scenario_values,
            state="readonly",
            width=10,
        )
        filter_combo.pack(side="left", padx=(8, 0))
        filter_combo.bind(
            "<<ComboboxSelected>>",
            lambda _event: self._change_filter(),
        )

        ttk.Label(
            self,
            text="Click cases to toggle them. Ctrl/Shift is not required.",
            padding=(12, 0, 12, 8),
        ).pack(fill="x")

        body = ttk.Frame(self, padding=(12, 0, 12, 8))
        body.pack(fill="both", expand=True)
        self.listbox = tk.Listbox(
            body,
            selectmode=tk.MULTIPLE,
            exportselection=False,
            font=("TkDefaultFont", 10),
        )
        scroll = ttk.Scrollbar(
            body, orient="vertical", command=self.listbox.yview
        )
        self.listbox.configure(yscrollcommand=scroll.set)
        self.listbox.grid(row=0, column=0, sticky="nsew")
        scroll.grid(row=0, column=1, sticky="ns")
        body.rowconfigure(0, weight=1)
        body.columnconfigure(0, weight=1)
        self.listbox.bind("<<ListboxSelect>>", self._on_select)

        footer = ttk.Frame(self, padding=(12, 4, 12, 12))
        footer.pack(fill="x")
        ttk.Label(footer, textvariable=self.count_var).pack(side="left")
        ttk.Button(
            footer, text="Select Visible", command=self._select_visible
        ).pack(side="left", padx=(16, 4))
        ttk.Button(
            footer, text="Clear", command=self._clear_selection
        ).pack(side="left", padx=4)
        ttk.Button(
            footer, text="Cancel", command=self._cancel
        ).pack(side="right")
        ttk.Button(
            footer, text="Run Selected", command=self._accept
        ).pack(side="right", padx=(0, 8))

        self.protocol("WM_DELETE_WINDOW", self._cancel)
        self._refresh_list()
        self.wait_visibility()
        self.focus_set()

    def _visible_cases(self):
        current = self.filter_var.get()
        if current == "All":
            cases = []
            for scenario in sorted(
                self.cases_by_scenario,
                key=scenario_sort_key,
            ):
                cases.extend(self.cases_by_scenario[scenario])
            return cases
        return list(self.cases_by_scenario.get(current, ()))

    def _capture_visible_selection(self):
        selected_indexes = set(self.listbox.curselection())
        for case_id in self.visible_cases:
            self.selected_cases.discard(case_id)
        for index in selected_indexes:
            if 0 <= index < len(self.visible_cases):
                self.selected_cases.add(self.visible_cases[index])

    def _refresh_list(self):
        self.visible_cases = self._visible_cases()
        self.listbox.delete(0, "end")
        for case_id in self.visible_cases:
            self.listbox.insert("end", case_id)
        for index, case_id in enumerate(self.visible_cases):
            if case_id in self.selected_cases:
                self.listbox.selection_set(index)
        self._update_count()

    def _change_filter(self):
        self._capture_visible_selection()
        self._refresh_list()

    def _on_select(self, _event=None):
        self._capture_visible_selection()
        self._update_count()

    def _select_visible(self):
        self.selected_cases.update(self._visible_cases())
        self._refresh_list()

    def _clear_selection(self):
        self.selected_cases.clear()
        self._refresh_list()

    def _update_count(self):
        self.count_var.set(f"{len(self.selected_cases)} selected")

    def _accept(self):
        self._capture_visible_selection()
        if not self.selected_cases:
            messagebox.showwarning(
                "No cases selected",
                "Select one or more benchmark cases.",
                parent=self,
            )
            return
        self.result = sorted(self.selected_cases, key=case_sort_key)
        self.destroy()

    def _cancel(self):
        self.result = None
        self.destroy()


# Backward-compatible private aliases used by the main GUI.
_scenario_sort_key = scenario_sort_key
_case_sort_key = case_sort_key
