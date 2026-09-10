# Nav Benchmark QA UI Theme - benchmark suite 2.0
# Visual design remains independent from benchmark execution semantics.

import tkinter as tk
import tkinter.font as tkfont
from tkinter import ttk

# Neutral engineering-tool palette. Layout sizing stays in the GUI; this file
# only owns visual styling and platform-safe typography.
BG = "#F3F5F7"
SURFACE = "#FFFFFF"
TEXT = "#1F2937"
TEXT_MUTED = "#667085"
BORDER = "#D9DEE7"
PRIMARY = "#2563EB"
PRIMARY_HOVER = "#1D4ED8"
DANGER = "#DC2626"
DANGER_HOVER = "#B91C1C"
SELECT = "#E8F0FE"
HEADER = "#E9EDF3"
HEADER_HOVER = "#DFE5EC"
DISABLED = "#A5ADBA"
CONSOLE_BG = "#111827"
CONSOLE_FG = "#E5E7EB"


def apply_theme(root):
    """Apply a compact, platform-safe theme without imposing widget sizes."""
    style = ttk.Style(root)
    try:
        style.theme_use("clam")
    except tk.TclError:
        pass

    # Reuse the platform's installed families instead of assuming Windows-only
    # fonts such as Segoe UI / Cascadia Mono on WSL Ubuntu.
    default_font = tkfont.nametofont("TkDefaultFont")
    text_font = tkfont.nametofont("TkTextFont")
    fixed_font = tkfont.nametofont("TkFixedFont")
    heading_font = tkfont.nametofont("TkHeadingFont")

    default_font.configure(size=10)
    text_font.configure(size=10)
    fixed_font.configure(size=9)
    heading_font.configure(size=10, weight="bold")

    root.configure(bg=BG)

    # Native Tk widgets used by the existing GUI.
    root.option_add("*Text.background", CONSOLE_BG)
    root.option_add("*Text.foreground", CONSOLE_FG)
    root.option_add("*Text.insertBackground", "#FFFFFF")
    root.option_add("*Text.selectBackground", "#374151")
    root.option_add("*Text.selectForeground", "#FFFFFF")
    root.option_add("*Text.relief", "flat")
    root.option_add("*Text.borderWidth", 0)

    root.option_add("*Listbox.background", SURFACE)
    root.option_add("*Listbox.foreground", TEXT)
    root.option_add("*Listbox.selectBackground", SELECT)
    root.option_add("*Listbox.selectForeground", TEXT)
    root.option_add("*Listbox.relief", "flat")
    root.option_add("*Listbox.borderWidth", 1)
    root.option_add("*Listbox.highlightThickness", 1)
    root.option_add("*Listbox.highlightBackground", BORDER)

    root.option_add("*TCombobox*Listbox.background", SURFACE)
    root.option_add("*TCombobox*Listbox.foreground", TEXT)
    root.option_add("*TCombobox*Listbox.selectBackground", PRIMARY)
    root.option_add("*TCombobox*Listbox.selectForeground", "#FFFFFF")

    # Do not let clam impose a large minimum button width. Buttons should size
    # themselves from their text and padding so toolbars remain responsive.
    root.option_add("*TButton.width", 0)

    style.configure(".", background=BG, foreground=TEXT, font="TkDefaultFont")
    style.configure("TFrame", background=BG)
    style.configure("TLabel", background=BG, foreground=TEXT)

    style.configure(
        "TLabelframe",
        background=BG,
        bordercolor=BORDER,
        lightcolor=BORDER,
        darkcolor=BORDER,
        borderwidth=1,
        relief="solid",
    )
    style.configure(
        "TLabelframe.Label",
        background=BG,
        foreground=TEXT,
        font="TkHeadingFont",
        padding=(4, 0),
    )

    style.configure(
        "TButton",
        font="TkDefaultFont",
        padding=(7, 4),
        background=SURFACE,
        foreground=TEXT,
        bordercolor=BORDER,
        lightcolor=BORDER,
        darkcolor=BORDER,
        relief="flat",
    )
    style.map(
        "TButton",
        background=[
            ("disabled", "#ECEFF3"),
            ("pressed", "#E2E7ED"),
            ("active", "#EDF1F5"),
        ],
        foreground=[("disabled", DISABLED)],
        bordercolor=[("focus", "#9AA8BC"), ("active", "#C5CDD8")],
    )

    style.configure(
        "Primary.TButton",
        font="TkHeadingFont",
        padding=(9, 5),
        background=PRIMARY,
        foreground="#FFFFFF",
        bordercolor=PRIMARY,
        lightcolor=PRIMARY,
        darkcolor=PRIMARY,
    )
    style.map(
        "Primary.TButton",
        background=[
            ("disabled", "#AABCE5"),
            ("pressed", PRIMARY_HOVER),
            ("active", PRIMARY_HOVER),
        ],
        foreground=[("disabled", "#EEF2FF")],
        bordercolor=[("disabled", "#AABCE5"), ("active", PRIMARY_HOVER)],
    )

    style.configure(
        "Danger.TButton",
        font="TkHeadingFont",
        padding=(9, 5),
        background=DANGER,
        foreground="#FFFFFF",
        bordercolor=DANGER,
        lightcolor=DANGER,
        darkcolor=DANGER,
    )
    style.map(
        "Danger.TButton",
        background=[
            ("disabled", "#E5E7EB"),
            ("pressed", DANGER_HOVER),
            ("active", DANGER_HOVER),
        ],
        foreground=[("disabled", "#9CA3AF")],
        bordercolor=[("disabled", "#D1D5DB"), ("active", DANGER_HOVER)],
    )

    style.configure(
        "TEntry",
        padding=(7, 5),
        fieldbackground=SURFACE,
        foreground=TEXT,
        bordercolor=BORDER,
        lightcolor=BORDER,
        darkcolor=BORDER,
        insertcolor=TEXT,
    )
    style.map(
        "TEntry",
        bordercolor=[("focus", PRIMARY), ("disabled", "#E0E4EA")],
        fieldbackground=[("disabled", "#F0F2F5")],
    )

    style.configure(
        "TCombobox",
        padding=(7, 5),
        fieldbackground=SURFACE,
        background=SURFACE,
        foreground=TEXT,
        arrowcolor=TEXT_MUTED,
        bordercolor=BORDER,
        lightcolor=BORDER,
        darkcolor=BORDER,
    )
    style.map(
        "TCombobox",
        bordercolor=[("focus", PRIMARY)],
        fieldbackground=[("readonly", SURFACE)],
        selectbackground=[("readonly", SURFACE)],
        selectforeground=[("readonly", TEXT)],
        background=[("readonly", SURFACE), ("active", "#F2F4F7")],
    )

    style.configure(
        "Treeview",
        background=SURFACE,
        fieldbackground=SURFACE,
        foreground=TEXT,
        rowheight=29,
        borderwidth=0,
        relief="flat",
        font="TkDefaultFont",
    )
    style.map(
        "Treeview",
        background=[("selected", SELECT)],
        foreground=[("selected", TEXT)],
    )
    style.configure(
        "Treeview.Heading",
        background=HEADER,
        foreground="#344054",
        font=(default_font.actual("family"), 9, "bold"),
        padding=(2, 6),
        relief="flat",
        bordercolor=BORDER,
        lightcolor=BORDER,
        darkcolor=BORDER,
    )
    style.map("Treeview.Heading", background=[("active", HEADER_HOVER)])

    style.configure(
        "Vertical.TScrollbar",
        background="#C7CED8",
        troughcolor="#EEF1F5",
        bordercolor="#EEF1F5",
        arrowcolor=TEXT_MUTED,
        width=13,
    )
    style.configure(
        "Horizontal.TScrollbar",
        background="#C7CED8",
        troughcolor="#EEF1F5",
        bordercolor="#EEF1F5",
        arrowcolor=TEXT_MUTED,
    )
    style.configure("TPanedwindow", background=BG)
    style.configure("TCheckbutton", background=BG, foreground=TEXT, padding=(2, 3))
    style.map("TCheckbutton", background=[("active", BG)])

    style.configure(
        "Title.TLabel",
        background=BG,
        foreground=TEXT,
        font=(default_font.actual("family"), 18, "bold"),
    )
    style.configure(
        "Muted.TLabel",
        background=BG,
        foreground=TEXT_MUTED,
        font=(default_font.actual("family"), 9),
    )
    style.configure(
        "Status.TLabel",
        background=BG,
        foreground=TEXT,
        font=(default_font.actual("family"), 10, "bold"),
    )
