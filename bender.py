import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import numpy as np
import re
import csv
import sys
import os
import json
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

CONFIG_FILE = "tube_bender_config.json"

DEFAULT_CONFIG = {
    "theme": "Auto (OS Theme)",
    "units": "mm",
    "clr": 19.05,
    "od": 25.40,
    "wt": 2.00,
    "show_grid": True
}

def load_config():
    """Loads application configuration from a JSON file.
    
    Returns:
        dict: Merged user configuration with default fallbacks.
    """
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)
                return {**DEFAULT_CONFIG, **data}
        except Exception:
            return DEFAULT_CONFIG.copy()
    return DEFAULT_CONFIG.copy()

def save_config(config_dict):
    """Serializes the configuration dictionary to the JSON config file."""
    try:
        with open(CONFIG_FILE, "w", encoding="utf-8") as f:
            json.dump(config_dict, f, indent=4)
    except Exception as e:
        print(f"Failed to save configuration: {e}")

def is_os_dark_mode():
    """Detects whether the host OS is configured in Dark Mode.
    
    Checks the Windows Registry on Windows systems and system defaults
    via `defaults read` on macOS systems.
    
    Returns:
        bool: True if system is in dark mode, False otherwise.
    """
    if sys.platform.startswith("win"):
        try:
            import winreg
            key = winreg.OpenKey(
                winreg.HKEY_CURRENT_USER,
                r"Software\Microsoft\Windows\CurrentVersion\Themes\Personalize"
            )
            val, _ = winreg.QueryValueEx(key, "AppsUseLightTheme")
            winreg.CloseKey(key)
            return val == 0
        except Exception:
            return False
    elif sys.platform == "darwin":
        try:
            import subprocess
            res = subprocess.run(
                ["defaults", "read", "-g", "AppleInterfaceStyle"],
                capture_output=True, text=True
            )
            return "Dark" in res.stdout
        except Exception:
            return False
    return False

# Color Palettes with Functional Input Group Coding
PALETTES = {
    "Auto (OS Theme)": None,
    "Dark Modern": {
        "bg": "#1e1e1e", "fg": "#f0f0f0", "frame_bg": "#252526",
        "entry_bg": "#2d2d30", "entry_fg": "#ffffff",
        "tree_bg": "#1e1e1e", "tree_fg": "#cccccc", "tree_sel": "#094771",
        "btn_bg": "#3e3e42", "btn_fg": "#ffffff",
        "group_tooling": "#263545", "group_pt": "#382d24", "group_status": "#1b2d24",
        "plot_bg": "#181a1f", "plot_fg": "#e0e0e0", "plot_grid": "#2d3139",
        "tube_color": "#0099ff", "tube_edge": "#00558f",
        "cl_color": "#ffaa00", "pt_color": "#ff3344"
    },
    "Light Clean": {
        "bg": "#f3f3f3", "fg": "#1a1a1a", "frame_bg": "#ffffff",
        "entry_bg": "#ffffff", "entry_fg": "#1a1a1a",
        "tree_bg": "#ffffff", "tree_fg": "#1a1a1a", "tree_sel": "#cce8ff",
        "btn_bg": "#e1e1e1", "btn_fg": "#1a1a1a",
        "group_tooling": "#e6f0fa", "group_pt": "#faefe6", "group_status": "#eaf5ea",
        "plot_bg": "#ffffff", "plot_fg": "#222222", "plot_grid": "#e4e4e4",
        "tube_color": "#2a80d2", "tube_edge": "#184f85",
        "cl_color": "#d97706", "pt_color": "#dc2626"
    },
    "Industrial Machinery": {
        "bg": "#22252a", "fg": "#e6edf3", "frame_bg": "#2c313a",
        "entry_bg": "#1b1d23", "entry_fg": "#f5f5f5",
        "tree_bg": "#1b1d23", "tree_fg": "#dcdcdc", "tree_sel": "#9a3412",
        "btn_bg": "#3e4451", "btn_fg": "#ffffff",
        "group_tooling": "#1f3b4d", "group_pt": "#4d341f", "group_status": "#1f4333",
        "plot_bg": "#1e2227", "plot_fg": "#e6edf3", "plot_grid": "#2d323a",
        "tube_color": "#e67e22", "tube_edge": "#873600",
        "cl_color": "#38bdf8", "pt_color": "#f43f5e"
    },
    "High-Contrast Blueprint": {
        "bg": "#0a192f", "fg": "#e6f1ff", "frame_bg": "#112240",
        "entry_bg": "#020c1b", "entry_fg": "#64ffda",
        "tree_bg": "#0d1b2a", "tree_fg": "#e0e1dd", "tree_sel": "#1d3557",
        "btn_bg": "#1e3a8a", "btn_fg": "#ffffff",
        "group_tooling": "#13315c", "group_pt": "#3d2b1f", "group_status": "#134e4a",
        "plot_bg": "#05101e", "plot_fg": "#64ffda", "plot_grid": "#112644",
        "tube_color": "#00ffff", "tube_edge": "#007799",
        "cl_color": "#ff007f", "pt_color": "#ffe600"
    }
}

UNIT_FACTORS_TO_MM = {
    "mm": 1.0,
    "cm": 10.0,
    "inch": 25.4
}

def compute_lra_data(pts, b_radius):
    """Converts 3D Cartesian intersection points (XYZ) into CNC bending data (LRA / YBC).
    
    ALGORITHM OVERVIEW (LRA / YBC Decomposition):
    1. Segment Vectors: Evaluates displacement vector and length between consecutive points.
    2. Bend Angles (A / C): Determined by the exterior turning angle between adjacent vectors:
       theta = arccos((v_curr . v_next) / (|v_curr| * |v_next|)).
    3. Tangent Offsets (Pullbacks): Since the bend is an arc with radius R (Centerline Radius, CLR),
       the straight section must end before the theoretical sharp intersection vertex.
       Offset distance = R * tan(theta / 2).
    4. Bend Plane Rotations (R / B): The orientation of the bend plane is given by the normal
       vector n = (v_curr x v_next). The dihedral twist between bend planes is:
       twist = arccos(n_prev . n_curr). The sign/direction is verified using the scalar triple
       product with the connecting tube vector.
    5. Feed Lengths (L / Y): Straight distance between tangents of consecutive bends:
       feed[i] = |v[i]| - offset[i-1] - offset[i].
    """
    rows = pts.shape[0]
    if rows < 3:
        raise ValueError("At least 3 points are required to calculate bend geometry.")

    vector = np.zeros((rows, 3))
    v_mag = np.zeros(rows)
    norms = np.zeros((rows, 3))
    bends = np.zeros(rows)
    bend_offset = np.zeros(rows + 1)
    rot = np.zeros(rows)
    feed = np.zeros(rows)

    # 1. Calculate direction vectors and magnitudes between sequential vertices
    for i in range(1, rows):
        vector[i] = pts[i] - pts[i - 1]
        v_mag[i] = np.linalg.norm(vector[i])
        if v_mag[i] == 0:
            raise ValueError(f"Duplicate consecutive point detected at row {i+1}.")

    # 2. Compute bend angles, tangent offsets, and plane normal vectors
    for i in range(1, rows - 1):
        v_curr = vector[i]
        v_next = vector[i + 1]

        # Exterior angle between vector segments
        cos_theta = np.dot(v_curr, v_next) / (v_mag[i] * v_mag[i + 1])
        cos_theta = np.clip(cos_theta, -1.0, 1.0)
        bend_angle_rad = np.arccos(cos_theta)
        bends[i] = np.rad2deg(bend_angle_rad)

        # Tangent distance (pullback) from vertex to arc start/end points
        half_angle = bend_angle_rad / 2.0
        bend_offset[i] = b_radius * np.tan(half_angle)

        # Normal vector perpendicular to the bend plane spanned by v_curr and v_next
        cross_prod = np.cross(v_curr, v_next)
        norm_mag = np.linalg.norm(cross_prod)
        if norm_mag > 1e-7:
            norms[i] = cross_prod / norm_mag
        else:
            # Collinear segments inherit previous normal or a default reference
            norms[i] = norms[i - 1] if i > 1 else np.array([0, 0, 1])

    # 3. Compute twist/rotation angle between consecutive bend planes
    for i in range(2, rows - 1):
        n_prev = norms[i - 1]
        n_curr = norms[i]
        dot_n = np.clip(np.dot(n_prev, n_curr), -1.0, 1.0)
        twist = np.rad2deg(np.arccos(dot_n))

        # Scalar triple product determines the clockwise/counter-clockwise orientation
        direction_test = np.dot(np.cross(n_prev, n_curr), vector[i] / v_mag[i])
        if direction_test < 0:
            twist = 360.0 - twist
        rot[i] = twist

    # 4. Straight feed calculations subtracting bend pullback offsets
    feed[1] = v_mag[1] - bend_offset[1]
    for i in range(2, rows - 1):
        feed[i] = v_mag[i] - bend_offset[i - 1] - bend_offset[i]
    feed[rows - 1] = v_mag[rows - 1] - bend_offset[rows - 2]

    cnc_data = []
    for i in range(1, rows):
        bend_id = f"Bend {i}" if i < rows - 1 else "End Cut"
        cnc_data.append((bend_id, round(feed[i], 3), round(rot[i] if i < rows - 1 else 0.0, 2), round(bends[i] if i < rows - 1 else 0.0, 2)))

    return cnc_data, bend_offset

def generate_tube_mesh(pts, b_radius, outer_radius, num_circle_pts=16, arc_resolution=10):
    """Generates a 3D polygonal mesh surface and a smoothed centerline path of the bent tube.
    
    ALGORITHM OVERVIEW:
    Part 1: Continuous Centerline Construction via Arc Interpolation
    - For every corner vertex, tangent points (T1, T2) are located via tangent offset: R * tan(theta / 2).
    - Arc center is computed by projecting along the angle bisector vector:
      center = vertex + bisector_unit * (R / cos(theta / 2)).
    - Discrete arc vertices are generated using Rodrigues' Rotation Formula around the normal axis:
      v_rot = v*cos(a) + (k x v)*sin(a) + k*(k . v)*(1 - cos(a)).
      
    Part 2: Rotation-Minimizing Frame (RMF) Surface Sweep
    - To prevent gimbal lock or seam twisting along arbitrary 3D spatial curves, an orthonormal frame
      (Tangent, Normal, Binormal) is initialized and transported along curve increments.
    - Each successive frame is rotated using the minimal angle between consecutive tangent vectors:
      axis = T_prev x T_curr, rotating the Normal vector via Rodrigues' formula.
    - Outer pipe vertices are sampled along the circular cross-section at each frame:
      P(theta) = Centerline + R_outer * (Normal * cos(theta) + Binormal * sin(theta)).
    """
    rows = pts.shape[0]
    centerline = [pts[0]]

    # --- Part 1: Centerline arc fillets generation ---
    for i in range(1, rows - 1):
        p_prev = pts[i - 1]
        p_curr = pts[i]
        p_next = pts[i + 1]

        v1 = p_curr - p_prev
        v2 = p_next - p_curr
        u1 = v1 / np.linalg.norm(v1)
        u2 = v2 / np.linalg.norm(v2)

        cos_theta = np.clip(np.dot(u1, u2), -1.0, 1.0)
        bend_rad = np.arccos(cos_theta)

        if bend_rad < 1e-4:
            centerline.append(p_curr)
            continue

        # Tangent departure and arrival points
        t_dist = b_radius * np.tan(bend_rad / 2.0)
        t1 = p_curr - u1 * t_dist
        t2 = p_curr + u2 * t_dist
        centerline.append(t1)

        # Center of curvature via bisector projection
        bisector = (u2 - u1)
        bisector_mag = np.linalg.norm(bisector)
        if bisector_mag > 1e-7:
            bisector_dir = bisector / bisector_mag
            center_dist = b_radius / np.cos(bend_rad / 2.0)
            arc_center = p_curr + bisector_dir * center_dist
            r_start = t1 - arc_center
            arc_axis = np.cross(u1, u2)
            arc_axis = arc_axis / np.linalg.norm(arc_axis)

            # Sample intermediate arc points via Rodrigues' rotation formula
            for ang in np.linspace(0, bend_rad, arc_resolution)[1:-1]:
                r_rot = (r_start * np.cos(ang) +
                         np.cross(arc_axis, r_start) * np.sin(ang) +
                         arc_axis * np.dot(arc_axis, r_start) * (1 - np.cos(ang)))
                centerline.append(arc_center + r_rot)

        centerline.append(t2)

    centerline.append(pts[-1])
    centerline = np.array(centerline)

    # --- Part 2: Rotation-Minimizing Frame surface sweep ---
    n_pts = len(centerline)
    theta = np.linspace(0, 2 * np.pi, num_circle_pts)
    cos_t, sin_t = np.cos(theta), np.sin(theta)

    mesh_x = np.zeros((n_pts, num_circle_pts))
    mesh_y = np.zeros((n_pts, num_circle_pts))
    mesh_z = np.zeros((n_pts, num_circle_pts))

    # Initialize reference coordinate frame at point 0
    t0 = centerline[1] - centerline[0]
    t0 /= np.linalg.norm(t0)
    arb = np.array([0, 0, 1]) if abs(t0[2]) < 0.9 else np.array([0, 1, 0])
    n0 = np.cross(t0, arb)
    n0 /= np.linalg.norm(n0)
    binormal = np.cross(t0, n0)
    normal = n0
    tangent = t0

    # Transport the frame along the path using minimal rotation
    for i in range(n_pts):
        if i > 0:
            prev_t = tangent
            if i < n_pts - 1:
                tangent = centerline[i + 1] - centerline[i]
                tangent /= np.linalg.norm(tangent)
            axis = np.cross(prev_t, tangent)
            axis_len = np.linalg.norm(axis)
            if axis_len > 1e-6:
                axis /= axis_len
                angle = np.arccos(np.clip(np.dot(prev_t, tangent), -1.0, 1.0))
                # Rotate the normal vector to match the change in tangent
                normal = (normal * np.cos(angle) + np.cross(axis, normal) * np.sin(angle) +
                          axis * np.dot(axis, normal) * (1 - np.cos(angle)))
            # Gram-Schmidt orthogonalization to enforce perpendicularity
            normal -= np.dot(normal, tangent) * tangent
            normal /= np.linalg.norm(normal)
            binormal = np.cross(tangent, normal)

        # Generate ring coordinates for outer tube radius
        circle_pts = (centerline[i][:, None] +
                      outer_radius * (normal[:, None] * cos_t + binormal[:, None] * sin_t))
        mesh_x[i, :] = circle_pts[0, :]
        mesh_y[i, :] = circle_pts[1, :]
        mesh_z[i, :] = circle_pts[2, :]

    return mesh_x, mesh_y, mesh_z, centerline


class Render3DWindow(tk.Toplevel):
    """Secondary Tkinter window providing an interactive 3D Matplotlib viewport of the bent tube."""

    def __init__(self, parent, pts, b_radius, od, wt, unit_str, initial_theme_name, show_grid_init=True):
        """Initializes viewport window, geometry variables, and UI controls."""
        super().__init__(parent)
        self.title("3D Pipe Form Visualization & Viewport")
        self.geometry("1020x820")
        self.minsize(800, 600)

        self.pts = pts
        self.b_radius = b_radius
        self.od = od
        self.wt = wt
        self.unit_str = unit_str
        self.selected_theme_name = initial_theme_name
        self.theme = self._resolve_theme(self.selected_theme_name)
        self.configure(bg=self.theme["bg"])

        # Display and geometry toggles
        self.show_tube = tk.BooleanVar(value=True)
        self.show_centerline = tk.BooleanVar(value=True)
        self.show_vertices = tk.BooleanVar(value=True)
        self.show_grid = tk.BooleanVar(value=show_grid_init)

        self._build_controls()
        self._build_plot()

    def _resolve_theme(self, name):
        """Resolves active theme dictionary, checking system dark mode if 'Auto' is selected."""
        if name == "Auto (OS Theme)":
            return PALETTES["Dark Modern"] if is_os_dark_mode() else PALETTES["Light Clean"]
        return PALETTES[name]

    def _build_controls(self):
        """Constructs UI toolbar buttons for camera presets, themes, and layer toggles."""
        self.control_bar = tk.Frame(self, bg=self.theme["frame_bg"], padx=8, pady=6)
        self.control_bar.pack(fill=tk.X, side=tk.TOP)

        btn_opts = {"bg": self.theme["btn_bg"], "fg": self.theme["btn_fg"], "relief": tk.RAISED, "padx": 5, "pady": 2}

        # Camera view buttons
        self.lbl_views = tk.Label(self.control_bar, text="Views:", bg=self.theme["frame_bg"], fg=self.theme["fg"], font=("Segoe UI", 9, "bold"))
        self.lbl_views.pack(side=tk.LEFT, padx=(0, 3))

        self.btn_iso = tk.Button(self.control_bar, text="Isometric", command=lambda: self.set_view(30, -45), **btn_opts)
        self.btn_iso.pack(side=tk.LEFT, padx=2)
        self.btn_front = tk.Button(self.control_bar, text="Front (X-Z)", command=lambda: self.set_view(0, -90), **btn_opts)
        self.btn_front.pack(side=tk.LEFT, padx=2)
        self.btn_top = tk.Button(self.control_bar, text="Top (X-Y)", command=lambda: self.set_view(90, -90), **btn_opts)
        self.btn_top.pack(side=tk.LEFT, padx=2)
        self.btn_side = tk.Button(self.control_bar, text="Side (Y-Z)", command=lambda: self.set_view(0, 0), **btn_opts)
        self.btn_side.pack(side=tk.LEFT, padx=2)
        self.btn_reset = tk.Button(self.control_bar, text="Reset", command=lambda: self.set_view(30, -60), **btn_opts)
        self.btn_reset.pack(side=tk.LEFT, padx=(2, 10))

        # Viewport theme selection
        self.lbl_theme = tk.Label(self.control_bar, text="Theme:", bg=self.theme["frame_bg"], fg=self.theme["fg"], font=("Segoe UI", 9, "bold"))
        self.lbl_theme.pack(side=tk.LEFT, padx=(0, 2))
        self.combo_theme = ttk.Combobox(self.control_bar, values=list(PALETTES.keys()), width=16, state="readonly")
        self.combo_theme.set(self.selected_theme_name)
        self.combo_theme.bind("<<ComboboxSelected>>", self.on_theme_change)
        self.combo_theme.pack(side=tk.LEFT, padx=(0, 10))

        # Checkboxes
        chk_opts = {"bg": self.theme["frame_bg"], "fg": self.theme["fg"],
                    "selectcolor": self.theme["entry_bg"], "activebackground": self.theme["frame_bg"]}

        self.chk_tube = tk.Checkbutton(self.control_bar, text="Tube", variable=self.show_tube, command=self.redraw, **chk_opts)
        self.chk_tube.pack(side=tk.LEFT, padx=3)
        self.chk_cl = tk.Checkbutton(self.control_bar, text="Centerline", variable=self.show_centerline, command=self.redraw, **chk_opts)
        self.chk_cl.pack(side=tk.LEFT, padx=3)
        self.chk_pt = tk.Checkbutton(self.control_bar, text="Vertices", variable=self.show_vertices, command=self.redraw, **chk_opts)
        self.chk_pt.pack(side=tk.LEFT, padx=3)
        self.chk_grid = tk.Checkbutton(self.control_bar, text="Grid", variable=self.show_grid, command=self.redraw, **chk_opts)
        self.chk_grid.pack(side=tk.LEFT, padx=3)

        self.lbl_info = tk.Label(self.control_bar, text="Left-Drag: Orbit | Right-Drag: Zoom | Mid-Drag: Pan",
                                 bg=self.theme["frame_bg"], fg="#a0a0a0", font=("Segoe UI", 8, "italic"))
        self.lbl_info.pack(side=tk.RIGHT, padx=4)

    def _build_plot(self):
        """Instantiates the Matplotlib 3D figure canvas and integrates navigation toolbar."""
        self.fig = plt.figure(figsize=(9, 7), facecolor=self.theme["plot_bg"])
        self.ax = self.fig.add_subplot(111, projection="3d", facecolor=self.theme["plot_bg"])

        r_outer = self.od / 2.0
        self.X, self.Y, self.Z, self.cl_pts = generate_tube_mesh(self.pts, self.b_radius, r_outer)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.toolbar = NavigationToolbar2Tk(self.canvas, self)
        self.toolbar.config(background=self.theme["frame_bg"])
        self.toolbar.update()
        self.toolbar.pack(side=tk.BOTTOM, fill=tk.X)

        self.redraw()

    def redraw(self):
        """Renders/redraws the 3D surface mesh, smoothed centerline, and vertex points in Matplotlib."""
        elev, azim = self.ax.elev, self.ax.azim
        self.ax.clear()

        t = self.theme
        self.fig.patch.set_facecolor(t["plot_bg"])
        self.ax.set_facecolor(t["plot_bg"])

        # 1. Outer Pipe Surface
        if self.show_tube.get():
            self.ax.plot_surface(
                self.X, self.Y, self.Z,
                color=t["tube_color"],
                alpha=0.45,
                edgecolor=t.get("tube_edge", t["plot_grid"]),
                linewidth=0.2,
                shade=True,
                zorder=2
            )

        # 2. Smooth Centerline Trace
        if self.show_centerline.get():
            self.ax.plot(
                self.cl_pts[:, 0], self.cl_pts[:, 1], self.cl_pts[:, 2],
                color=t["cl_color"],
                linestyle="-",
                linewidth=2.8,
                label="Centerline",
                zorder=5
            )

        # 3. Intersection Point Vertices
        if self.show_vertices.get():
            self.ax.scatter(
                self.pts[:, 0], self.pts[:, 1], self.pts[:, 2],
                color=t["pt_color"],
                edgecolors="#ffffff",
                linewidths=1.2,
                s=80,
                depthshade=False,
                label="Vertices (Intersection)",
                zorder=10
            )
            for idx, pt in enumerate(self.pts, 1):
                self.ax.text(
                    pt[0], pt[1], pt[2], f" P{idx}",
                    color=t["fg"],
                    fontsize=9,
                    fontweight="bold",
                    zorder=11
                )

        # 4. Aspect Ratio & Margins (Enforces equal bounding-box scaling)
        all_x = np.concatenate([self.X.flatten(), self.pts[:, 0]])
        all_y = np.concatenate([self.Y.flatten(), self.pts[:, 1]])
        all_z = np.concatenate([self.Z.flatten(), self.pts[:, 2]])

        max_span = max(all_x.max() - all_x.min(), all_y.max() - all_y.min(), all_z.max() - all_z.min(), 1e-3)
        half_span = (max_span / 2.0) * 1.25

        mid_x = (all_x.max() + all_x.min()) * 0.5
        mid_y = (all_y.max() + all_y.min()) * 0.5
        mid_z = (all_z.max() + all_z.min()) * 0.5

        self.ax.set_xlim(mid_x - half_span, mid_x + half_span)
        self.ax.set_ylim(mid_y - half_span, mid_y + half_span)
        self.ax.set_zlim(mid_z - half_span, mid_z + half_span)
        try:
            self.ax.set_box_aspect([1, 1, 1])
        except Exception:
            pass

        # 5. Coordinate Panes & Faint/Toggleable Grid
        for pane in [self.ax.xaxis.pane, self.ax.yaxis.pane, self.ax.zaxis.pane]:
            pane.fill = False
            pane.set_edgecolor(t["plot_grid"])
            pane.set_alpha(0.15)

        if self.show_grid.get():
            self.ax.grid(True, color=t["plot_grid"], linestyle=":", linewidth=0.5, alpha=0.3)
        else:
            self.ax.grid(False)

        self.ax.set_xlabel(f"X ({self.unit_str})", color=t["plot_fg"], labelpad=10, fontweight="bold")
        self.ax.set_ylabel(f"Y ({self.unit_str})", color=t["plot_fg"], labelpad=10, fontweight="bold")
        self.ax.set_zlabel(f"Z ({self.unit_str})", color=t["plot_fg"], labelpad=10, fontweight="bold")
        self.ax.tick_params(colors=t["plot_fg"], labelsize=8)

        id_tube = max(0.0, self.od - 2 * self.wt)
        self.ax.set_title(
            f"Formed Tube Model | OD: {self.od:.3f} {self.unit_str} | WT: {self.wt:.3f} {self.unit_str} (ID: {id_tube:.3f}) | CLR: {self.b_radius:.3f} {self.unit_str}",
            color=t["plot_fg"], fontsize=10, pad=12
        )

        # 6. Bottom Legend Docking
        handles, labels = self.ax.get_legend_handles_labels()
        if handles:
            self.fig.legend(
                handles, labels,
                loc="lower center",
                bbox_to_anchor=(0.5, 0.015),
                ncol=len(labels),
                facecolor=t["frame_bg"],
                edgecolor=t["plot_grid"],
                labelcolor=t["fg"],
                fontsize=9,
                framealpha=0.85
            )

        self.ax.view_init(elev=elev, azim=azim)
        self.fig.subplots_adjust(left=0.04, right=0.96, top=0.94, bottom=0.08)
        self.canvas.draw_idle()

    def set_view(self, elev, azim):
        """Updates camera elevation and azimuth angles and requests a redraw."""
        self.ax.view_init(elev=elev, azim=azim)
        self.canvas.draw_idle()

    def on_theme_change(self, _=None):
        """Updates UI control colors and re-renders plot with the selected palette."""
        self.selected_theme_name = self.combo_theme.get()
        self.theme = self._resolve_theme(self.selected_theme_name)
        self.configure(bg=self.theme["bg"])
        self.control_bar.configure(bg=self.theme["frame_bg"])

        for lbl in (self.lbl_views, self.lbl_theme):
            lbl.configure(bg=self.theme["frame_bg"], fg=self.theme["fg"])
        self.lbl_info.configure(bg=self.theme["frame_bg"])

        for chk in (self.chk_tube, self.chk_cl, self.chk_pt, self.chk_grid):
            chk.configure(bg=self.theme["frame_bg"], fg=self.theme["fg"],
                          selectcolor=self.theme["entry_bg"], activebackground=self.theme["frame_bg"])

        for btn in (self.btn_iso, self.btn_front, self.btn_top, self.btn_side, self.btn_reset):
            btn.configure(bg=self.theme["btn_bg"], fg=self.theme["btn_fg"])

        self.toolbar.config(background=self.theme["frame_bg"])
        self.redraw()


class TubeBenderGUI(tk.Tk):
    """Primary application window handling input tables, tooling, settings, and calculations."""

    def __init__(self):
        """Initializes main GUI window, config settings, and default coordinate rows."""
        super().__init__()
        self.title("CNC Tube Bender - Geometry Processor & Visualizer")
        self.geometry("1100x740")
        self.minsize(900, 560)

        self.protocol("WM_DELETE_WINDOW", self.on_close)

        self.config = load_config()
        self.current_unit = self.config.get("units", "mm")
        self.selected_theme_name = self.config.get("theme", "Auto (OS Theme)")
        if self.selected_theme_name not in PALETTES:
            self.selected_theme_name = "Auto (OS Theme)"
        self.active_palette = self._resolve_theme(self.selected_theme_name)

        self._create_widgets()
        self.apply_theme()

        default_pts = [
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 60.0),
            (151.299, 0.0, 289.944),
            (170.881, -2.612, 477.909),
            (188.059, 104.054, 505.755),
            (215.229, 245.949, 575.032),
            (180.276, 250.610, 578.739)
        ]
        factor = 1.0 / UNIT_FACTORS_TO_MM[self.current_unit]
        scaled_pts = [(pt[0]*factor, pt[1]*factor, pt[2]*factor) for pt in default_pts]
        self._populate_xyz_table(scaled_pts)

    def _resolve_theme(self, name):
        """Resolves active theme dictionary, checking system dark mode if 'Auto' is selected."""
        if name == "Auto (OS Theme)":
            return PALETTES["Dark Modern"] if is_os_dark_mode() else PALETTES["Light Clean"]
        return PALETTES[name]

    def _create_widgets(self):
        """Constructs layout, tables (XYZ and LRA), tooling frames, and action buttons."""
        self.top_bar = tk.Frame(self, padx=10, pady=8)
        self.top_bar.pack(fill=tk.X)

        self.tool_frame = tk.LabelFrame(self.top_bar, text=" Tooling Parameters ", padx=8, pady=4)
        self.tool_frame.pack(side=tk.LEFT, padx=(0, 10))

        tk.Label(self.tool_frame, text="CLR:").pack(side=tk.LEFT, padx=(0, 2))
        self.entry_radius = tk.Entry(self.tool_frame, width=7, font=("Segoe UI", 9))
        self.entry_radius.insert(0, f"{self.config.get('clr', 19.05):.3f}")
        self.entry_radius.pack(side=tk.LEFT, padx=(0, 8))

        tk.Label(self.tool_frame, text="OD:").pack(side=tk.LEFT, padx=(0, 2))
        self.entry_od = tk.Entry(self.tool_frame, width=7, font=("Segoe UI", 9))
        self.entry_od.insert(0, f"{self.config.get('od', 25.40):.3f}")
        self.entry_od.pack(side=tk.LEFT, padx=(0, 8))

        tk.Label(self.tool_frame, text="Wall:").pack(side=tk.LEFT, padx=(0, 2))
        self.entry_wt = tk.Entry(self.tool_frame, width=7, font=("Segoe UI", 9))
        self.entry_wt.insert(0, f"{self.config.get('wt', 2.00):.3f}")
        self.entry_wt.pack(side=tk.LEFT)

        self.env_frame = tk.LabelFrame(self.top_bar, text=" Preferences ", padx=8, pady=4)
        self.env_frame.pack(side=tk.LEFT, padx=(0, 10))

        tk.Label(self.env_frame, text="Units:").pack(side=tk.LEFT, padx=(0, 2))
        self.combo_unit = ttk.Combobox(self.env_frame, values=["mm", "cm", "inch"], width=5, state="readonly")
        self.combo_unit.set(self.current_unit)
        self.combo_unit.bind("<<ComboboxSelected>>", self.on_unit_change)
        self.combo_unit.pack(side=tk.LEFT, padx=(0, 8))

        tk.Label(self.env_frame, text="Theme:").pack(side=tk.LEFT, padx=(0, 2))
        self.combo_theme = ttk.Combobox(self.env_frame, values=list(PALETTES.keys()), width=16, state="readonly")
        self.combo_theme.set(self.selected_theme_name)
        self.combo_theme.bind("<<ComboboxSelected>>", self.on_theme_select)
        self.combo_theme.pack(side=tk.LEFT)

        self.btn_import = tk.Button(self.top_bar, text="Import File", command=self.import_file)
        self.btn_import.pack(side=tk.LEFT, padx=3)
        self.btn_calc = tk.Button(self.top_bar, text="Calculate CNC Data", font=("Segoe UI", 9, "bold"), command=self.calculate)
        self.btn_calc.pack(side=tk.LEFT, padx=3)
        self.btn_3d = tk.Button(self.top_bar, text="3D Model Viewer", command=self.open_3d_render)
        self.btn_3d.pack(side=tk.LEFT, padx=3)
        self.btn_export = tk.Button(self.top_bar, text="Export CSV", command=self.export_csv)
        self.btn_export.pack(side=tk.LEFT, padx=3)

        self.paned = ttk.PanedWindow(self, orient=tk.HORIZONTAL)
        self.paned.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        self.left_frame = tk.LabelFrame(self.paned, text=" XYZ Centerline Points ", padx=6, pady=6)
        self.paned.add(self.left_frame, weight=1)

        self.tree_xyz = ttk.Treeview(self.left_frame, columns=("Pnt", "X", "Y", "Z"), show="headings", selectmode="browse")
        for col in ("Pnt", "X", "Y", "Z"):
            self.tree_xyz.heading(col, text=col)
            self.tree_xyz.column(col, width=55, anchor=tk.CENTER)

        scroll_xyz = ttk.Scrollbar(self.left_frame, orient=tk.VERTICAL, command=self.tree_xyz.yview)
        self.tree_xyz.configure(yscrollcommand=scroll_xyz.set)
        self.tree_xyz.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        scroll_xyz.pack(side=tk.RIGHT, fill=tk.Y)

        self.xyz_btn_box = tk.Frame(self.left_frame, pady=4)
        self.xyz_btn_box.pack(fill=tk.X)
        self.btn_add = tk.Button(self.xyz_btn_box, text="Add Blank", command=self.add_row)
        self.btn_add.pack(side=tk.LEFT, padx=2)
        self.btn_del = tk.Button(self.xyz_btn_box, text="Delete", command=self.delete_row)
        self.btn_del.pack(side=tk.LEFT, padx=2)
        self.btn_clear = tk.Button(self.xyz_btn_box, text="Clear All", command=self.clear_xyz)
        self.btn_clear.pack(side=tk.LEFT, padx=2)

        self.entry_strip = tk.LabelFrame(self.left_frame, text=" Manual Coordinate Entry ", padx=4, pady=4)
        self.entry_strip.pack(fill=tk.X, pady=(2, 0))

        tk.Label(self.entry_strip, text="X:").pack(side=tk.LEFT)
        self.entry_x = tk.Entry(self.entry_strip, width=6)
        self.entry_x.pack(side=tk.LEFT, padx=(0, 4))
        tk.Label(self.entry_strip, text="Y:").pack(side=tk.LEFT)
        self.entry_y = tk.Entry(self.entry_strip, width=6)
        self.entry_y.pack(side=tk.LEFT, padx=(0, 4))
        tk.Label(self.entry_strip, text="Z:").pack(side=tk.LEFT)
        self.entry_z = tk.Entry(self.entry_strip, width=6)
        self.entry_z.pack(side=tk.LEFT, padx=(0, 4))
        self.btn_insert = tk.Button(self.entry_strip, text="Insert", command=self.insert_manual_pt)
        self.btn_insert.pack(side=tk.LEFT)

        self.right_frame = tk.LabelFrame(self.paned, text=" CNC Bend Schedule (L-R-A / Y-B-C) ", padx=6, pady=6)
        self.paned.add(self.right_frame, weight=1)

        cols_lra = ("Step", "Length (Feed)", "Rotation (Twist)", "Angle (Bend)")
        self.tree_lra = ttk.Treeview(self.right_frame, columns=cols_lra, show="headings", selectmode="browse")
        for col in cols_lra:
            self.tree_lra.heading(col, text=col)
            self.tree_lra.column(col, width=105, anchor=tk.CENTER)

        scroll_lra = ttk.Scrollbar(self.right_frame, orient=tk.VERTICAL, command=self.tree_lra.yview)
        self.tree_lra.configure(yscrollcommand=scroll_lra.set)
        self.tree_lra.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scroll_lra.pack(side=tk.RIGHT, fill=tk.Y)

        self.status = tk.Label(self, text="Ready.", relief=tk.SUNKEN, anchor=tk.W, padx=6, pady=3)
        self.status.pack(side=tk.BOTTOM, fill=tk.X)

    def apply_theme(self):
        """Applies colors from the active theme to all Tk and ttk widgets."""
        t = self.active_palette
        self.configure(bg=t["bg"])
        self.top_bar.configure(bg=t["bg"])
        self.xyz_btn_box.configure(bg=t["frame_bg"])

        for f in (self.left_frame, self.right_frame):
            f.configure(bg=t["frame_bg"], fg=t["fg"])

        self.tool_frame.configure(bg=t["group_tooling"], fg=t["fg"])
        for child in self.tool_frame.winfo_children():
            if isinstance(child, tk.Label):
                child.configure(bg=t["group_tooling"], fg=t["fg"])
            elif isinstance(child, tk.Entry):
                child.configure(bg=t["entry_bg"], fg=t["entry_fg"], insertbackground=t["fg"])

        self.env_frame.configure(bg=t["frame_bg"], fg=t["fg"])
        for child in self.env_frame.winfo_children():
            if isinstance(child, tk.Label):
                child.configure(bg=t["frame_bg"], fg=t["fg"])

        self.entry_strip.configure(bg=t["group_pt"], fg=t["fg"])
        for child in self.entry_strip.winfo_children():
            if isinstance(child, tk.Label):
                child.configure(bg=t["group_pt"], fg=t["fg"])
            elif isinstance(child, tk.Entry):
                child.configure(bg=t["entry_bg"], fg=t["entry_fg"], insertbackground=t["fg"])

        self.status.configure(bg=t["group_status"], fg=t["fg"])

        all_buttons = [self.btn_import, self.btn_calc, self.btn_3d, self.btn_export,
                       self.btn_add, self.btn_del, self.btn_clear, self.btn_insert]
        for b in all_buttons:
            b.configure(bg=t["btn_bg"], fg=t["btn_fg"], activebackground=t["tree_sel"], relief=tk.RAISED)

        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure("Treeview", background=t["tree_bg"], foreground=t["tree_fg"],
                        fieldbackground=t["tree_bg"], rowheight=22, font=("Consolas", 9))
        style.map("Treeview", background=[("selected", t["tree_sel"])])
        style.configure("Treeview.Heading", background=t["btn_bg"], foreground=t["btn_fg"], font=("Segoe UI", 9, "bold"))

    def persist_current_settings(self):
        """Reads user inputs from controls and persists them into the configuration file."""
        self.config["theme"] = self.selected_theme_name
        self.config["units"] = self.current_unit
        try:
            self.config["clr"] = float(self.entry_radius.get())
            self.config["od"] = float(self.entry_od.get())
            self.config["wt"] = float(self.entry_wt.get())
        except ValueError:
            pass
        save_config(self.config)

    def on_theme_select(self, _=None):
        """Callback triggered when the theme dropdown value changes."""
        self.selected_theme_name = self.combo_theme.get()
        self.active_palette = self._resolve_theme(self.selected_theme_name)
        self.apply_theme()
        self.persist_current_settings()

    def on_unit_change(self, _=None):
        """Callback for measurement unit changes; recalculates and scales table coordinates."""
        new_unit = self.combo_unit.get()
        if new_unit == self.current_unit:
            return

        ratio = UNIT_FACTORS_TO_MM[self.current_unit] / UNIT_FACTORS_TO_MM[new_unit]
        self.current_unit = new_unit

        for entry in (self.entry_radius, self.entry_od, self.entry_wt):
            try:
                val = float(entry.get())
                entry.delete(0, tk.END)
                entry.insert(0, f"{val * ratio:.3f}")
            except ValueError:
                pass

        for item in self.tree_xyz.get_children():
            vals = list(self.tree_xyz.item(item, "values"))
            for col_idx in (1, 2, 3):
                vals[col_idx] = f"{float(vals[col_idx]) * ratio:.3f}"
            self.tree_xyz.item(item, values=vals)

        if self.tree_lra.get_children():
            self.calculate()
        self.persist_current_settings()
        self.status.config(text=f"Units scaled to {new_unit} (Factor: {ratio:.4f}). Settings saved.")

    def on_close(self):
        """Handles application shutdown event and persists configurations."""
        self.persist_current_settings()
        self.destroy()

    def _populate_xyz_table(self, points):
        """Populates the XYZ Treeview table with point tuples or arrays."""
        self.clear_xyz()
        for idx, pt in enumerate(points, 1):
            self.tree_xyz.insert("", tk.END, values=(idx, f"{pt[0]:.3f}", f"{pt[1]:.3f}", f"{pt[2]:.3f}"))

    def add_row(self):
        """Appends a blank (0.000, 0.000, 0.000) coordinate row to the XYZ table."""
        idx = len(self.tree_xyz.get_children()) + 1
        self.tree_xyz.insert("", tk.END, values=(idx, "0.000", "0.000", "0.000"))

    def insert_manual_pt(self):
        """Validates and appends manual X, Y, Z entries to the coordinate table."""
        try:
            x = float(self.entry_x.get())
            y = float(self.entry_y.get())
            z = float(self.entry_z.get())
        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter valid numeric coordinates for X, Y, and Z.")
            return

        idx = len(self.tree_xyz.get_children()) + 1
        self.tree_xyz.insert("", tk.END, values=(idx, f"{x:.3f}", f"{y:.3f}", f"{z:.3f}"))
        self.entry_x.delete(0, tk.END)
        self.entry_y.delete(0, tk.END)
        self.entry_z.delete(0, tk.END)

    def delete_row(self):
        """Removes the selected row in the XYZ table and re-indexes row labels."""
        selected = self.tree_xyz.selection()
        if not selected:
            return
        self.tree_xyz.delete(selected[0])
        for i, item in enumerate(self.tree_xyz.get_children(), 1):
            vals = list(self.tree_xyz.item(item, "values"))
            vals[0] = i
            self.tree_xyz.item(item, values=vals)

    def clear_xyz(self):
        """Clears all entries from the XYZ table."""
        for item in self.tree_xyz.get_children():
            self.tree_xyz.delete(item)

    def import_file(self):
        """Parses coordinate lines from CSV, PTS, or text files using regex matching."""
        filepath = filedialog.askopenfilename(
            filetypes=[("Coordinate Files", "*.pts *.pts.* *.csv *.txt"), ("All Files", "*.*")]
        )
        if not filepath:
            return

        pts = []
        try:
            with open(filepath, "r", encoding="utf-8", errors="ignore") as f:
                for line in f:
                    match = re.findall(r"(-?\d+\.?\d*(?:[eE][-+]?\d+)?)", line)
                    if len(match) >= 3:
                        pts.append([float(v) for v in match[-3:]])

            if len(pts) < 2:
                messagebox.showerror("File Error", "Could not parse sufficient XYZ points from file.")
                return

            self._populate_xyz_table(pts)
            self.status.config(text=f"Loaded {len(pts)} points from {filepath}")
        except Exception as e:
            messagebox.showerror("Import Error", f"Failed to read file: {e}")

    def get_xyz_array(self):
        """Extracts and returns the XYZ coordinates from the table as a NumPy float array."""
        rows = self.tree_xyz.get_children()
        if not rows:
            return None
        data = []
        for r in rows:
            vals = self.tree_xyz.item(r, "values")
            data.append([float(vals[1]), float(vals[2]), float(vals[3])])
        return np.array(data, dtype=float)

    def _get_tooling_params(self):
        """Validates and parses tooling inputs (CLR, OD, Wall Thickness).
        
        Returns:
            tuple(float, float, float) or None: (CLR, OD, WT) if inputs are valid.
        """
        try:
            r_val = float(self.entry_radius.get())
            od_val = float(self.entry_od.get())
            wt_val = float(self.entry_wt.get())
            if r_val <= 0 or od_val <= 0 or wt_val <= 0:
                raise ValueError("Parameters must be strictly positive.")
            if 2 * wt_val >= od_val:
                raise ValueError("Wall thickness must be less than half of the outer diameter.")
            return r_val, od_val, wt_val
        except ValueError as e:
            messagebox.showerror("Tooling Input Error", str(e))
            return None

    def calculate(self):
        """Executes LRA calculations on current points and populates the results table."""
        params = self._get_tooling_params()
        if params is None:
            return
        r_val, _, _ = params

        pts = self.get_xyz_array()
        if pts is None or pts.shape[0] < 3:
            messagebox.showerror("Error", "At least 3 valid XYZ coordinate rows are required.")
            return

        try:
            cnc_rows, _ = compute_lra_data(pts, r_val)
        except Exception as err:
            messagebox.showerror("Calculation Error", str(err))
            return

        for item in self.tree_lra.get_children():
            self.tree_lra.delete(item)

        has_negative_feed = False
        for row in cnc_rows:
            if row[1] < 0:
                has_negative_feed = True
            self.tree_lra.insert("", tk.END, values=row)

        self.persist_current_settings()
        if has_negative_feed:
            self.status.config(text=f"Warning: Negative straight feed calculated in {self.current_unit}. Bend tangents overlap for selected CLR!")
        else:
            self.status.config(text=f"Calculation complete. {len(cnc_rows)} steps generated in {self.current_unit}.")

    def open_3d_render(self):
        """Opens the 3D Matplotlib visualization window with current points and tooling data."""
        params = self._get_tooling_params()
        if params is None:
            return
        r_val, od_val, wt_val = params

        pts = self.get_xyz_array()
        if pts is None or pts.shape[0] < 3:
            messagebox.showerror("Render Error", "At least 3 points are needed to render formed tube geometry.")
            return

        show_grid_setting = self.config.get("show_grid", True)
        Render3DWindow(self, pts, r_val, od_val, wt_val, self.current_unit, self.selected_theme_name, show_grid_setting)

    def export_csv(self):
        """Exports the generated CNC L-R-A bend schedule to a CSV file."""
        rows = self.tree_lra.get_children()
        if not rows:
            messagebox.showwarning("Export Warning", "No CNC data to export. Please calculate first.")
            return

        path = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV File", "*.csv")])
        if not path:
            return

        try:
            with open(path, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow([f"Step", f"Feed ({self.current_unit})", "Rotation (deg)", "Bend Angle (deg)"])
                for r in rows:
                    writer.writerow(self.tree_lra.item(r, "values"))
            self.status.config(text=f"Exported successfully to {path}")
        except Exception as e:
            messagebox.showerror("Export Error", f"Failed to save CSV: {e}")


if __name__ == "__main__":
    app = TubeBenderGUI()
    app.mainloop()