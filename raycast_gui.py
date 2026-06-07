#!/usr/bin/env python3
"""
Offline Interactive Raycasting & Target Geolocation GUI
Allows point-selection on logged high-resolution imagery and projects
gimbal-lock-free 3D coordinates using precise ENU-to-NED rotation transformations.
"""

import csv
import math
import os
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import cv2
import numpy as np
import yaml
from PIL import Image, ImageTk
from scipy.spatial.transform import Rotation as R_scipy

R_EARTH = 6378137.0


class RaycastGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("CUASC Interactive Target Geolocation Desk")
        self.geometry("1400x900")

        # State Data
        self.session_dir = None
        self.metadata_records = []
        self.current_index = 0
        self.camera_k = None
        self.display_img = None
        self.raw_frame = None
        self._resize_timer = None

        self._setup_layout()

        # Bind the Configure event directly to the canvas instead of the root window
        # This prevents the infinite layout cascade loop and UI freezing
        self.canvas.bind("<Configure>", self._on_canvas_resize)

    def _setup_layout(self):
        # Left Side Control Panel - wrapped in scrollable container
        panel_container = ttk.Frame(self, width=350)
        panel_container.pack(side=tk.LEFT, fill=tk.Y, expand=False)
        panel_container.pack_propagate(False)

        # Embed a Canvas & Scrollbar within panel_container to handle small screens gracefully
        panel_canvas = tk.Canvas(
            panel_container, borderwidth=0, highlightthickness=0, width=330
        )
        scrollbar = ttk.Scrollbar(
            panel_container, orient="vertical", command=panel_canvas.yview
        )
        control_panel = ttk.Frame(panel_canvas, padding="10")

        control_panel.bind(
            "<Configure>",
            lambda e: panel_canvas.configure(scrollregion=panel_canvas.bbox("all")),
        )
        panel_canvas.create_window((0, 0), window=control_panel, anchor="nw")
        panel_canvas.configure(yscrollcommand=scrollbar.set)

        panel_canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        # Enable mousewheel scrolling
        def _on_mousewheel(event):
            panel_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

        panel_canvas.bind("<MouseWheel>", _on_mousewheel)
        control_panel.bind("<MouseWheel>", _on_mousewheel)

        # 1. Session Operations
        btn_load = ttk.Button(
            control_panel, text="📁 Open Flight Session", command=self._load_session
        )
        btn_load.pack(fill=tk.X, pady=5)

        self.lbl_session = ttk.Label(
            control_panel,
            text="No Session Loaded",
            wraplength=300,
            font=("Arial", 10, "italic"),
        )
        self.lbl_session.pack(fill=tk.X, pady=5)

        # 2. System Settings (Offsets & Orientations)
        ttk.Separator(control_panel, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
        ttk.Label(
            control_panel, text="Camera Mount Parameters", font=("Arial", 11, "bold")
        ).pack(anchor=tk.W)

        self.param_entries = {}
        defaults = [
            ("Mount X Offset (m)", "mount_x", "-0.127"),
            ("Mount Y Offset (m)", "mount_y", "0.0"),
            ("Mount Z Offset (m)", "mount_z", "-0.1524"),
            ("Mount Roll (deg)", "roll_deg", "0.0"),
            ("Mount Pitch (deg)", "pitch_deg", "0.0"),
            ("Mount Yaw (deg)", "yaw_deg", "0.0"),
            ("Ground Altitude AMSL (m)", "ground_z", "0.0"),
        ]
        for label, key, default_val in defaults:
            frame = ttk.Frame(control_panel)
            frame.pack(fill=tk.X, pady=2)
            ttk.Label(frame, text=label, width=22, anchor=tk.W).pack(side=tk.LEFT)
            entry = ttk.Entry(frame)
            entry.insert(0, default_val)
            entry.pack(side=tk.RIGHT, fill=tk.X, expand=True)
            self.param_entries[key] = entry

        # 3. Navigation
        ttk.Separator(control_panel, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
        nav_frame = ttk.Frame(control_panel)
        nav_frame.pack(fill=tk.X, pady=5)
        self.btn_prev = ttk.Button(
            nav_frame, text="◀ Prev", state=tk.DISABLED, command=self._prev_frame
        )
        self.btn_prev.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        self.btn_next = ttk.Button(
            nav_frame, text="Next ▶", state=tk.DISABLED, command=self._next_frame
        )
        self.btn_next.pack(side=tk.RIGHT, expand=True, fill=tk.X, padx=2)

        self.lbl_index = ttk.Label(control_panel, text="Frame: 0 / 0", anchor=tk.CENTER)
        self.lbl_index.pack(fill=tk.X, pady=2)

        # 4. Active Frame Telemetry Output Fields
        ttk.Separator(control_panel, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
        ttk.Label(
            control_panel, text="Active Frame Telemetry", font=("Arial", 11, "bold")
        ).pack(anchor=tk.W)
        self.txt_telemetry = tk.Text(
            control_panel, height=8, width=40, font=("Courier", 9)
        )
        self.txt_telemetry.pack(fill=tk.X, pady=5)

        # 5. Geolocation Outputs
        ttk.Separator(control_panel, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
        ttk.Label(
            control_panel, text="Raycast Result Matrix", font=("Arial", 11, "bold")
        ).pack(anchor=tk.W)
        self.txt_results = tk.Text(
            control_panel,
            height=10,
            width=40,
            font=("Courier", 10, "bold"),
            fg="darkgreen",
        )
        self.txt_results.pack(fill=tk.X, pady=5)

        # Right Side Visual Canvas Workspace
        self.canvas_frame = ttk.Frame(self, padding="5")
        self.canvas_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(self.canvas_frame, background="#0f172a")
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Button-1>", self._on_canvas_click)

    def _load_session(self):
        default_dir = os.path.abspath(os.path.expanduser("~/raycast_sessions"))
        if not os.path.exists(default_dir):
            os.makedirs(default_dir, exist_ok=True)

        chosen_dir = filedialog.askdirectory(
            initialdir=default_dir,
            title="Select Registered Flight Session Directory",
        )
        if not chosen_dir:
            return

        # Normalize chosen path to resolve symlinks, relative segments, and platform-specific slashes
        chosen_dir = os.path.abspath(os.path.normpath(chosen_dir))

        csv_file = os.path.abspath(os.path.join(chosen_dir, "telemetry_metadata.csv"))
        yaml_file = os.path.abspath(os.path.join(chosen_dir, "camera_info.yaml"))

        # Fail-safe diagnostic check
        if not (os.path.exists(csv_file) and os.path.exists(yaml_file)):
            messagebox.showerror(
                "Error Folder Layout",
                f"Missing required session files in the chosen folder:\n"
                f'"{chosen_dir}"\n\n'
                f"Please ensure you select the specific individual session folder (e.g., 'session_20260606_140300') "
                f"and not its parent directory.\n\n"
                f"Paths searched:\n"
                f"1. {csv_file} ({'FOUND' if os.path.exists(csv_file) else 'NOT FOUND'})\n"
                f"2. {yaml_file} ({'FOUND' if os.path.exists(yaml_file) else 'NOT FOUND'})",
            )
            return

        self.session_dir = chosen_dir
        self.lbl_session.config(text=os.path.basename(chosen_dir))

        # Parse Intrinsic Calibration parameters
        with open(yaml_file, "r") as f:
            try:
                cam_info = yaml.safe_load(f)
            except yaml.constructor.ConstructorError:
                f.seek(0)
                cam_info = yaml.full_load(f)
            self.camera_k = np.array(cam_info["camera_matrix"]["data"]).reshape((3, 3))

        # Parse saved Telemetry coordinates list
        self.metadata_records = []
        with open(csv_file, mode="r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                # Clean dictionary keys and values to protect against whitespaces
                clean_row = {
                    k.strip(): v.strip() for k, v in row.items() if k is not None
                }
                self.metadata_records.append(clean_row)

        if not self.metadata_records:
            messagebox.showwarning(
                "Empty Log Data", "Chosen session has no metadata entries."
            )
            return

        self.current_index = 0
        self._render_current_state()

    def _prev_frame(self):
        if self.current_index > 0:
            self.current_index -= 1
            self._render_current_state()

    def _next_frame(self):
        if self.current_index < len(self.metadata_records) - 1:
            self.current_index += 1
            self._render_current_state()

    def _render_current_state(self):
        if not self.metadata_records:
            return

        record = self.metadata_records[self.current_index]
        self.lbl_index.config(
            text=f"Frame: {self.current_index + 1} / {len(self.metadata_records)}"
        )

        # Update Navigation Button States dynamically
        if self.current_index == 0:
            self.btn_prev.config(state=tk.DISABLED)
        else:
            self.btn_prev.config(state=tk.NORMAL)

        if self.current_index >= len(self.metadata_records) - 1:
            self.btn_next.config(state=tk.DISABLED)
        else:
            self.btn_next.config(state=tk.NORMAL)

        # Teleplot diagnostics
        self.txt_telemetry.delete("1.0", tk.END)
        home_alt = float(record.get("home_alt", 0.0))

        telemetry_text = (
            f"Filename : {record['filename']}\n"
            f"Pos Local: ({float(record['drone_x']):.2f}, "
            f"{float(record['drone_y']):.2f}, "
            f"{float(record['drone_z']):.2f})\n"
            f"Quat Pose: ({float(record['qx']):.3f}, "
            f"{float(record['qy']):.3f}, "
            f"{float(record['qz']):.3f}, "
            f"{float(record['qw']):.3f})\n"
            f"Home Lat : {float(record['home_lat']):.7f}\n"
            f"Home Lon : {float(record['home_lon']):.7f}\n"
            f"Home Alt : {home_alt:.2f} m"
        )
        self.txt_telemetry.insert(tk.END, telemetry_text)

        # Draw Frame Image on Canvas with robust path check
        filename = record["filename"].strip()
        img_path = os.path.join(self.session_dir, "raw_frames", filename)

        if os.path.exists(img_path):
            # Attempt cv2 load, fallback to PIL if cv2 returns None
            self.raw_frame = cv2.imread(img_path)
            if self.raw_frame is None:
                try:
                    pil_img = Image.open(img_path)
                    self.raw_frame = cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)
                except Exception:
                    self.raw_frame = None

            if self.raw_frame is not None:
                self._update_canvas_image()
            else:
                self.canvas.delete("all")
                self.canvas.create_text(
                    400,
                    300,
                    text=f"Error decoding image frame '{filename}'",
                    fill="red",
                    font=("Arial", 12),
                )
        else:
            self.canvas.delete("all")
            self.canvas.create_text(
                400,
                300,
                text=f"Frame '{filename}' missing on disk.\nPath: {img_path}",
                fill="orange",
                font=("Arial", 12),
            )

    def _on_canvas_resize(self, event):
        # Debounce the canvas resizing handler to prevent rendering race conditions
        if self._resize_timer is not None:
            self.after_cancel(self._resize_timer)
        self._resize_timer = self.after(50, self._debounced_resize)

    def _debounced_resize(self):
        self._resize_timer = None
        if self.raw_frame is not None:
            self._update_canvas_image()

    def _update_canvas_image(self):
        if self.raw_frame is None:
            return

        canvas_w = self.canvas.winfo_width()
        canvas_h = self.canvas.winfo_height()

        # Safely fall back if the geometry manager hasn't finalized window mappings
        if canvas_w < 10 or canvas_h < 10:
            canvas_w, canvas_h = 1000, 700

        img_h, img_w = self.raw_frame.shape[:2]
        scale = min(canvas_w / img_w, canvas_h / img_h)
        new_w = max(1, int(img_w * scale))
        new_h = max(1, int(img_h * scale))

        resized = cv2.resize(
            self.raw_frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR
        )
        rgb_img = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        self.pil_img = Image.fromarray(rgb_img)
        self.display_img = ImageTk.PhotoImage(image=self.pil_img)

        self.canvas.delete("all")
        self.img_offset_x = (canvas_w - new_w) // 2
        self.img_offset_y = (canvas_h - new_h) // 2
        self.img_scale = scale

        self.canvas.create_image(
            self.img_offset_x,
            self.img_offset_y,
            anchor=tk.NW,
            image=self.display_img,
        )

        # CRITICAL FIX: Keep a strong reference to prevent garbage collection
        self.canvas.image = self.display_img

    def _on_canvas_click(self, event):
        if self.raw_frame is None or self.camera_k is None:
            return

        click_x = event.x - self.img_offset_x
        click_y = event.y - self.img_offset_y

        u = click_x / self.img_scale
        v = click_y / self.img_scale

        img_h, img_w = self.raw_frame.shape[:2]
        if not (0 <= u < img_w and 0 <= v < img_h):
            return

        # Redraw clean image background, then place target markers
        self._render_current_state()
        self.canvas.create_oval(
            event.x - 6,
            event.y - 6,
            event.x + 6,
            event.y + 6,
            outline="#f43f5e",
            width=2,
        )
        self.canvas.create_line(
            event.x - 15,
            event.y,
            event.x + 15,
            event.y,
            fill="#f43f5e",
            width=1.5,
        )
        self.canvas.create_line(
            event.x,
            event.y - 15,
            event.x,
            event.y + 15,
            fill="#f43f5e",
            width=1.5,
        )

        self._execute_raycast(u, v)

    def _execute_raycast(self, u, v):
        try:
            m_x = float(self.param_entries["mount_x"].get())
            m_y = float(self.param_entries["mount_y"].get())
            m_z = float(self.param_entries["mount_z"].get())
            m_roll = float(self.param_entries["roll_deg"].get())
            m_pitch = float(self.param_entries["pitch_deg"].get())
            m_yaw = float(self.param_entries["yaw_deg"].get())
            ground_z = float(self.param_entries["ground_z"].get())

            record = self.metadata_records[self.current_index]

            # 1. Optical unit vector calculation
            fx, fy = self.camera_k[0, 0], self.camera_k[1, 1]
            cx, cy = self.camera_k[0, 2], self.camera_k[1, 2]

            ray_opt = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
            ray_opt /= np.linalg.norm(ray_opt)

            # 2. OpenCV Optical coordinates mapping into Drone Body NED
            ray_body_ned = np.array([-ray_opt[1], ray_opt[0], ray_opt[2]])

            # Apply Euler camera mount rotations
            mount_rotation = R_scipy.from_euler(
                "xyz", [m_roll, m_pitch, m_yaw], degrees=True
            )
            ray_body_ned = mount_rotation.apply(ray_body_ned)

            # 3. Direct, gimbal-lock-free ENU-to-NED frame matrix transformation
            q = [
                float(record["qx"]),
                float(record["qy"]),
                float(record["qz"]),
                float(record["qw"]),
            ]
            r_enu = R_scipy.from_quat(q)

            R_enu_to_ned = R_scipy.from_matrix([[0, 1, 0], [1, 0, 0], [0, 0, -1]])

            # Combine matrix orientations directly to avoid gimbal lock singularities
            drone_r_ned = R_enu_to_ned * r_enu
            ray_world_ned = drone_r_ned.apply(ray_body_ned)
            ray_world_enu = np.array(
                [ray_world_ned[1], ray_world_ned[0], -ray_world_ned[2]]
            )

            # 4. Map camera physical position transformations
            mount_offset_ned = np.array([m_x, m_y, m_z])
            cam_offset_ned = drone_r_ned.apply(mount_offset_ned)
            cam_offset_enu = np.array(
                [cam_offset_ned[1], cam_offset_ned[0], -cam_offset_ned[2]]
            )

            drone_pos_z = float(record["drone_z"])
            cam_z = drone_pos_z + cam_offset_enu[2]

            # 5. Geodetic Earth intersection calculations
            if abs(ray_world_enu[2]) < 1e-6:
                raise ValueError(
                    "Calculated vector projection is horizontal to the horizon."
                )

            t = (ground_z - cam_z) / ray_world_enu[2]
            if t < 0:
                raise ValueError(
                    "Calculated vector projection points skyward (away from ground)."
                )

            target_x = (
                float(record["drone_x"]) + cam_offset_enu[0] + (t * ray_world_enu[0])
            )
            target_y = (
                float(record["drone_y"]) + cam_offset_enu[1] + (t * ray_world_enu[1])
            )

            # Home Origin mapping
            lat0 = float(record["home_lat"])
            lon0 = float(record["home_lon"])

            lat_offset = (target_y / R_EARTH) * (180.0 / math.pi)
            lon_scale = math.cos(math.radians(lat0))
            lon_offset = (target_x / (R_EARTH * lon_scale)) * (180.0 / math.pi)

            final_lat = lat0 + lat_offset
            final_lon = lon0 + lon_offset

            alt_mode = "Relative"

            # Print calculated results
            self.txt_results.delete("1.0", tk.END)
            result_text = (
                f"🎯 TARGET ESTIMATION\n"
                f"=====================\n"
                f"Mode      : {alt_mode}\n"
                f"Pixel U,V : ({u:.1f}, {v:.1f})\n"
                f"Ray Dist  : {t:.2f} meters\n"
                f"Local E,N : ({target_x:.2f}m, {target_y:.2f}m)\n\n"
                f"LATITUDE  : {final_lat:.7f}\n"
                f"LONGITUDE : {final_lon:.7f}"
            )
            self.txt_results.insert(tk.END, result_text)

        except Exception as err:
            messagebox.showerror(
                "Raycast Math Error", f"Raycast execution failed:\n{str(err)}"
            )


if __name__ == "__main__":
    app = RaycastGUI()
    app.mainloop()
