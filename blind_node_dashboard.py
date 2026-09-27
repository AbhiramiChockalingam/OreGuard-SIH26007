#!/usr/bin/env python3
"""
Blind Node Dashboard — staff-facing explainer view
-----------------------------------------------------
Reads the "DATA,..." lines printed by blind_node.ino over USB serial and
shows, in plain terms, what the blind node is doing right now:

    - Which vehicles it currently hears, and their RSSI-estimated distance
    - Whether each one counts as "detected" (inside the detection range)
    - The moment BOTH vehicles are detected at once, and it relays a
      cross-alert telling each vehicle about the other

This is NOT the in-vehicle dashboard — it's meant for demonstrating /
explaining the blind-node logic to staff.

Requires: pyserial   ->   pip install pyserial
Run:      python blind_node_dashboard.py
"""

import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

# ---------------- Configuration ----------------
BAUD_RATE = 115200
STALE_AFTER_SEC = 2.0          # a vehicle card greys out if not refreshed this long
ALERT_TTL_SEC = 1.5            # how long the "both detected" banner stays lit
BLIND_DETECT_RANGE_M = 20.0    # mirrors BLIND_DETECT_RANGE_M in blind_node.ino (display only)

BG = "#1e2228"
PANEL_BG = "#262b33"
FG = "#e8e8e8"
MUTED = "#8a8f98"
ACCENT_GREEN = "#2ecc71"
ACCENT_RED = "#e74c3c"
ACCENT_AMBER = "#f1c40f"
LANE_COLORS = ["#5dade2", "#af7ac5"]  # colors for up to 2 tracked vehicles' lanes


# ---------------- Data holders ----------------
class TrackedVehicle:
    def __init__(self, vehicle_id):
        self.vehicle_id = vehicle_id
        self.dist_m = 0.0
        self.last_update = 0.0

    def is_stale(self):
        return (time.time() - self.last_update) > STALE_AFTER_SEC


class AlertState:
    def __init__(self):
        self.id_a = "--"
        self.id_b = "--"
        self.dist_a = 0.0
        self.dist_b = 0.0
        self.last_update = 0.0

    def is_active(self):
        return self.last_update > 0.0 and (time.time() - self.last_update) <= ALERT_TTL_SEC


# ---------------- Serial reader thread ----------------
class SerialReader(threading.Thread):
    def __init__(self, port, baud, out_queue):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.queue = out_queue
        self._stop_event = threading.Event()
        self.ser = None

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
        except Exception as exc:
            self.queue.put(("ERROR", f"Could not open {self.port}: {exc}"))
            return

        self.queue.put(("STATUS", f"Connected to {self.port} @ {self.baud} baud"))
        while not self._stop_event.is_set():
            try:
                raw = self.ser.readline()
            except Exception as exc:
                self.queue.put(("ERROR", f"Serial read failed: {exc}"))
                break
            if not raw:
                continue
            line = raw.decode(errors="ignore").strip()
            if not line or not line.startswith("DATA,"):
                continue
            parsed = self._parse_data_line(line)
            if parsed:
                self.queue.put(parsed)

        if self.ser and self.ser.is_open:
            self.ser.close()

    @staticmethod
    def _parse_data_line(line):
        parts = line.split(",")
        if len(parts) < 2:
            return None
        role = parts[1]
        try:
            if role == "VEHICLE":
                # DATA,VEHICLE,id,distM,msSinceLastSeen
                if len(parts) != 5:
                    return None
                v = TrackedVehicle(parts[2])
                v.dist_m = float(parts[3])
                v.last_update = time.time()
                return ("VEHICLE", v)

            elif role == "ALERT":
                # DATA,ALERT,idA,idB,distA,distB
                if len(parts) != 6:
                    return None
                a = AlertState()
                a.id_a, a.id_b = parts[2], parts[3]
                a.dist_a, a.dist_b = float(parts[4]), float(parts[5])
                a.last_update = time.time()
                return ("ALERT", a)
        except (ValueError, IndexError):
            return None
        return None

    def stop(self):
        self._stop_event.set()


# ---------------- Junction schematic ----------------
class JunctionDiagram(tk.Canvas):
    """Draws the blind node at the center of a road with up to two tracked
    vehicles approaching from either side, positioned by their distance."""

    def __init__(self, master, **kwargs):
        super().__init__(master, bg=PANEL_BG, highlightthickness=0, **kwargs)
        self.vehicles = []  # list of (id, dist_m, lane_index, active)
        self.bind("<Configure>", lambda e: self.redraw())
        self.redraw()

    def update_vehicles(self, vehicles):
        self.vehicles = vehicles
        self.redraw()

    def redraw(self):
        self.delete("all")
        w = max(self.winfo_width(), 500)
        h = max(self.winfo_height(), 200)
        cy = h * 0.55
        cx = w * 0.5

        self.create_line(30, cy, w - 30, cy, fill="#3a4150", width=6)  # the road
        self.create_oval(cx - 16, cy - 16, cx + 16, cy + 16, fill=ACCENT_AMBER, outline="")
        self.create_text(cx, cy - 28, text="BLIND NODE", fill=ACCENT_AMBER, font=("Segoe UI", 10, "bold"))
        self.create_text(cx, h * 0.10, text=f"Detection range: {BLIND_DETECT_RANGE_M:.0f} m",
                          fill=MUTED, font=("Segoe UI", 9))

        half_road = (w - 60) / 2.0
        for i, (vid, dist_m, lane, active) in enumerate(self.vehicles):
            color = LANE_COLORS[lane % len(LANE_COLORS)]
            frac = min(dist_m / BLIND_DETECT_RANGE_M, 1.0)
            if lane == 0:
                x = cx - 30 - frac * (half_road - 30)
            else:
                x = cx + 30 + frac * (half_road - 30)
            fill = color if active else MUTED
            self.create_polygon(x - 12, cy + 16, x + 12, cy + 16, x, cy - 4,
                                 fill=fill, outline="")
            self.create_text(x, cy + 32, text=vid, fill=fill, font=("Segoe UI", 10, "bold"))
            self.create_text(x, cy + 48, text=f"{dist_m:.1f} m", fill=MUTED, font=("Segoe UI", 9))


# ---------------- Main application ----------------
class BlindNodeDashboardApp:
    def __init__(self, root):
        self.root = root
        root.title("Blind Node Dashboard (Staff View)")
        root.geometry("900x680")
        root.configure(bg=BG)
        root.minsize(700, 560)

        self.data_queue = queue.Queue()
        self.reader = None
        self.tracked = {}          # id -> TrackedVehicle
        self.lane_order = []       # ids in the order first seen, assigns lane 0/1/...
        self.alert = AlertState()

        self._build_style()
        self._build_ui()
        self._refresh_ports()

        self.root.after(150, self._poll_queue)
        self.root.after(300, self._tick_refresh)

    def _build_style(self):
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("TFrame", background=BG)
        style.configure("TLabel", background=BG, foreground=FG, font=("Segoe UI", 10))
        style.configure("Header.TLabel", background=BG, foreground=FG, font=("Segoe UI", 16, "bold"))
        style.configure("TButton", font=("Segoe UI", 10))

    def _build_ui(self):
        top = ttk.Frame(self.root, padding=(14, 10))
        top.pack(fill="x")
        ttk.Label(top, text="Blind Node Dashboard", style="Header.TLabel").pack(side="left")
        ttk.Label(top, text="  (staff explainer view)", foreground=MUTED, background=BG).pack(side="left")

        conn = ttk.Frame(self.root, padding=(14, 0, 14, 10))
        conn.pack(fill="x")
        ttk.Label(conn, text="Port:").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn, textvariable=self.port_var, width=18, state="readonly")
        self.port_combo.pack(side="left", padx=(6, 6))
        ttk.Button(conn, text="Refresh", command=self._refresh_ports).pack(side="left")
        self.connect_btn = ttk.Button(conn, text="Connect", command=self._toggle_connection)
        self.connect_btn.pack(side="left", padx=(10, 0))
        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(conn, textvariable=self.status_var, foreground=MUTED, background=BG).pack(side="left", padx=(14, 0))

        # Alert banner (frame always present; label shown/hidden)
        self.alert_frame = tk.Frame(self.root, bg=BG)
        self.alert_frame.pack(fill="x", padx=14, pady=(0, 6))
        self.alert_label = tk.Label(self.alert_frame, text="", bg=ACCENT_RED, fg="#1e2228",
                                     font=("Segoe UI", 12, "bold"), pady=10)

        # Vehicle cards row
        cards_frame = ttk.Frame(self.root, padding=(14, 0))
        cards_frame.pack(fill="x")
        cards_frame.columnconfigure(0, weight=1)
        cards_frame.columnconfigure(1, weight=1)
        self.card_a = self._build_vehicle_card(cards_frame)
        self.card_a[0].grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        self.card_b = self._build_vehicle_card(cards_frame)
        self.card_b[0].grid(row=0, column=1, sticky="nsew", padx=(8, 0))

        # Junction schematic
        diagram_frame = ttk.Frame(self.root, padding=(14, 12))
        diagram_frame.pack(fill="both", expand=True)
        ttk.Label(diagram_frame, text="Junction Schematic", style="Header.TLabel").pack(anchor="w", pady=(0, 6))
        diagram_border = tk.Frame(diagram_frame, bg="#3a4150", bd=0)
        diagram_border.pack(fill="both", expand=True)
        self.diagram = JunctionDiagram(diagram_border, height=220)
        self.diagram.pack(fill="both", expand=True, padx=2, pady=2)

        footer = ttk.Frame(self.root, padding=(14, 0, 14, 12))
        footer.pack(fill="x")
        ttk.Label(
            footer,
            text=("Distances are RSSI estimates from the blind node's radio, not GPS — coarse, "
                  "but enough to judge who is near the junction. When two different vehicle IDs are "
                  "detected at once, the blind node relays each one's presence to the other."),
            foreground=MUTED, background=BG, font=("Segoe UI", 9), wraplength=860,
        ).pack(anchor="w")

    def _build_vehicle_card(self, parent):
        border = tk.Frame(parent, bg="#3a4150", bd=0)
        inner = tk.Frame(border, bg=PANEL_BG)
        inner.pack(fill="both", expand=True, padx=2, pady=2)

        id_label = tk.Label(inner, text="No vehicle detected", bg=PANEL_BG, fg=MUTED,
                             font=("Segoe UI", 13, "bold"))
        id_label.pack(anchor="w", padx=14, pady=(12, 4))

        dist_label = tk.Label(inner, text="--", bg=PANEL_BG, fg=MUTED, font=("Segoe UI", 20, "bold"))
        dist_label.pack(anchor="w", padx=14)

        status_label = tk.Label(inner, text="", bg=PANEL_BG, fg=MUTED, font=("Segoe UI", 10))
        status_label.pack(anchor="w", padx=14, pady=(2, 12))

        return border, id_label, dist_label, status_label

    # ---------- serial connection ----------
    def _refresh_ports(self):
        if serial is None:
            self.port_combo["values"] = []
            return
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _toggle_connection(self):
        if self.reader is not None:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        if serial is None:
            messagebox.showerror("Missing dependency", "pyserial is not installed.\n\nRun:\n  pip install pyserial")
            return
        port = self.port_var.get()
        if not port:
            messagebox.showwarning("No port selected", "Choose a serial port first.")
            return
        self.reader = SerialReader(port, BAUD_RATE, self.data_queue)
        self.reader.start()
        self.connect_btn.config(text="Disconnect")
        self.status_var.set(f"Connecting to {port} ...")

    def _disconnect(self):
        if self.reader:
            self.reader.stop()
            self.reader = None
        self.connect_btn.config(text="Connect")
        self.status_var.set("Disconnected")

    # ---------- queue polling ----------
    def _poll_queue(self):
        try:
            while True:
                item = self.data_queue.get_nowait()
                if item[0] == "STATUS":
                    self.status_var.set(item[1])
                    continue
                if item[0] == "ERROR":
                    self.status_var.set("Error")
                    messagebox.showerror("Serial error", item[1])
                    self._disconnect()
                    continue

                kind, obj = item
                if kind == "VEHICLE":
                    self.tracked[obj.vehicle_id] = obj
                    if obj.vehicle_id not in self.lane_order:
                        self.lane_order.append(obj.vehicle_id)
                elif kind == "ALERT":
                    self.alert = obj
                self._refresh_ui()
        except queue.Empty:
            pass
        self.root.after(150, self._poll_queue)

    def _tick_refresh(self):
        # Drop vehicles that have gone stale so cards/diagram reflect reality
        for vid in list(self.tracked.keys()):
            if self.tracked[vid].is_stale():
                del self.tracked[vid]
        self._refresh_ui()
        self.root.after(300, self._tick_refresh)

    # ---------- rendering ----------
    def _refresh_ui(self):
        ids_present = [vid for vid in self.lane_order if vid in self.tracked][:2]
        cards = [self.card_a, self.card_b]
        for i, card in enumerate(cards):
            _, id_label, dist_label, status_label = card
            if i < len(ids_present):
                v = self.tracked[ids_present[i]]
                within_range = v.dist_m <= BLIND_DETECT_RANGE_M
                color = ACCENT_GREEN if within_range else MUTED
                id_label.config(text=f"Vehicle {v.vehicle_id}", fg=FG)
                dist_label.config(text=f"{v.dist_m:.2f} m", fg=color)
                status_label.config(
                    text=("Detected — within range" if within_range else "Seen, but outside detection range"),
                    fg=color)
            else:
                id_label.config(text="No vehicle detected", fg=MUTED)
                dist_label.config(text="--", fg=MUTED)
                status_label.config(text="", fg=MUTED)

        # Alert banner
        if self.alert.is_active():
            self.alert_label.config(
                text=(f"\u26A0  Both sides occupied: Vehicle {self.alert.id_a} @ {self.alert.dist_a:.2f} m "
                      f"and Vehicle {self.alert.id_b} @ {self.alert.dist_b:.2f} m — relaying cross-alert to both"))
            if not self.alert_label.winfo_ismapped():
                self.alert_label.pack(fill="x")
        else:
            if self.alert_label.winfo_ismapped():
                self.alert_label.pack_forget()

        # Diagram
        diagram_data = []
        for lane, vid in enumerate(ids_present):
            v = self.tracked[vid]
            within_range = v.dist_m <= BLIND_DETECT_RANGE_M
            diagram_data.append((vid, v.dist_m, lane, within_range))
        self.diagram.update_vehicles(diagram_data)

    def on_close(self):
        self._disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = BlindNodeDashboardApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
