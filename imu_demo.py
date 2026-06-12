"""IMU flight monitor — multi-source PFD dashboard.

Shows an artificial horizon with crash warning from any MAVLink flight
controller, the laptop's own IMU (where available), or a built-in simulation.
Display only — never sends commands that change autopilot state.

Run:  python3 imu_demo.py
"""
import collections
import time
import tkinter as tk
from tkinter import ttk

import safety
from pfd import PFD
from panels import AttitudeCard, GpsCard, LidarCard, PANEL_BG, GREEN, RED
from sources import ATTITUDE, GPS, LIDAR, Telemetry, SimSource, SourceLost

POLL_MS  = 50
FLASH_MS = 300
PFD_STALE_S = 1.0      # PFD greys out faster than the cards

SIM_LABEL    = 'Demo: Simulated'
LAPTOP_LABEL = 'Demo: Laptop IMU'

# USB descriptions that suggest a flight controller (for pre-selection only)
FC_KEYWORDS = ['ardupilot', 'pixhawk', 'px4', 'cuav', 'cubepilot', 'mro',
               'holybro', '3dr', 'stm32', 'stmicroelectronics']


def list_serial_ports():
    """Return {display_label: device_path}, preferring /dev/serial/by-id paths."""
    import os
    from serial.tools import list_ports
    by_id = {}
    try:
        d = '/dev/serial/by-id'
        for name in os.listdir(d):
            path = os.path.join(d, name)
            by_id[os.path.realpath(path)] = path
    except OSError:
        pass
    out = {}
    for p in list_ports.comports():
        device = by_id.get(p.device, p.device)
        label = device if device in by_id.values() else \
            f'{p.device} — {p.description}' if p.description else p.device
        out[label] = (device, (p.description or '') + ' ' + (p.manufacturer or ''))
    return out


class App:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title('IMU Flight Monitor')
        self.root.geometry('1000x720')
        self.root.minsize(900, 620)
        self.root.configure(bg='#0a0a0a')

        self.telemetry = Telemetry()
        self.source = None
        self.in_warning = False
        self.flash_on = False
        self.att_stamps = collections.deque(maxlen=200)
        self.last_att_stamp = None

        self._build_toolbar()
        self._build_main()
        self._build_statusbar()
        self._refresh_ports(preselect=True)

        self.root.after(POLL_MS, self._poll)
        self.root.after(FLASH_MS, self._flash_tick)

    # ── UI construction ────────────────────────────────────────────────────
    def _build_toolbar(self):
        bar = tk.Frame(self.root, bg='#1c1c1c')
        bar.pack(fill='x')

        tk.Label(bar, text='Source', bg='#1c1c1c', fg='#888',
                 font=('Arial', 9)).pack(side='left', padx=(10, 4), pady=6)
        self.source_var = tk.StringVar()
        # Editable: besides the listed ports you can type any MAVLink connection
        # string, e.g. udpin:0.0.0.0:14551 to read MAVProxy's UDP output
        self.source_box = ttk.Combobox(bar, textvariable=self.source_var, width=28)
        self.source_box.pack(side='left', pady=6)
        self.source_box.bind('<Button-1>', lambda e: self._refresh_ports())

        self.connect_btn = tk.Button(bar, text='Connect', command=self._toggle_connect,
                                     bg='#0a3a0a', fg=GREEN, activebackground='#0d4d0d',
                                     activeforeground=GREEN, relief='flat',
                                     font=('Arial', 9, 'bold'), padx=14)
        self.connect_btn.pack(side='left', padx=8, pady=6)

        self.link_dot = tk.Canvas(bar, width=12, height=12, bg='#1c1c1c',
                                  highlightthickness=0)
        self.link_dot.pack(side='right', padx=(4, 12))
        self.link_lbl = tk.Label(bar, text='idle', bg='#1c1c1c', fg='#888',
                                 font=('Arial', 9))
        self.link_lbl.pack(side='right')
        self.error_lbl = tk.Label(bar, text='', bg='#1c1c1c', fg='#ffaa00',
                                  font=('Arial', 9), anchor='e')
        self.error_lbl.pack(side='right', padx=10, fill='x', expand=True)
        self._set_link('idle')

    def _build_main(self):
        main = tk.Frame(self.root, bg='#0a0a0a')
        main.pack(fill='both', expand=True)

        self.pfd = PFD(main)
        self.pfd.canvas.pack(side='left', fill='both', expand=True)

        panel = tk.Frame(main, bg=PANEL_BG, width=290)
        panel.pack(side='left', fill='y')
        panel.pack_propagate(False)

        self.attitude_card = AttitudeCard(panel)
        self.gps_card = GpsCard(panel)
        self.lidar_card = LidarCard(panel)
        self.attitude_card.frame.pack(fill='x', padx=8, pady=(8, 0))
        self.gps_card.frame.pack(fill='x', padx=8, pady=(8, 0))
        self.lidar_card.frame.pack(fill='both', expand=True, padx=8, pady=8)

    def _build_statusbar(self):
        self.status = tk.Label(self.root, text='NO DATA', bg='#1c1c1c', fg='#888',
                               font=('Arial', 11, 'bold'), pady=6)
        self.status.pack(fill='x', side='bottom')

    # ── Source management ──────────────────────────────────────────────────
    def _refresh_ports(self, preselect=False):
        current = self.source_var.get()
        self.port_map = list_serial_ports()
        values = []
        try:
            from laptop_imu import probe_laptop_imu
            self._laptop_probe = probe_laptop_imu
            if probe_laptop_imu(dry_run=True):
                values.append(LAPTOP_LABEL)
        except Exception:
            self._laptop_probe = None
        values.append(SIM_LABEL)
        values.extend(self.port_map.keys())
        self.source_box.configure(values=values)
        if preselect or current not in values:
            fc = [lbl for lbl, (_, desc) in self.port_map.items()
                  if any(k in desc.lower() for k in FC_KEYWORDS)]
            self.source_var.set(fc[0] if len(fc) == 1 else SIM_LABEL)

    def _toggle_connect(self):
        if self.source is not None:
            self._disconnect('idle')
            return
        label = self.source_var.get()
        self.error_lbl.configure(text='')
        try:
            if label == SIM_LABEL:
                source = SimSource(seed=int(time.time()))
            elif label == LAPTOP_LABEL:
                source = self._laptop_probe()
                if source is None:
                    raise RuntimeError('laptop IMU no longer readable')
            else:
                from sources import MavlinkSource
                device = self.port_map.get(label, (label.strip(), ''))[0]
                source = MavlinkSource(device)
            source.start()
        except Exception as e:
            self.error_lbl.configure(text=self._friendly_error(e))
            return
        self.source = source
        self.telemetry = Telemetry()
        self.in_warning = False
        self.att_stamps.clear()
        self.last_att_stamp = None
        self.gps_card.reset()
        self.lidar_card.reset()
        self.connect_btn.configure(text='Disconnect', bg='#3a0a0a', fg='#ff8888',
                                   activebackground='#4d0d0d', activeforeground='#ff8888')
        self._set_link('linked')

    def _disconnect(self, state):
        if self.source is not None:
            try:
                self.source.stop()
            except Exception:
                pass
            self.source = None
        self.connect_btn.configure(text='Connect', bg='#0a3a0a', fg=GREEN,
                                   activebackground='#0d4d0d', activeforeground=GREEN)
        self._set_link(state)

    @staticmethod
    def _friendly_error(e):
        msg = str(e)
        if 'busy' in msg.lower() or 'PermissionError' in msg or 'denied' in msg.lower():
            return 'port busy or no permission — close Mission Planner/QGC, check dialout group'
        return msg[:90]

    def _set_link(self, state):
        col = {'idle': '#666666', 'linked': GREEN, 'lost': RED}[state]
        self.link_dot.delete('all')
        self.link_dot.create_oval(2, 2, 10, 10, fill=col, outline='')
        if state == 'idle':
            self.link_lbl.configure(text='idle', fg='#888')
        elif state == 'lost':
            self.link_lbl.configure(text='link lost', fg=RED)

    # ── Main loop ──────────────────────────────────────────────────────────
    def _flash_tick(self):
        self.flash_on = not self.flash_on
        self.root.after(FLASH_MS, self._flash_tick)

    def _poll(self):
        t = self.telemetry
        if self.source is not None:
            try:
                self.source.poll(t)
            except SourceLost as e:
                self._disconnect('lost')
                self.error_lbl.configure(text=self._friendly_error(e))
            except Exception as e:
                self._disconnect('lost')
                self.error_lbl.configure(text=self._friendly_error(e))

        # Attitude message rate for the toolbar
        stamp = t._updated.get(ATTITUDE)
        if stamp is not None and stamp != self.last_att_stamp:
            self.last_att_stamp = stamp
            self.att_stamps.append(time.monotonic())
        if self.source is not None:
            now = time.monotonic()
            rate = sum(1 for s in self.att_stamps if now - s <= 2.0) / 2.0
            self.link_lbl.configure(text=f'{rate:.0f} Hz', fg=GREEN)

        caps = self.source.capabilities if self.source else frozenset()
        att_age = t.age(ATTITUDE)
        pfd_live = att_age is not None and att_age <= PFD_STALE_S and self.source

        if pfd_live:
            warning = safety.is_unsafe(t.roll, t.pitch)
            self.pfd.draw(t.roll, t.pitch, t.yaw if t.yaw_valid else 0.0,
                          warning, self.flash_on)
            if warning and not self.in_warning:
                safety.beep(fallback=self.root.bell)
            self.in_warning = warning
        else:
            warning = False
            self.in_warning = False
            self.pfd.draw_no_data()

        self.attitude_card.update(t, stale=t.is_stale(ATTITUDE))
        self.gps_card.update(t, stale=t.is_stale(GPS), available=GPS in caps)
        self.lidar_card.update(t, stale=t.is_stale(LIDAR), available=LIDAR in caps)

        if not pfd_live:
            self.status.configure(text='NO DATA', bg='#1c1c1c', fg='#888')
        elif warning:
            flash_bg = '#5a0000' if self.flash_on else '#3a0000'
            self.status.configure(
                text=f'⚠ CRASH WARNING   Roll {t.roll:+.1f}°   Pitch {t.pitch:+.1f}°',
                bg=flash_bg, fg='#ffffff')
        else:
            yaw_txt = f'{t.yaw:+.1f}°' if t.yaw_valid else '—'
            self.status.configure(
                text=f'● SAFE   Roll {t.roll:+.1f}°   Pitch {t.pitch:+.1f}°   Yaw {yaw_txt}',
                bg='#0a3a0a', fg=GREEN)

        self.root.after(POLL_MS, self._poll)

    def run(self):
        self.root.mainloop()


def main():
    App().run()


if __name__ == '__main__':
    main()
