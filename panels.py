"""Right-column sensor cards: attitude, GPS readout + track plot, lidar scan map.

Each card greys out when its telemetry section is stale, or shows N/A when the
active source can never provide it. GPS and lidar cards pop out to a resizable
window on click.
"""
import math
import tkinter as tk

import safety
from sources import lidar_to_xy, latlon_to_xy

CARD_BG    = '#1e1e1e'
PANEL_BG   = '#161616'
PLOT_BG    = '#0d1117'
GRID_COL   = '#1d2430'
ACCENT     = '#88ccff'
BAR_COL    = '#00aaff'
DIM        = '#555555'
TEXT       = '#ffffff'
LABEL      = '#888888'
GREEN      = '#00cc44'
RED        = '#ff3333'

MAX_TRACK_POINTS = 10_000
MAX_LIDAR_POINTS = 5_000


class Card:
    """Dark card with a title row and a status tag (live / NO DATA / N/A)."""

    def __init__(self, parent, title, expandable=False):
        self.frame = tk.Frame(parent, bg=CARD_BG)
        self.header = tk.Frame(self.frame, bg=CARD_BG)
        self.header.pack(fill='x', padx=8, pady=(6, 2))
        self.title_lbl = tk.Label(self.header, text=title, bg=CARD_BG, fg=ACCENT,
                                  font=('Arial', 9, 'bold'))
        self.title_lbl.pack(side='left')
        self.status_lbl = tk.Label(self.header, text='', bg=CARD_BG, fg=DIM,
                                   font=('Arial', 8))
        self.status_lbl.pack(side='right')
        self.active = False
        if expandable:
            self.title_lbl.configure(cursor='hand2')

    def set_state(self, state):
        """state: 'live', 'stale', or 'unavailable'."""
        self.active = (state == 'live')
        if state == 'live':
            self.title_lbl.configure(fg=ACCENT)
            self.status_lbl.configure(text='')
        elif state == 'stale':
            self.title_lbl.configure(fg=DIM)
            self.status_lbl.configure(text='NO DATA')
        else:
            self.title_lbl.configure(fg=DIM)
            self.status_lbl.configure(text='not available')


class AttitudeCard(Card):
    def __init__(self, parent):
        super().__init__(parent, 'ATTITUDE')
        self.rows = {}
        for name in ('ROLL', 'PITCH', 'YAW'):
            row = tk.Frame(self.frame, bg=CARD_BG)
            row.pack(fill='x', padx=8, pady=(0, 4))
            lbl = tk.Label(row, text=name, bg=CARD_BG, fg=LABEL,
                           font=('Arial', 8), width=6, anchor='w')
            lbl.pack(side='left')
            val = tk.Label(row, text='--.-°', bg=CARD_BG, fg=TEXT,
                           font=('Arial', 13, 'bold'), width=7, anchor='e')
            val.pack(side='left')
            bar = tk.Canvas(row, height=5, bg='#333', highlightthickness=0)
            bar.pack(side='left', fill='x', expand=True, padx=(8, 0))
            self.rows[name] = (lbl, val, bar)

    def update(self, telemetry, stale):
        self.set_state('stale' if stale else 'live')
        t = telemetry
        axes = [
            ('ROLL',  t.roll,  abs(t.roll) > safety.ROLL_MAX_ABS_DEG, True),
            ('PITCH', t.pitch, t.pitch < safety.PITCH_MIN_DEG or t.pitch > safety.PITCH_MAX_DEG, True),
            ('YAW',   t.yaw,   False, t.yaw_valid),
        ]
        for name, value, warn, valid in axes:
            lbl, val, bar = self.rows[name]
            if stale or not valid:
                lbl.configure(fg=DIM)
                val.configure(fg=DIM, text='—' if not valid else f'{value:+.1f}°')
                bar.delete('all')
                continue
            lbl.configure(fg='#ff8888' if warn else LABEL,
                          text=f'{name} ⚠' if warn else name)
            val.configure(fg=RED if warn else TEXT, text=f'{value:+.1f}°')
            bar.delete('all')
            bw = bar.winfo_width()
            bw = bw if bw > 10 else 100
            rng = 180.0 if name in ('ROLL', 'YAW') else 90.0
            norm = max(0.0, min(1.0, (value + rng) / (2 * rng)))
            bar.create_rectangle(0, 0, bw, 5, fill='#333', outline='')
            bar.create_rectangle(0, 0, max(1, int(bw * norm)), 5,
                                 fill=RED if warn else BAR_COL, outline='')


class _PlotCard(Card):
    """Card with a square-ish plot canvas and an optional pop-out window."""

    def __init__(self, parent, title, plot_h=150):
        super().__init__(parent, title, expandable=True)
        self.canvas = tk.Canvas(self.frame, height=plot_h, bg=PLOT_BG,
                                highlightthickness=0, cursor='hand2')
        self.canvas.pack(fill='both', expand=True, padx=8, pady=(2, 8))
        self.popout = None
        self.popout_canvas = None
        for w in (self.canvas, self.title_lbl):
            w.bind('<Button-1>', self._toggle_popout)

    def _toggle_popout(self, _event=None):
        if self.popout is not None:
            self.popout.destroy()
            self._popout_closed()
            return
        self.popout = tk.Toplevel(self.frame)
        self.popout.title(self.title_lbl.cget('text'))
        self.popout.geometry('640x640')
        self.popout.configure(bg=PANEL_BG)
        self.popout.minsize(300, 300)
        self.popout_canvas = tk.Canvas(self.popout, bg=PLOT_BG, highlightthickness=0)
        self.popout_canvas.pack(fill='both', expand=True)
        self.popout.protocol('WM_DELETE_WINDOW', lambda: (self.popout.destroy(),
                                                          self._popout_closed()))

    def _popout_closed(self):
        self.popout = None
        self.popout_canvas = None

    def _canvases(self):
        out = [(self.canvas, self.canvas.winfo_width() or 200,
                self.canvas.winfo_height() or int(self.canvas.cget('height')))]
        if self.popout_canvas is not None:
            out.append((self.popout_canvas,
                        self.popout_canvas.winfo_width() or 640,
                        self.popout_canvas.winfo_height() or 640))
        return [(c, w if w > 10 else 200, h if h > 10 else 200) for c, w, h in out]


def _scale_to_fit(points, w, h, margin=18, min_half=5.0):
    """Pixels-per-meter so all points fit a centered square viewport."""
    half = min_half
    for x, y in points:
        half = max(half, abs(x), abs(y))
    return (min(w, h) / 2.0 - margin) / half


class GpsCard(_PlotCard):
    def __init__(self, parent):
        super().__init__(parent, 'GPS', plot_h=100)
        self.readout = tk.Frame(self.frame, bg=CARD_BG)
        self.readout.pack(fill='x', padx=8, before=self.canvas)
        self.fields = {}
        for r, name in enumerate(('FIX', 'LAT', 'LON', 'ALT')):
            tk.Label(self.readout, text=name, bg=CARD_BG, fg=LABEL,
                     font=('Arial', 8), anchor='w', width=4).grid(row=r, column=0, sticky='w')
            v = tk.Label(self.readout, text='--', bg=CARD_BG, fg=TEXT,
                         font=('Arial', 9), anchor='w')
            v.grid(row=r, column=1, sticky='w', padx=(4, 0))
            self.fields[name] = v
        self.readout.columnconfigure(1, weight=1)
        self.track = []          # [(x_east, y_north), ...] meters from first fix
        self.origin = None       # (lat0, lon0)

    def reset(self):
        self.track = []
        self.origin = None

    def update(self, telemetry, stale, available=True):
        if not available:
            self.set_state('unavailable')
        else:
            self.set_state('stale' if stale else 'live')
        t = telemetry
        dim = stale or not available
        has_fix = available and not stale and t.fix_type not in ('NO FIX', 'NO GPS')

        vals = {'FIX': f'{t.fix_type} · {t.sats} sats',
                'LAT': f'{t.lat:.6f}°', 'LON': f'{t.lon:.6f}°',
                'ALT': f'{t.alt_m:.1f} m · {t.speed_ms:.1f} m/s'}
        for name, lbl in self.fields.items():
            if dim:
                lbl.configure(fg=DIM, text='--')
            else:
                fg = TEXT
                if name == 'FIX':
                    fg = GREEN if has_fix else '#ffaa00'
                lbl.configure(fg=fg, text=vals[name])

        if has_fix:
            if self.origin is None:
                self.origin = (t.lat, t.lon)
            xy = latlon_to_xy(t.lat, t.lon, *self.origin)
            if not self.track or self.track[-1] != xy:
                self.track.append(xy)
                if len(self.track) > MAX_TRACK_POINTS:
                    del self.track[:len(self.track) - MAX_TRACK_POINTS]
        self._draw(dim)

    def _draw(self, dim):
        for c, w, h in self._canvases():
            c.delete('all')
            cx, cy = w / 2.0, h / 2.0
            col_grid = '#15191f' if dim else GRID_COL
            for f in (0.25, 0.5, 0.75):
                c.create_line(0, h * f, w, h * f, fill=col_grid)
                c.create_line(w * f, 0, w * f, h, fill=col_grid)
            c.create_text(12, 10, text='N ↑', fill=DIM, font=('Arial', 8), anchor='w')
            if dim:
                c.create_text(cx, cy, text='NO DATA', fill=DIM, font=('Arial', 11, 'bold'))
                continue
            if not self.track:
                c.create_text(cx, cy, text='waiting for fix…', fill=DIM, font=('Arial', 9))
                continue
            ppm = _scale_to_fit(self.track, w, h)
            px = [(cx + x * ppm, cy - y * ppm) for x, y in self.track]
            if len(px) > 1:
                c.create_line(*[v for pt in px for v in pt], fill=BAR_COL, width=2)
            sx, sy = px[0]
            c.create_oval(sx - 4, sy - 4, sx + 4, sy + 4, outline=GREEN, width=2)
            ex, ey = px[-1]
            c.create_oval(ex - 3, ey - 3, ex + 3, ey + 3, fill=TEXT, outline='')
            # Scale bar: nice round meters spanning ~1/3 of the width
            target_m = (w / 3.0) / ppm
            nice = 1
            for cand in (1, 2, 5, 10, 20, 50, 100, 200, 500, 1000):
                if cand <= target_m:
                    nice = cand
            bar_px = nice * ppm
            c.create_line(w - 12 - bar_px, h - 10, w - 12, h - 10, fill=LABEL)
            c.create_text(w - 12 - bar_px / 2, h - 18, text=f'{nice} m',
                          fill=LABEL, font=('Arial', 8))


class LidarCard(_PlotCard):
    def __init__(self, parent):
        super().__init__(parent, 'LIDAR MAP', plot_h=120)
        self.clear_btn = tk.Button(self.header, text='Clear', bg='#2a2a2a', fg=TEXT,
                                   activebackground='#3a3a3a', activeforeground=TEXT,
                                   relief='flat', font=('Arial', 8), pady=0,
                                   command=self.reset)
        self.clear_btn.pack(side='right', padx=(0, 6))
        self.points = []         # [(x_east, y_north), ...] meters, world frame
        self.yaw = 0.0

    def reset(self):
        self.points = []

    def update(self, telemetry, stale, available=True):
        if not available:
            self.set_state('unavailable')
        else:
            self.set_state('stale' if stale else 'live')
        dim = stale or not available
        if not dim:
            self.yaw = telemetry.yaw
            for bearing, dist in telemetry.lidar_returns:
                self.points.append(lidar_to_xy(bearing, dist, telemetry.yaw))
            if len(self.points) > MAX_LIDAR_POINTS:
                del self.points[:len(self.points) - MAX_LIDAR_POINTS]
        self._draw(dim)

    def _draw(self, dim):
        for c, w, h in self._canvases():
            c.delete('all')
            cx, cy = w / 2.0, h / 2.0
            if dim:
                ring_col = '#15191f'
                for f in (0.33, 0.66, 0.98):
                    rr = min(w, h) / 2.0 * f
                    c.create_oval(cx - rr, cy - rr, cx + rr, cy + rr, outline=ring_col)
                c.create_text(cx, cy, text='NO DATA',
                              fill=DIM, font=('Arial', 11, 'bold'))
                continue
            ppm = _scale_to_fit(self.points, w, h, min_half=2.0)
            # Range rings at nice round radii
            ring_m = 1
            for cand in (1, 2, 5, 10, 20, 50):
                if cand * ppm <= min(w, h) / 2.0 - 14:
                    ring_m = cand
            drawn = []
            for mult in (1, 2, 3):
                rr = ring_m * mult * ppm
                if rr > min(w, h) / 2.0:
                    break
                c.create_oval(cx - rr, cy - rr, cx + rr, cy + rr, outline=GRID_COL)
                drawn.append((rr, ring_m * mult))
            if drawn:
                rr, meters = drawn[-1]
                c.create_text(cx + rr - 3, cy - 9, text=f'{meters} m',
                              fill=DIM, font=('Arial', 7), anchor='e')
            c.create_text(12, 10, text='N ↑', fill=DIM, font=('Arial', 8), anchor='w')
            for x, y in self.points:
                px, py = cx + x * ppm, cy - y * ppm
                c.create_rectangle(px, py, px + 1.5, py + 1.5,
                                   fill='#00ffaa', outline='')
            # Vehicle heading arrow
            a = math.radians(self.yaw)
            tip = (cx + 10 * math.sin(a), cy - 10 * math.cos(a))
            left = (cx + 5 * math.sin(a + 2.5), cy - 5 * math.cos(a + 2.5))
            right = (cx + 5 * math.sin(a - 2.5), cy - 5 * math.cos(a - 2.5))
            c.create_polygon(*tip, *left, *right, fill=TEXT, outline='')
