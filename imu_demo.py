"""Pixhawk IMU safety monitor — Mission Planner-style PFD.

Reads MAVLink ATTITUDE messages from an ArduPilot/Pixhawk board over USB
serial and shows an artificial horizon with a crash-warning overlay.
Display only — never sends commands to the autopilot.
"""
import math
import time
import subprocess
import sys
import tkinter as tk
import tkinter.font
from pymavlink import mavutil
from serial.tools import list_ports

# ── Constants ──────────────────────────────────────────────────────────────────
PITCH_MIN_DEG    = -30
PITCH_MAX_DEG    =  35
ROLL_MAX_ABS_DEG =  45
MAV_PORT = None   # set to e.g. 'COM5' to skip autodetect
MAV_BAUD = 115200
POLL_MS          = 50
FLASH_MS         = 300

WIN_W      = 800
WIN_H      = 520
PFD_W      = 600
DATA_W     = 200
HEADING_H  = 28
STATUS_H   = 22
PX_PER_DEG = 4

SKY_COL  = '#4a8ac4'
GND_COL  = '#7a5230'
SKY_WARN = '#7a2020'
GND_WARN = '#4a1010'
GREEN    = '#00cc44'
RED      = '#ff3333'

_in_warning = False
_flash_on   = False
last_attitude_time = None


# ── Connection ─────────────────────────────────────────────────────────────────
def autodetect_port():
    KNOWN = ['ardupilot', 'pixhawk', 'cuav', 'cubepilot', 'mro', 'holybro', '3dr', 'stm32', 'stmicroelectronics']
    candidates = []
    all_ports = list_ports.comports()
    for p in all_ports:
        desc = (p.description or '').lower()
        mfr  = (p.manufacturer or '').lower()
        if any(k in desc or k in mfr for k in KNOWN):
            candidates.append(p)
    if not candidates:
        available = ', '.join(p.device for p in all_ports) or 'none found'
        raise RuntimeError(
            f'No ArduPilot/Pixhawk serial port detected. '
            f'Available ports: {available}. '
            f'Set MAV_PORT manually to override.'
        )
    if len(candidates) > 1:
        print(f'[autodetect] Multiple candidates found, using {candidates[0].device}. Set MAV_PORT manually to override.')
    return candidates[0].device


# ── Logic ──────────────────────────────────────────────────────────────────────
def is_unsafe(roll: float, pitch: float) -> bool:
    return abs(roll) > ROLL_MAX_ABS_DEG or pitch < PITCH_MIN_DEG or pitch > PITCH_MAX_DEG


def beep() -> None:
    try:
        if sys.platform == 'win32':
            import winsound
            winsound.MessageBeep(winsound.MB_ICONHAND)
        elif sys.platform == 'darwin':
            subprocess.Popen(
                ['afplay', '/System/Library/Sounds/Sosumi.aiff'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
        else:
            subprocess.Popen(
                ['paplay', '/usr/share/sounds/freedesktop/stereo/bell.oga'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
    except Exception:
        root.bell()


# ── UI construction ────────────────────────────────────────────────────────────
def _make_card(parent):
    card = tk.Frame(parent, bg='#1e1e1e')
    card.pack(fill='x', padx=10, pady=6)
    lbl = tk.Label(card, text='', bg='#1e1e1e', fg='#888', font=_FONT_L)
    lbl.pack(anchor='w', padx=8, pady=(6, 0))
    val = tk.Label(card, text='--.-°', bg='#1e1e1e', fg='#fff', font=_FONT_V)
    val.pack(pady=(0, 4))
    bar = tk.Canvas(card, height=5, bg='#333', highlightthickness=0)
    bar.pack(fill='x', padx=8, pady=(0, 8))
    return card, lbl, val, bar


def build_ui() -> None:
    global root, pfd, panel, badge
    global _FONT_T, _FONT_L, _FONT_V, _FONT_B
    global roll_card, roll_lbl, roll_val, roll_bar
    global pitch_card, pitch_lbl, pitch_val, pitch_bar
    global yaw_card, yaw_lbl, yaw_val, yaw_bar

    root = tk.Tk()
    root.title('Pixhawk IMU Demo — PFD')
    root.geometry(f'{WIN_W}x{WIN_H}')
    root.resizable(False, False)
    root.configure(bg='#0a0a0a')

    container = tk.Frame(root, bg='#0a0a0a')
    container.pack(fill='both', expand=True)

    pfd = tk.Canvas(container, width=PFD_W, height=WIN_H,
                    bg='#111', highlightthickness=0)
    pfd.pack(side='left')

    panel = tk.Frame(container, bg='#161616', width=DATA_W)
    panel.pack(side='left', fill='both', expand=True)
    panel.pack_propagate(False)

    _FONT_T = tk.font.Font(family='Arial', size=10, weight='bold')
    _FONT_L = tk.font.Font(family='Arial', size=8)
    _FONT_V = tk.font.Font(family='Arial', size=20, weight='bold')
    _FONT_B = tk.font.Font(family='Arial', size=13, weight='bold')

    tk.Label(panel, text='IMU  DATA', bg='#161616', fg='#88ccff',
             font=_FONT_T).pack(pady=(14, 4))
    tk.Frame(panel, bg='#333', height=1).pack(fill='x', padx=10)

    roll_card,  roll_lbl,  roll_val,  roll_bar  = _make_card(panel)
    pitch_card, pitch_lbl, pitch_val, pitch_bar = _make_card(panel)
    yaw_card,   yaw_lbl,   yaw_val,   yaw_bar   = _make_card(panel)

    badge = tk.Label(panel, text='SAFE', bg='#0a3a0a', fg=GREEN,
                     font=_FONT_B, pady=10)
    badge.pack(fill='x', padx=10, pady=8, side='bottom')


# ── PFD rendering ──────────────────────────────────────────────────────────────
def draw_pfd(roll_deg: float, pitch_deg: float, yaw_deg: float,
             warning: bool, flash_on: bool) -> None:
    pfd.delete('all')

    W, H = PFD_W, WIN_H
    pfd_top = HEADING_H
    pfd_bot = H - STATUS_H
    cx = W / 2.0
    cy = (pfd_top + pfd_bot) / 2.0

    r = math.radians(roll_deg)
    cos_r, sin_r = math.cos(r), math.sin(r)

    # Horizon-right and aircraft-up unit vectors in screen space
    hr_x, hr_y =  cos_r, -sin_r
    up_x, up_y = -sin_r, -cos_r

    # Horizon centre: pitch up → horizon moves down on screen
    pitch_px = pitch_deg * PX_PER_DEG
    hx = cx - up_x * pitch_px
    hy = cy - up_y * pitch_px

    sky_col = SKY_WARN if warning else SKY_COL
    gnd_col = GND_WARN if warning else GND_COL

    # Ground base fill
    pfd.create_rectangle(0, pfd_top, W, pfd_bot, fill=gnd_col, outline='')

    # Sky polygon
    BIG = max(W, H) * 4
    pfd.create_polygon(
        hx - BIG * hr_x,              hy - BIG * hr_y,
        hx + BIG * hr_x,              hy + BIG * hr_y,
        hx + BIG * hr_x + BIG * up_x, hy + BIG * hr_y + BIG * up_y,
        hx - BIG * hr_x + BIG * up_x, hy - BIG * hr_y + BIG * up_y,
        fill=sky_col, outline='',
    )

    # Pitch ladder
    for a in (-30, -20, -10, 10, 20, 30):
        mx = hx + a * PX_PER_DEG * up_x
        my = hy + a * PX_PER_DEG * up_y
        if not (pfd_top - 20 <= my <= pfd_bot + 20):
            continue
        hl = 28 if abs(a) == 10 else 42
        pfd.create_line(mx - hl * hr_x, my - hl * hr_y,
                         mx + hl * hr_x, my + hl * hr_y,
                         fill='white', width=1)
        lx = mx + (hl + 6) * hr_x
        ly = my + (hl + 6) * hr_y
        pfd.create_text(lx, ly, text=str(a), fill='white', font=('Arial', 8))

    # Horizon line
    pfd.create_line(hx - W * hr_x, hy - W * hr_y,
                     hx + W * hr_x, hy + W * hr_y,
                     fill='white', width=2)

    # Roll arc
    arc_cx, arc_cy, arc_r_px = cx, float(pfd_top + 62), 52.0
    pfd.create_arc(arc_cx - arc_r_px, arc_cy - arc_r_px,
                    arc_cx + arc_r_px, arc_cy + arc_r_px,
                    start=30, extent=120, style='arc',
                    outline='#aaaaaa', width=1)
    for t in (-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60):
        ang_t = math.radians(90.0 - t)
        inner = arc_r_px - (9 if t % 30 == 0 else 5)
        pfd.create_line(
            arc_cx + inner * math.cos(ang_t),    arc_cy - inner * math.sin(ang_t),
            arc_cx + arc_r_px * math.cos(ang_t), arc_cy - arc_r_px * math.sin(ang_t),
            fill='#cccccc', width=1,
        )

    # Roll indicator triangle
    ang_r = math.radians(90.0 - roll_deg)
    tip = (arc_cx + (arc_r_px - 10) * math.cos(ang_r),
           arc_cy - (arc_r_px - 10) * math.sin(ang_r))
    bl  = (arc_cx + (arc_r_px + 3) * math.cos(ang_r + 0.13),
           arc_cy - (arc_r_px + 3) * math.sin(ang_r + 0.13))
    br  = (arc_cx + (arc_r_px + 3) * math.cos(ang_r - 0.13),
           arc_cy - (arc_r_px + 3) * math.sin(ang_r - 0.13))
    pfd.create_polygon(tip[0], tip[1], bl[0], bl[1], br[0], br[1],
                        fill='white', outline='')

    # Fixed aircraft symbol
    WING = 36
    pfd.create_line(cx - WING, cy, cx - 8, cy, fill='#ff4444', width=3)
    pfd.create_line(cx + 8, cy, cx + WING, cy, fill='#ff4444', width=3)
    pfd.create_line(cx - 8, cy, cx, cy + 8, fill='#ff4444', width=3)
    pfd.create_line(cx + 8, cy, cx, cy + 8, fill='#ff4444', width=3)
    pfd.create_oval(cx - 2, cy - 2, cx + 2, cy + 2, fill='#ff4444', outline='')

    # Warning overlay
    if warning:
        if flash_on:
            pfd.create_rectangle(0, pfd_top, W, pfd_bot,
                                  fill='#cc0000', stipple='gray25', outline='')
        pfd.create_text(cx, cy - 44, text='⚠  CRASH  WARNING',
                         fill='white', font=('Arial', 16, 'bold'))

    # Heading tape
    _draw_heading(yaw_deg, cx)

    # Status bar
    pfd.create_rectangle(0, pfd_bot, W, H, fill='#1a1a1a', outline='')
    s_col = RED if warning else GREEN
    s_txt = ('⚠ UNSAFE' if warning else '● SAFE') + \
            f'   Roll {roll_deg:+.1f}°   Pitch {pitch_deg:+.1f}°   Yaw {yaw_deg:+.1f}°'
    pfd.create_text(10, pfd_bot + STATUS_H // 2,
                     text=s_txt, fill=s_col, anchor='w', font=('Arial', 8))


def _draw_heading(yaw_deg: float, cx: float) -> None:
    W = PFD_W
    H = HEADING_H
    pfd.create_rectangle(0, 0, W, H, fill='#1a2a3a', outline='')
    hdg = yaw_deg % 360.0
    PPD = 3
    CARDS = {0: 'N', 90: 'E', 180: 'S', 270: 'W'}
    for off in range(-100, 101, 5):
        x = cx + off * PPD
        if not (0 <= x <= W):
            continue
        deg  = (hdg + off) % 360
        ideg = int(round(deg / 10.0)) * 10 % 360
        is_major = (off % 30 == 0)
        tick_h = 7 if is_major else 4
        pfd.create_line(x, H - tick_h - 2, x, H - 2, fill='#8888cc')
        if is_major:
            label = CARDS.get(ideg, f'{ideg:03d}')
            col   = '#ffffff' if label in CARDS.values() else '#88ccff'
            pfd.create_text(x, H // 2 - 1, text=label, fill=col, font=('Arial', 8))
    pfd.create_polygon(cx, H - 1, cx - 5, H - 8, cx + 5, H - 8,
                        fill='white', outline='')


def update_panel(roll_deg: float, pitch_deg: float, yaw_deg: float,
                 warning: bool, flash_on: bool) -> None:
    bg    = '#1a0000' if warning else '#161616'
    c_bg  = '#2a0000' if warning else '#1e1e1e'
    c_dim = '#ff8888' if warning else '#888888'
    panel.configure(bg=bg)

    axes = [
        (roll_card,  roll_lbl,  roll_val,  roll_bar,
         roll_deg,  'ROLL',  abs(roll_deg) > ROLL_MAX_ABS_DEG),
        (pitch_card, pitch_lbl, pitch_val, pitch_bar,
         pitch_deg, 'PITCH', pitch_deg < PITCH_MIN_DEG or pitch_deg > PITCH_MAX_DEG),
        (yaw_card,   yaw_lbl,   yaw_val,   yaw_bar,
         yaw_deg,   'YAW',   False),
    ]
    for card, lbl, val, bar, value, name, ax_warn in axes:
        card.configure(bg=c_bg)
        lbl.configure(bg=c_bg, fg=c_dim, text=f'{name} ⚠' if ax_warn else name)
        val.configure(bg=c_bg, fg=RED if ax_warn else '#ffffff',
                       text=f'{value:+.1f}°')
        bar.delete('all')
        bw = bar.winfo_width()
        bw = bw if bw > 10 else DATA_W - 20
        norm = ((value + 180.0) / 360.0) if name in ('ROLL', 'YAW') else \
               ((value + 90.0)  / 180.0)
        norm    = max(0.0, min(1.0, norm))
        fill_w  = max(1, int(bw * norm))
        bar.create_rectangle(0, 0, bw,     5, fill='#333',                      outline='')
        bar.create_rectangle(0, 0, fill_w, 5, fill=RED if ax_warn else '#00aaff', outline='')

    if warning and flash_on:
        badge.configure(bg='#3a0000', fg=RED,      text='CRASH WARNING')
    elif warning:
        badge.configure(bg='#260000', fg='#ff6666', text='CRASH WARNING')
    else:
        badge.configure(bg='#0a3a0a', fg=GREEN,     text='SAFE')


# ── Callbacks ──────────────────────────────────────────────────────────────────
def flash_tick() -> None:
    global _flash_on
    _flash_on = not _flash_on
    root.after(FLASH_MS, flash_tick)


def poll() -> None:
    global _in_warning, last_attitude_time
    msg = conn.recv_match(type='ATTITUDE', blocking=False)
    if msg is not None:
        last_attitude_time = time.monotonic()
        roll  = math.degrees(msg.roll)
        pitch = math.degrees(msg.pitch)
        yaw   = math.degrees(msg.yaw)
        warn  = is_unsafe(roll, pitch)
        draw_pfd(roll, pitch, yaw, warn, _flash_on)
        update_panel(roll, pitch, yaw, warn, _flash_on)
        if warn and not _in_warning:
            beep()
        _in_warning = warn
    if last_attitude_time is not None and time.monotonic() - last_attitude_time > 1.0:
        draw_pfd(0.0, 0.0, 0.0, False, False)
        update_panel(0.0, 0.0, 0.0, False, False)
        pfd_bot = WIN_H - STATUS_H
        pfd.create_rectangle(0, pfd_bot, PFD_W, WIN_H, fill='#1a1a1a', outline='')
        pfd.create_text(10, pfd_bot + STATUS_H // 2,
                        text='NO DATA', fill='#888888',
                        anchor='w', font=('Arial', 8))
        _in_warning = False
    root.after(POLL_MS, poll)


# ── Entry point ────────────────────────────────────────────────────────────────
def main() -> None:
    global conn
    port = MAV_PORT or autodetect_port()
    print(f'[autodetect] Connecting to {port} at {MAV_BAUD} baud')
    conn = mavutil.mavlink_connection(port, baud=MAV_BAUD)
    build_ui()
    root.after(POLL_MS, poll)
    root.after(FLASH_MS, flash_tick)
    root.mainloop()


if __name__ == '__main__':
    main()
