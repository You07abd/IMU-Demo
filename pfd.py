"""Artificial-horizon (PFD) canvas widget with heading tape and warning overlay."""
import math
import tkinter as tk

SKY_COL  = '#4a8ac4'
GND_COL  = '#7a5230'
SKY_WARN = '#7a2020'
GND_WARN = '#4a1010'

HEADING_H  = 28
PX_PER_DEG = 4


class PFD:
    """Owns a canvas; call draw() with the latest attitude or draw_no_data()."""

    def __init__(self, parent):
        self.canvas = tk.Canvas(parent, bg='#111', highlightthickness=0)

    def _geometry(self):
        c = self.canvas
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 50 or h < 50:           # not yet laid out
            w, h = 600, 470
        return w, h

    def draw_no_data(self):
        c = self.canvas
        c.delete('all')
        w, h = self._geometry()
        c.create_rectangle(0, 0, w, h, fill='#1c1c1c', outline='')
        c.create_text(w / 2, h / 2, text='NO DATA', fill='#666666',
                      font=('Arial', 18, 'bold'))

    def draw(self, roll_deg, pitch_deg, yaw_deg, warning, flash_on):
        c = self.canvas
        c.delete('all')
        w, h = self._geometry()
        pfd_top = HEADING_H
        cx = w / 2.0
        cy = (pfd_top + h) / 2.0

        r = math.radians(roll_deg)
        cos_r, sin_r = math.cos(r), math.sin(r)

        # Horizon-right and aircraft-up unit vectors in screen space
        hr_x, hr_y =  cos_r, -sin_r
        up_x, up_y = -sin_r, -cos_r

        # Horizon centre: pitch up -> horizon moves down on screen
        pitch_px = pitch_deg * PX_PER_DEG
        hx = cx - up_x * pitch_px
        hy = cy - up_y * pitch_px

        sky_col = SKY_WARN if warning else SKY_COL
        gnd_col = GND_WARN if warning else GND_COL

        # Ground base fill
        c.create_rectangle(0, pfd_top, w, h, fill=gnd_col, outline='')

        # Sky polygon
        big = max(w, h) * 4
        c.create_polygon(
            hx - big * hr_x,              hy - big * hr_y,
            hx + big * hr_x,              hy + big * hr_y,
            hx + big * hr_x + big * up_x, hy + big * hr_y + big * up_y,
            hx - big * hr_x + big * up_x, hy - big * hr_y + big * up_y,
            fill=sky_col, outline='',
        )

        # Pitch ladder
        for a in (-30, -20, -10, 10, 20, 30):
            mx = hx + a * PX_PER_DEG * up_x
            my = hy + a * PX_PER_DEG * up_y
            if not (pfd_top - 20 <= my <= h + 20):
                continue
            hl = 28 if abs(a) == 10 else 42
            c.create_line(mx - hl * hr_x, my - hl * hr_y,
                          mx + hl * hr_x, my + hl * hr_y,
                          fill='white', width=1)
            c.create_text(mx + (hl + 6) * hr_x, my + (hl + 6) * hr_y,
                          text=str(a), fill='white', font=('Arial', 8))

        # Horizon line
        c.create_line(hx - w * hr_x, hy - w * hr_y,
                      hx + w * hr_x, hy + w * hr_y,
                      fill='white', width=2)

        # Roll arc
        arc_cx, arc_cy, arc_r = cx, float(pfd_top + 62), 52.0
        c.create_arc(arc_cx - arc_r, arc_cy - arc_r,
                     arc_cx + arc_r, arc_cy + arc_r,
                     start=30, extent=120, style='arc',
                     outline='#aaaaaa', width=1)
        for t in (-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60):
            ang_t = math.radians(90.0 - t)
            inner = arc_r - (9 if t % 30 == 0 else 5)
            c.create_line(
                arc_cx + inner * math.cos(ang_t), arc_cy - inner * math.sin(ang_t),
                arc_cx + arc_r * math.cos(ang_t), arc_cy - arc_r * math.sin(ang_t),
                fill='#cccccc', width=1,
            )

        # Roll indicator triangle
        ang_r = math.radians(90.0 - roll_deg)
        tip = (arc_cx + (arc_r - 10) * math.cos(ang_r),
               arc_cy - (arc_r - 10) * math.sin(ang_r))
        bl  = (arc_cx + (arc_r + 3) * math.cos(ang_r + 0.13),
               arc_cy - (arc_r + 3) * math.sin(ang_r + 0.13))
        br  = (arc_cx + (arc_r + 3) * math.cos(ang_r - 0.13),
               arc_cy - (arc_r + 3) * math.sin(ang_r - 0.13))
        c.create_polygon(tip[0], tip[1], bl[0], bl[1], br[0], br[1],
                         fill='white', outline='')

        # Fixed aircraft symbol
        wing = 36
        c.create_line(cx - wing, cy, cx - 8, cy, fill='#ff4444', width=3)
        c.create_line(cx + 8, cy, cx + wing, cy, fill='#ff4444', width=3)
        c.create_line(cx - 8, cy, cx, cy + 8, fill='#ff4444', width=3)
        c.create_line(cx + 8, cy, cx, cy + 8, fill='#ff4444', width=3)
        c.create_oval(cx - 2, cy - 2, cx + 2, cy + 2, fill='#ff4444', outline='')

        # Warning overlay
        if warning:
            if flash_on:
                c.create_rectangle(0, pfd_top, w, h,
                                   fill='#cc0000', stipple='gray25', outline='')
            c.create_text(cx, cy - 44, text='⚠  CRASH  WARNING',
                          fill='white', font=('Arial', 16, 'bold'))

        self._draw_heading(yaw_deg, cx, w)

    def _draw_heading(self, yaw_deg, cx, w):
        c = self.canvas
        h = HEADING_H
        c.create_rectangle(0, 0, w, h, fill='#1a2a3a', outline='')
        hdg = yaw_deg % 360.0
        ppd = 3
        cards = {0: 'N', 90: 'E', 180: 'S', 270: 'W'}
        for off in range(-150, 151, 5):
            x = cx + off * ppd
            if not (0 <= x <= w):
                continue
            deg = (hdg + off) % 360
            ideg = int(round(deg / 10.0)) * 10 % 360
            is_major = (off % 30 == 0)
            tick_h = 7 if is_major else 4
            c.create_line(x, h - tick_h - 2, x, h - 2, fill='#8888cc')
            if is_major:
                label = cards.get(ideg, f'{ideg:03d}')
                col = '#ffffff' if label in cards.values() else '#88ccff'
                c.create_text(x, h // 2 - 1, text=label, fill=col, font=('Arial', 8))
        c.create_polygon(cx, h - 1, cx - 5, h - 8, cx + 5, h - 8,
                         fill='white', outline='')
