"""Telemetry model and data sources (simulated; MAVLink and laptop IMU plug in here)."""
import math
import random
import time

ATTITUDE = 'attitude'
GPS      = 'gps'
LIDAR    = 'lidar'

STALE_AFTER_S = 2.0


class SourceLost(Exception):
    """Raised by a source's poll() when the link is irrecoverably gone."""

# Equator meters per degree of latitude (equirectangular approximation)
M_PER_DEG = 111_320.0


def lidar_to_xy(bearing_deg: float, distance_m: float, yaw_deg: float):
    """Body-frame lidar return -> world XY (x east, y north), vehicle at origin."""
    w = math.radians(yaw_deg + bearing_deg)
    return distance_m * math.sin(w), distance_m * math.cos(w)


def latlon_to_xy(lat: float, lon: float, lat0: float, lon0: float):
    """Lat/lon -> local meters (x east, y north) around (lat0, lon0)."""
    x = (lon - lon0) * M_PER_DEG * math.cos(math.radians(lat0))
    y = (lat - lat0) * M_PER_DEG
    return x, y


class Telemetry:
    """Latest values per section, each with its own last-update stamp."""

    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.yaw_valid = True
        self.fix_type = 'NO FIX'
        self.sats = 0
        self.lat = 0.0
        self.lon = 0.0
        self.alt_m = 0.0
        self.speed_ms = 0.0
        self.lidar_returns = []          # [(bearing_deg_body, distance_m), ...]
        self._updated = {}               # section -> monotonic time

    def touch(self, section, now=None):
        self._updated[section] = time.monotonic() if now is None else now

    def age(self, section, now=None):
        if section not in self._updated:
            return None
        now = time.monotonic() if now is None else now
        return now - self._updated[section]

    def is_stale(self, section, now=None):
        age = self.age(section, now=now)
        return age is None or age > STALE_AFTER_S


class Source:
    """Interface: start(), poll(telemetry), stop(); declares what it can provide."""
    name = 'source'
    capabilities = frozenset()

    def start(self):
        pass

    def poll(self, telemetry: Telemetry, now=None):
        raise NotImplementedError

    def stop(self):
        pass


_FIX_NAMES = {0: 'NO FIX', 1: 'NO FIX', 2: '2D', 3: '3D',
              4: 'DGPS', 5: 'RTK FLT', 6: 'RTK FIX'}

_UINT16_UNKNOWN = 65535


class MavlinkSource(Source):
    """Any MAVLink autopilot (ArduPilot, PX4, ...) over a serial port.

    Read-only apart from message-interval requests; never changes autopilot state.
    """

    name = 'MAVLink'
    capabilities = frozenset({ATTITUDE, GPS, LIDAR})

    MSG_RATES_HZ = {30: 20, 24: 2, 132: 10, 330: 10}   # ATTITUDE, GPS_RAW_INT, DISTANCE_SENSOR, OBSTACLE_DISTANCE

    def __init__(self, port, baud=115200):
        self.port = port
        self.baud = baud
        self.conn = None

    def start(self):
        from pymavlink import mavutil
        self.conn = mavutil.mavlink_connection(self.port, baud=self.baud)
        if self.conn.wait_heartbeat(timeout=5) is None:
            self.conn.close()
            self.conn = None
            raise RuntimeError('no MAVLink heartbeat — is this a flight controller?')
        self._request_messages()

    def _request_messages(self):
        """Ask for the streams we display; harmless if the FC ignores them."""
        from pymavlink import mavutil
        m = self.conn.mav
        target = (self.conn.target_system, self.conn.target_component)
        for msg_id, hz in self.MSG_RATES_HZ.items():
            m.command_long_send(*target,
                                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)
        # Old ArduPilot fallback
        m.request_data_stream_send(*target,
                                   mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

    def poll(self, telemetry: Telemetry, now=None):
        if self.conn is None:
            raise SourceLost('not connected')
        now = time.monotonic() if now is None else now
        for _ in range(100):                       # drain, but bound the work per tick
            try:
                msg = self.conn.recv_match(blocking=False)
            except Exception as e:
                raise SourceLost(f'serial link lost ({e})') from e
            if msg is None:
                break
            self._handle(msg, telemetry, now)

    def _handle(self, msg, telemetry: Telemetry, now):
        kind = msg.get_type()
        if kind == 'ATTITUDE':
            telemetry.roll = math.degrees(msg.roll)
            telemetry.pitch = math.degrees(msg.pitch)
            telemetry.yaw = math.degrees(msg.yaw)
            telemetry.yaw_valid = True
            telemetry.touch(ATTITUDE, now=now)
        elif kind == 'GPS_RAW_INT':
            telemetry.fix_type = _FIX_NAMES.get(msg.fix_type, f'FIX {msg.fix_type}')
            telemetry.lat = msg.lat / 1e7
            telemetry.lon = msg.lon / 1e7
            telemetry.alt_m = msg.alt / 1000.0
            if msg.vel != _UINT16_UNKNOWN:
                telemetry.speed_ms = msg.vel / 100.0
            telemetry.sats = msg.satellites_visible
            telemetry.touch(GPS, now=now)
        elif kind == 'DISTANCE_SENSOR':
            # Orientations 0..7 are yaw 0,45,...,315; ignore up/down/custom mounts
            if 0 <= msg.orientation <= 7:
                telemetry.lidar_returns = [(msg.orientation * 45.0,
                                            msg.current_distance / 100.0)]
                telemetry.touch(LIDAR, now=now)
        elif kind == 'OBSTACLE_DISTANCE':
            returns = []
            for i, d in enumerate(msg.distances):
                if d == _UINT16_UNKNOWN or d < msg.min_distance or d > msg.max_distance:
                    continue
                bearing = (msg.angle_offset + i * msg.increment) % 360.0
                returns.append((bearing, d / 100.0))
            telemetry.lidar_returns = returns
            telemetry.touch(LIDAR, now=now)

    def stop(self):
        if self.conn is not None:
            try:
                self.conn.close()
            finally:
                self.conn = None


class SimSource(Source):
    """Scripted gentle flight: attitude with periodic crash-warning excursions,
    a circular GPS walk, and a lidar sweep of a 6 m x 4 m room."""

    name = 'Demo: Simulated'
    capabilities = frozenset({ATTITUDE, GPS, LIDAR})

    ROOM_HALF_X = 3.0    # meters, east-west walls
    ROOM_HALF_Y = 2.0    # meters, north-south walls
    EXCURSION_PERIOD_S = 30.0
    EXCURSION_LEN_S = 4.0
    ORIGIN_LAT = 30.0
    ORIGIN_LON = 31.0

    def __init__(self, seed=0):
        rng = random.Random(seed)
        self._phase = [rng.uniform(0.0, 2.0 * math.pi) for _ in range(5)]
        self._noise = random.Random(seed + 1)
        self._t0 = None

    def poll(self, telemetry: Telemetry, now=None):
        now = time.monotonic() if now is None else now
        if self._t0 is None:
            self._t0 = now
        t = now - self._t0
        p = self._phase

        roll = 18.0 * math.sin(0.50 * t + p[0]) + 9.0 * math.sin(0.23 * t + p[1])
        pitch = 10.0 * math.sin(0.31 * t + p[2]) + 5.0 * math.sin(0.11 * t + p[3])
        # Periodic excursion blends roll toward 60 deg so the warning demos itself
        phase = t % self.EXCURSION_PERIOD_S
        start = self.EXCURSION_PERIOD_S - self.EXCURSION_LEN_S
        if phase >= start:
            env = math.sin(math.pi * (phase - start) / self.EXCURSION_LEN_S)
            roll = roll * (1.0 - env) + 60.0 * env
        telemetry.roll = roll
        telemetry.pitch = pitch
        telemetry.yaw = ((8.0 * t + math.degrees(p[4])) % 360.0) - 180.0
        telemetry.yaw_valid = True
        telemetry.touch(ATTITUDE, now=now)

        # GPS: 25 m-radius circle at walking speed, starting at the origin
        r, speed = 25.0, 1.4
        a = (speed / r) * t
        x = r * (math.sin(a + p[0]) - math.sin(p[0]))
        y = r * (math.cos(a + p[0]) - math.cos(p[0]))
        telemetry.fix_type = '3D'
        telemetry.sats = 12
        telemetry.lat = self.ORIGIN_LAT + y / M_PER_DEG
        telemetry.lon = self.ORIGIN_LON + x / (M_PER_DEG * math.cos(math.radians(self.ORIGIN_LAT)))
        telemetry.alt_m = 20.0 + 2.0 * math.sin(0.1 * t + p[2])
        telemetry.speed_ms = speed
        telemetry.touch(GPS, now=now)

        # Lidar: 8 rays spread around the sweep angle, ranged against the room walls
        base = (200.0 * t) % 360.0
        returns = []
        for k in range(8):
            bearing = (base + 45.0 * k) % 360.0
            world = math.radians(telemetry.yaw + bearing)
            dx, dy = math.sin(world), math.cos(world)
            dist = min(
                self.ROOM_HALF_X / abs(dx) if dx else math.inf,
                self.ROOM_HALF_Y / abs(dy) if dy else math.inf,
            )
            returns.append((bearing, dist + self._noise.uniform(-0.05, 0.05)))
        telemetry.lidar_returns = returns
        telemetry.touch(LIDAR, now=now)
