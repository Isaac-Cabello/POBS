"""SEN0599 framing and shared-host ping scheduling, independent of ROS."""

import math


class DistanceParser:
    """Recover checksum-valid millimeter readings from a UART byte stream."""

    def __init__(self):
        self.buffer = bytearray()

    def reset(self):
        """Discard bytes belonging to an earlier measurement."""
        self.buffer.clear()

    def feed(self, data):
        """Return decoded readings, tolerating partial frames and noise."""
        self.buffer.extend(data)
        readings = []
        while self.buffer:
            if self.buffer[0] != 0xff:
                del self.buffer[0]
                continue
            if len(self.buffer) < 4:
                break
            header, high, low, checksum = self.buffer[:4]
            if (header + high + low) & 0xff != checksum:
                del self.buffer[0]
                continue
            readings.append((high << 8) | low)
            del self.buffer[:4]
        return readings


class PingSchedule:
    """Give each process a window in a shared monotonic-clock cycle.

    All sonar processes must run on the same host with the same cycle period.
    Late callbacks skip their window rather than firing in another sonar's slot.
    """

    def __init__(self, period, offset, window=0.02):
        if not all(math.isfinite(v) for v in (period, offset, window)):
            raise ValueError('Ping timing values must be finite')
        if period < 0.019 or not 0 <= offset < period:
            raise ValueError('Require period >= 0.019 s and 0 <= offset < period')
        if not 0 < window <= period - offset:
            raise ValueError('Trigger window must fit inside the cycle')
        self.period = period
        self.offset = offset
        self.window = window
        self.last_cycle = None

    def due(self, now):
        """Consume at most one trigger opportunity per cycle."""
        cycle = math.floor(now / self.period)
        phase = now - cycle * self.period
        if (cycle != self.last_cycle
                and self.offset <= phase < self.offset + self.window):
            self.last_cycle = cycle
            return True
        return False
