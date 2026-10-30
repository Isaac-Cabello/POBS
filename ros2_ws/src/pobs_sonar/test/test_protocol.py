"""Exercise byte framing and multi-process polling without sensor hardware."""

from pobs_sonar.protocol import DistanceParser, PingSchedule
import pytest


def frame(distance):
    high, low = divmod(distance, 256)
    return bytes([0xff, high, low, (0xff + high + low) & 0xff])


def test_partial_frames_noise_and_checksum_recovery():
    parser = DistanceParser()
    assert parser.feed(b'noise' + frame(1953)[:2]) == []
    assert parser.feed(frame(1953)[2:]) == [1953]
    corrupt = bytearray(frame(1200))
    corrupt[-1] ^= 0x80
    assert parser.feed(corrupt + frame(2500) + frame(6000)) == [2500, 6000]
    parser.feed(frame(1000)[:2])
    parser.reset()
    assert parser.feed(frame(500)) == [500]


def test_three_schedules_stagger_pings_and_skip_late_callbacks():
    schedules = [PingSchedule(0.3, offset) for offset in (0.0, 0.1, 0.2)]
    for index, now in enumerate((0.005, 0.105, 0.205)):
        assert [schedule.due(now) for schedule in schedules] == [
            other == index for other in range(3)]
        assert not any(schedule.due(now + 0.001) for schedule in schedules)
    assert not any(schedule.due(0.29) for schedule in schedules)
    # A process missing an entire cycle never fires a catch-up burst.
    assert not schedules[0].due(0.65)
    assert schedules[0].due(0.905)
    assert not schedules[0].due(0.906)


@pytest.mark.parametrize('period,offset', [
    (0.0, 0.0), (0.01, 0.0), (0.3, -0.1), (0.3, 0.3),
    (float('nan'), 0.0), (0.3, float('inf')),
])
def test_invalid_schedules_rejected(period, offset):
    with pytest.raises(ValueError):
        PingSchedule(period, offset)
