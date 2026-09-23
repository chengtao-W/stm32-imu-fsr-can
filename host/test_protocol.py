import unittest

from protocol import (
    FootSample,
    ImuSample,
    MSG_SENSOR_STATE,
    SensorSample,
    StreamParser,
    decode_sensor_payload,
    encode_sensor_payload,
    make_frame,
)


def sample() -> SensorSample:
    imu1 = ImuSample((1.0, 2.0, 3.0), (4.0, 5.0, 6.0), (7.0, 8.0, 9.0), (1.0, 0.0, 0.0, 0.0))
    imu2 = ImuSample((-1.0, -2.0, -3.0), (-4.0, -5.0, -6.0), (-7.0, -8.0, -9.0), (0.5, 0.5, 0.5, 0.5))
    left = FootSample(2, 20, 190, tuple(range(20)))
    right = FootSample(3, 20, 590, tuple(range(20, 40)))
    return SensorSample(123456, imu1, imu2, left, right)


class ProtocolTest(unittest.TestCase):
    def test_payload_round_trip(self):
        expected = sample()
        self.assertEqual(decode_sensor_payload(encode_sensor_payload(expected)), expected)

    def test_fragmented_stream(self):
        expected = sample()
        frame = make_frame(MSG_SENSOR_STATE, encode_sensor_payload(expected))
        parser = StreamParser()
        messages = []
        for index in range(0, len(frame), 8):
            messages.extend(parser.feed(frame[index:index + 8]))
        self.assertEqual(len(messages), 1)
        self.assertEqual(messages[0][0], MSG_SENSOR_STATE)
        self.assertEqual(decode_sensor_payload(messages[0][1]), expected)
        self.assertEqual(parser.frames_ok, 1)

    def test_crc_error_resynchronizes(self):
        expected = sample()
        good = make_frame(MSG_SENSOR_STATE, encode_sensor_payload(expected))
        bad = bytearray(good)
        bad[30] ^= 0x80
        parser = StreamParser()
        messages = parser.feed(b"noise" + bad + good)
        self.assertEqual(len(messages), 1)
        self.assertEqual(decode_sensor_payload(messages[0][1]), expected)
        self.assertGreaterEqual(parser.crc_errors, 1)


if __name__ == "__main__":
    unittest.main()
