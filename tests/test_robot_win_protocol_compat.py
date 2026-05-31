import unittest

from viz_dashboard.protocol import build_read_request, crc8


class RobotWinProtocolCompatTest(unittest.TestCase):
    def test_crc8_matches_working_robot_win_implementation(self):
        self.assertEqual(crc8(bytes.fromhex("55 01 00 01")), 0xDE)

    def test_read_request_matches_working_robot_win_frame(self):
        self.assertEqual(
            build_read_request(0x00, 1),
            bytes.fromhex("55 01 00 01 DE"),
        )


if __name__ == "__main__":
    unittest.main()
