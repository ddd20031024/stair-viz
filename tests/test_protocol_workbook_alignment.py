import unittest

from viz_dashboard.protocol import RegAddr


class ProtocolWorkbookAlignmentTest(unittest.TestCase):
    def test_register_addresses_match_working_robot_win_app(self):
        self.assertEqual(RegAddr.GIMBAL_DUTY, 0x11)
        self.assertEqual(RegAddr.TOF1, 0x17)
        self.assertEqual(RegAddr.IMU_QUAT_W, 0x21)
        self.assertEqual(RegAddr.IMU_PITCH, 0x26)
        self.assertEqual(RegAddr.IMU_TEMP, 0x34)
        self.assertEqual(RegAddr.MOTOR_L3_TORQUE_SPEED, 0x35)
        self.assertEqual(RegAddr.MOTOR_R2_ANGLE, 0x46)
        self.assertEqual(RegAddr.TRIWHEEL_ANGLE_CUR_FRONT, 0x47)
        self.assertEqual(RegAddr.GIMBAL_DUTY_CUR, 0x76)
        self.assertEqual(RegAddr.ONLINE_STATUS, 0x82)


if __name__ == "__main__":
    unittest.main()
