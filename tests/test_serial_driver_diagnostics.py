import time
import unittest

from viz_dashboard.serial_driver import SerialDriver


class SerialDriverDiagnosticsTest(unittest.TestCase):
    def test_mock_driver_reports_polling_diagnostics(self):
        driver = SerialDriver()
        try:
            driver.start()
            deadline = time.time() + 1.0
            diagnostics = {}
            while time.time() < deadline:
                diagnostics = driver.diagnostics()
                if diagnostics["total_responses"] > 0:
                    break
                time.sleep(0.02)

            self.assertTrue(diagnostics["connected"])
            self.assertGreater(diagnostics["total_responses"], 0)
            self.assertEqual(diagnostics["crc_errors"], 0)
            self.assertIsNotNone(diagnostics["last_success"])
            self.assertIn("reg_addr", diagnostics["last_success"])
            self.assertIn("name", diagnostics["last_success"])
        finally:
            driver.stop()


if __name__ == "__main__":
    unittest.main()
