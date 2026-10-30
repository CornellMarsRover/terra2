import csv
import json
import math
import tempfile
import unittest
from pathlib import Path

from astronaut_autonomy.competition_simulation import (
    CompetitionSimulationConfig,
    MockRover,
    run_competition_simulation,
)
from astronaut_autonomy.tracking_control import VelocityCommand


class CompetitionSimulationTests(unittest.TestCase):
    def test_mock_rover_applies_commands_and_estop(self):
        rover = MockRover(2.0, 1.0)
        linear, angular = rover.step(
            VelocityCommand(linear_x=0.5, angular_z=0.5), 1.0
        )
        self.assertAlmostEqual(linear, 1.0)
        self.assertAlmostEqual(angular, 0.5)
        self.assertAlmostEqual(rover.pose.yaw_rad, 0.5)
        stopped_pose = (rover.pose.x, rover.pose.y, rover.pose.yaw_rad)
        linear, angular = rover.step(
            VelocityCommand(linear_x=1.0, angular_z=1.0),
            1.0,
            emergency_stop=True,
        )
        self.assertEqual((linear, angular), (0.0, 0.0))
        self.assertEqual(
            (rover.pose.x, rover.pose.y, rover.pose.yaw_rad), stopped_pose
        )

    def test_complete_closed_loop_competition_mission(self):
        report = run_competition_simulation()
        self.assertTrue(report.succeeded, report.to_dict())
        self.assertEqual(
            report.recognized_commands,
            ["FOLLOW", "STAY", "FETCH", "COME", "GIVE"],
        )
        self.assertLessEqual(report.follow_stop_distance_m, 3.0)
        self.assertGreaterEqual(report.remote_command_distance_m, 20.0)
        self.assertAlmostEqual(report.come_stop_distance_m, 1.4, delta=0.25)
        self.assertEqual(report.safety_violations, [])
        self.assertEqual(report.unexpected_commands_rejected, 1)
        self.assertEqual(
            set(report.injected_events),
            {
                "temporary_tracking_loss",
                "stale_invalid_depth",
                "emergency_stop",
            },
        )
        self.assertGreater(report.trajectory_samples, 100)

    def test_noise_is_configurable_and_mission_still_converges(self):
        report = run_competition_simulation(
            CompetitionSimulationConfig(
                range_noise_std_m=0.02,
                bearing_noise_std_deg=0.25,
                random_seed=7,
            )
        )
        self.assertTrue(report.succeeded, report.to_dict())
        self.assertTrue(math.isfinite(report.final_distance_m))

    def test_all_command_modalities_complete_closed_loop(self):
        for mode in ("gesture", "voice", "matching", "conflict"):
            with self.subTest(mode=mode):
                report = run_competition_simulation(
                    CompetitionSimulationConfig(command_input_mode=mode)
                )
                self.assertTrue(report.succeeded, report.to_dict())
                self.assertEqual(report.safety_violations, [])
                self.assertIn("temporary_tracking_loss", report.injected_events)
                if mode == "conflict":
                    self.assertTrue(
                        any(
                            transition["reason"] == "conflict_safety_stop"
                            for transition in report.state_transitions
                        )
                    )

    def test_csv_and_json_reports_are_written(self):
        with tempfile.TemporaryDirectory() as directory:
            csv_path = Path(directory) / "trajectory.csv"
            report_path = Path(directory) / "report.json"
            report = run_competition_simulation(
                csv_path=csv_path, report_path=report_path
            )
            with csv_path.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            saved = json.loads(report_path.read_text(encoding="utf-8"))
            self.assertEqual(len(rows), report.trajectory_samples)
            self.assertTrue(saved["succeeded"])
            self.assertEqual(saved["recognized_commands"], report.recognized_commands)
            unsafe_inputs = [
                row
                for row in rows
                if row["distance_valid"] == "False"
                or row["emergency_stop"] == "True"
            ]
            self.assertTrue(unsafe_inputs)
            for row in unsafe_inputs:
                self.assertEqual(float(row["applied_linear_mps"]), 0.0)
                self.assertEqual(float(row["applied_angular_rad_s"]), 0.0)

            stay_rows = [
                row
                for row in rows
                if row["phase"] == "STAY"
                and row["mission_state"] == "STAYING"
            ]
            self.assertTrue(stay_rows)
            stay_positions = {
                (round(float(row["rover_x_m"]), 8), round(float(row["rover_y_m"]), 8))
                for row in stay_rows
            }
            self.assertEqual(len(stay_positions), 1)

            come_distances = [
                float(row["distance_m"])
                for row in rows
                if row["phase"] == "COME"
                and row["mission_state"] == "COMING"
            ]
            self.assertGreater(come_distances[0], come_distances[-1])

    def test_remote_commands_must_be_at_least_twenty_meters(self):
        with self.assertRaises(ValueError):
            CompetitionSimulationConfig(remote_command_distance_m=19.9)


if __name__ == "__main__":
    unittest.main()
