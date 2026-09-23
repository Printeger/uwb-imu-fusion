#!/usr/bin/env python3
"""Static contracts for runtime identity and opt-in ROS run logging."""

import pathlib
import re
import unittest
import xml.etree.ElementTree as ET


ROOT = pathlib.Path(__file__).resolve().parents[1]


def launch(path):
    return ET.parse(ROOT / path).getroot()


def arguments(root):
    return {element.attrib["name"]: element.attrib.get("default")
            for element in root.findall("arg")}


def command_keys(command):
    return set(re.findall(r"(?:^|\s)([A-Za-z_][A-Za-z0-9_]*):=", command))


NODE_IDENTITY = {
    "config_path", "fde_profile", "fixed_lag_epochs", "seed",
    "enable_run_logging", "run_directory", "write_global_diagnostics",
    "write_residuals", "write_timing", "output_root",
}
SIMULATOR_IDENTITY = NODE_IDENTITY | {
    "trajectory", "fault_mode", "fault_anchor_id", "fault_magnitude_m",
    "random_seed", "packet_loss_prob", "nlos_probability", "rviz",
}


class LaunchContracts(unittest.TestCase):
    def test_interactive_launches_default_to_no_run_logging(self):
        for path in ("launch/realtime.launch",
                     "launch/realtime_integrity_sim.launch",
                     "simulator/launch/realtime_integrity_stack.launch"):
            self.assertEqual(arguments(launch(path))["enable_run_logging"],
                             "false", path)

    def test_live_command_and_node_include_resolved_identity_parameters(self):
        root = launch("launch/realtime.launch")
        args = arguments(root)
        self.assertTrue(NODE_IDENTITY <= command_keys(args["execution_command"]))
        node = root.find("node")
        params = {item.attrib["name"] for item in node.findall("param")}
        self.assertTrue(NODE_IDENTITY <= params)

    def test_simulator_outer_and_stack_have_the_same_complete_identity(self):
        outer = launch("launch/realtime_integrity_sim.launch")
        stack = launch("simulator/launch/realtime_integrity_stack.launch")
        outer_command = arguments(outer)["execution_command"]
        stack_command = arguments(stack)["execution_command"]
        self.assertTrue(SIMULATOR_IDENTITY <= command_keys(outer_command))
        self.assertTrue(SIMULATOR_IDENTITY <= command_keys(stack_command))
        self.assertEqual(command_keys(outer_command), command_keys(stack_command))
        included = {item.attrib["name"]
                    for item in outer.find("include").findall("arg")}
        self.assertTrue(SIMULATOR_IDENTITY <= included)
        self.assertIn("execution_command", included)
        realtime = next(node for node in stack.findall("node")
                        if node.attrib.get("type") ==
                        "uwb_imu_pl_realtime_node")
        params = {item.attrib["name"] for item in realtime.findall("param")}
        self.assertTrue(NODE_IDENTITY <= params)

    def test_week4_command_is_complete_and_validation_logging_is_explicit(self):
        root = launch("launch/week4_topic_test.launch")
        args = arguments(root)
        expected = NODE_IDENTITY | {
            "scenario", "epochs", "rate_scale", "inventory",
        }
        self.assertEqual(args["enable_run_logging"], "true")
        self.assertTrue(expected <= command_keys(args["execution_command"]))
        self.assertNotIn("week4_topic_test $(arg scenario)",
                         args["execution_command"])


if __name__ == "__main__":
    unittest.main()
