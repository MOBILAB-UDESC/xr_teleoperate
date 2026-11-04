"""Tests for Dex1_1_Gripper_Controller with one, two or no grippers.

Each scenario runs the real controller in its own process against fake grippers that publish
rt/dex1/<side>/state, on a private DDS domain bound to the loopback interface, so a robot's real
grippers (domain 0) are never touched. Needs the xr_teleoperate environment (unitree_sdk2py,
logging_mp, dex_retargeting), e.g. the `tv` conda env on the robot:

    conda activate tv
    cd ~/xr_teleoperate
    python -m unittest teleop.robot_control.test_dex1_gripper_controller -v
"""

import os
import subprocess
import sys
import time
import unittest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DDS_DOMAIN = 9  # not 0 (real robot) nor 1 (Unitree simulation)
STATE_TIMEOUT_S = 1.0  # shortened Dex1_State_Timeout for the tests
SCENARIO_DEADLINE_S = 30.0  # a scenario that runs longer is stuck (e.g. waiting for a missing gripper)
Q_START = 2.7  # fake gripper position, rad
DEX1_Q_OPEN = 5.4


def run_scenario(present_sides):
    """Child process: fake grippers on present_sides, then start the controller and report."""
    from multiprocessing import Value

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorState_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_

    ChannelFactoryInitialize(DDS_DOMAIN, "lo")
    sys.path.insert(0, REPO_ROOT)
    from teleop.robot_control import robot_hand_unitree as rhu
    rhu.Dex1_State_Timeout = STATE_TIMEOUT_S

    received = {"left": [], "right": []}  # commanded q per side

    def on_cmd(side):
        return lambda msg: received[side].append(msg.cmds[0].q)

    cmd_subs = []
    for side in ("left", "right"):
        sub = ChannelSubscriber(f"rt/dex1/{side}/cmd", MotorCmds_)
        sub.Init(on_cmd(side), 10)
        cmd_subs.append(sub)

    state_pubs = []
    for side in present_sides:
        pub = ChannelPublisher(f"rt/dex1/{side}/state", MotorStates_)
        pub.Init()
        state_pubs.append(pub)
    state = MotorStates_(states=[unitree_go_msg_dds__MotorState_()])
    state.states[0].q = Q_START

    import threading

    def publish_state():
        while True:
            for pub in state_pubs:
                pub.Write(state)
            time.sleep(0.005)

    threading.Thread(target=publish_state, daemon=True).start()

    released = 10.0  # trigger released / hand open: the controller opens the gripper
    left_in, right_in = Value("d", released, lock=True), Value("d", released, lock=True)
    ready = Value("b", True, lock=True)
    try:
        rhu.Dex1_1_Gripper_Controller(left_in, right_in, xr_motion_data_ready_in=ready)
    except RuntimeError as e:
        print(f"RESULT error {e}", flush=True)
        return
    time.sleep(0.5)
    for side in ("left", "right"):
        cmds = received[side]
        print(f"RESULT {side} count={len(cmds)} last={cmds[-1] if cmds else 'none'}", flush=True)


class Dex1GripperControllerTest(unittest.TestCase):
    def scenario(self, *present_sides):
        """Run one scenario in a child process; returns {side: (count, last_q)} or {'error': msg}."""
        try:
            proc = subprocess.run(
                [sys.executable, os.path.abspath(__file__), "--scenario", ",".join(present_sides)],
                capture_output=True, text=True, timeout=SCENARIO_DEADLINE_S, cwd=REPO_ROOT,
            )
        except subprocess.TimeoutExpired:
            self.fail(f"controller still starting after {SCENARIO_DEADLINE_S:.0f} s with grippers {present_sides}")
        results = {}
        for line in proc.stdout.splitlines():
            if not line.startswith("RESULT "):
                continue
            kind, rest = line[len("RESULT "):].split(" ", 1)
            if kind == "error":
                results["error"] = rest
            else:
                fields = dict(f.split("=") for f in rest.split())
                last = None if fields["last"] == "none" else float(fields["last"])
                results[kind] = (int(fields["count"]), last)
        if not results:
            self.fail(f"scenario printed no result (exit {proc.returncode}):\n{proc.stdout}\n{proc.stderr}")
        return results

    def assert_commanded_open(self, results, side):
        count, last = results[side]
        self.assertGreater(count, 0, f"no commands sent to the {side} gripper")
        self.assertGreater(last, Q_START, f"{side} gripper not commanded towards open")
        self.assertLessEqual(last, DEX1_Q_OPEN)

    def test_both_grippers(self):
        results = self.scenario("left", "right")
        self.assert_commanded_open(results, "left")
        self.assert_commanded_open(results, "right")

    def test_right_gripper_only(self):
        results = self.scenario("right")
        self.assert_commanded_open(results, "right")
        self.assertEqual(results["left"][0], 0, "commands sent to the missing left gripper")

    def test_left_gripper_only(self):
        results = self.scenario("left")
        self.assert_commanded_open(results, "left")
        self.assertEqual(results["right"][0], 0, "commands sent to the missing right gripper")

    def test_no_gripper_fails(self):
        results = self.scenario()
        self.assertIn("error", results)
        self.assertIn("rt/dex1/left/state", results["error"])


if __name__ == "__main__":
    if len(sys.argv) == 3 and sys.argv[1] == "--scenario":
        run_scenario([s for s in sys.argv[2].split(",") if s])
    else:
        unittest.main()
