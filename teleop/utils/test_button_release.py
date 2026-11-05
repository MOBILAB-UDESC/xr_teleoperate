"""Tests for ButtonRelease (press and release a controller button to act once). Plain Python, no robot:

    cd ~/xr_teleoperate
    python -m unittest teleop.utils.test_button_release -v
"""

import unittest

from teleop.utils.button_release import ButtonRelease


class ButtonReleaseTest(unittest.TestCase):
    def feed(self, *states):
        button = ButtonRelease()
        return [button.update(s) for s in states]

    def test_not_pressed(self):
        self.assertEqual(self.feed(False, False, False), [False, False, False])

    def test_fires_on_release_not_on_press(self):
        self.assertEqual(self.feed(False, True, True, False), [False, False, False, True])

    def test_fires_once_per_press(self):
        self.assertEqual(self.feed(True, False, False, True, False), [False, True, False, False, True])

    def test_held_does_not_fire(self):
        self.assertEqual(self.feed(True, True, True), [False, False, False])


if __name__ == "__main__":
    unittest.main()
