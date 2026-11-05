class ButtonRelease:
    """Acts once per press of a controller button, when it is released.

    On release rather than on press, so a button still held afterwards doesn't also trigger what it does
    next (with --motion, the right A that starts teleop also quits it while tracking).
    """

    def __init__(self):
        self.pressed = False

    def update(self, pressed):
        """Feed the button's current state; True when it was just released."""
        released = self.pressed and not pressed
        self.pressed = bool(pressed)
        return released
