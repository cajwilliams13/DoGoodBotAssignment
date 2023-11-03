import logging
import os
import threading

import spatialgeometry as geometry
from pyjoystick.sdl2 import Joystick, Key, run_event_loop
from spatialmath import SE3


class EStop(geometry.Mesh):
    def __init__(self, initial_pose=SE3(0, 0, 0), use_physical_button=False, queue=None):
        self._e_stop_state = False
        self._queue = queue
        full_path = os.path.realpath(__file__)
        directory = os.path.dirname(full_path)
        estop_stl = os.path.join(directory, 'e_stop.stl')
        super().__init__(estop_stl, pose=initial_pose, color=(0.5, 0, 0, 1))
        if use_physical_button:
            # Should spawn this in a new thread
            thread = threading.Thread(target=run_event_loop,
                                      args=(self._added_handler, self._removed_handler, self._key_received_handler))
            thread.start()

    def _added_handler(self, joy):
        logging.debug('Added %s', joy)

    def _removed_handler(self, joy):
        logging.debug('Removed %s', joy)

    def _key_received_handler(self, key: Key):
        if key.keytype == Key.BUTTON and key.number == 0:
            if self._e_stop_state == False:
                self._e_stop_state = True
                if self._queue:
                    self._queue.put("E-Stop Pressed")
                logging.debug("E-Stop Pressed")

    def press(self):
        """Manually press the E-Stop button"""
        self._e_stop_state = True

    def release(self):
        """Manually release the E-Stop button"""
        self._e_stop_state = False

    def get_state(self):
        """Get the current state of the E-Stop button"""
        return self._e_stop_state

    def add_to_env(self, env):
        env.add(self)
