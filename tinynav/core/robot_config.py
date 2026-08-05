"""Robot geometry shared by the planner and the controller.

Its own module rather than living in ``planning_node`` because
``tinynav/platforms/cmd_vel_control.py`` needs the same numbers, and importing
``planning_node`` to get them would pull in the whole planning stack -- the
njit-compiled ESDF and theta* kernels, and a second ``Node`` subclass -- into a
process that runs on the D-Robotics X5 inside the Looper camera (~850 MB RAM free,
no swap).  Only ``dataclasses`` and ``numpy`` here.

Sharing them is not cosmetic.  Before this module existed the camera offset was
declared twice and disagreed: ``GO2_CONFIG.camera_x = 0.2`` against a literal
``camera_offset = [0.0, 0.0, 0.35]`` in ``cmd_vel_control._control_loop``, so the
planner and the controller placed the robot's control centre 150 mm apart.
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class RobotConfig:
    """Robot geometry. Body frame: +x forward, +y left."""
    name: str = 'go2'
    shape: str = 'square'
    length: float = 0.7
    width: float = 0.3
    radius: float = 0.3
    camera_x: float = 0.35
    camera_y: float = 0.0
    control_x: float = 0.0
    control_y: float = 0.0
    safety_radius: float = 0.1

    @property
    def cam_offset_3d(self):
        """Control-center -> camera offset, expressed in *camera optical* axes.

        Consumed as ``center = T[:3, 3] - T[:3, :3] @ cam_offset_3d``, so the
        vector lives in the camera frame, whose axes are ``+x`` right, ``+y``
        down, ``+z`` forward.  ``camera_y`` is a body-frame *left* offset, so it
        enters the right-handed ``x`` slot negated.  Both previously shipped
        configs have ``camera_y = 0``, which is why the missing sign never showed
        up; ``LEKIWI_CONFIG`` has the camera 50 mm off-centre and would otherwise
        pick up a 100 mm lateral bias.
        """
        return np.array(
            [-(self.camera_y - self.control_y), 0.0, self.camera_x - self.control_x],
            dtype=np.float32,
        )

    @property
    def half_size(self):
        if self.shape == 'circle':
            return (self.radius, self.radius)
        return (self.length / 2.0, self.width / 2.0)

    def footprint_from_control(self):
        """Returns (front_len, rear_len, half_w) relative to control center."""
        hl, hw = self.half_size
        return float(hl - self.control_x), float(hl + self.control_x), float(hw)

    def describe(self) -> str:
        size = (
            f"r={self.radius}m"
            if self.shape == 'circle'
            else f"{self.length}x{self.width}m"
        )
        return (
            f"{self.name} ({self.shape} {size}, "
            f"cam=({self.camera_x},{self.camera_y}), "
            f"ctrl=({self.control_x},{self.control_y}), "
            f"safety_r={self.safety_radius}m)"
        )


GO2_CONFIG = RobotConfig(
    name='go2', shape='square',
    length=0.4, width=0.3,
    camera_x=0.2, camera_y=0.0,
    control_x=0.0, control_y=0.0,
    safety_radius=0.2,
)

B2_CONFIG = RobotConfig(
    name='b2', shape='square',
    length=1.0, width=0.5,
    camera_x=0.5, camera_y=0.0,
    control_x=-0.5, control_y=0.0,
    safety_radius=0.1,
)

# LeKiwi three-wheel omni base carrying a Looper. Round footprint because it can
# rotate in place with no swept rectangle; radius is the chassis plate, a little
# outside the 127 mm wheel-centre base_radius the odometry calibration measured.
# camera_x/camera_y are the mount measured on the robot: 60 mm ahead of the base
# centre, 50 mm to its left. Note the distance from GO2's camera_x=0.2 -- running
# LeKiwi on the GO2 config puts the control centre 140 mm behind where it really
# is, on a robot 300 mm across, and 290 mm behind it if the controller's old 0.35
# literal is what you compare against.
LEKIWI_CONFIG = RobotConfig(
    name='lekiwi', shape='circle',
    radius=0.15,
    camera_x=0.06, camera_y=0.05,
    control_x=0.0, control_y=0.0,
    safety_radius=0.1,
)

ROBOT_CONFIGS = {cfg.name: cfg for cfg in (GO2_CONFIG, B2_CONFIG, LEKIWI_CONFIG)}


def robot_config(name: str) -> RobotConfig:
    """Look a config up by name, with the known names in the error message."""
    try:
        return ROBOT_CONFIGS[name]
    except KeyError:
        raise ValueError(
            f"unknown robot_type '{name}'; known: {sorted(ROBOT_CONFIGS)}"
        ) from None
