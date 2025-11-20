import sys
import math

from PySide6.QtWidgets import QApplication, QDialog
from PySide6.QtCore import Slot

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from nova2_gui import Ui_Dialog  # your generated UI file with Ui_Dialog

# Each dial controls one sliding cube
CUBE_TOPICS = [
    "/sliding_cube0/cmd_vel",
    "/sliding_cube1/cmd_vel",
    "/sliding_cube2/cmd_vel",
    "/sliding_cube3/cmd_vel",
]

# Each arm controller topic for the test arm button
ARM_TOPICS = [
    "/nova2_robot0/arm_controller/joint_trajectory",
    "/nova2_robot1/arm_controller/joint_trajectory",
    "/nova2_robot2/arm_controller/joint_trajectory",
    "/nova2_robot3/arm_controller/joint_trajectory",
]

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

# Two predefined joint position sets [joint1..joint6]
PRESET_POSITIONS = [
    [-0.00, -0.00, 0.004, -0.00, 0.00, -0.00],   # pose A (your example)
    [ 0.500, -0.50, 1.000, -2.000, 0.000,  0.50],  # pose B (adjust as you like)
]


class SlidingCubeNode(Node):
    def __init__(self):
        super().__init__("nova2_gui_node")

        # Publishers for the 4 sliding cubes (Twist)
        self.cube_publishers = []
        for topic in CUBE_TOPICS:
            pub = self.create_publisher(Twist, topic, 1)
            self.cube_publishers.append(pub)

        # Publishers for the 4 robot arms (JointTrajectory)
        self.arm_publishers = []
        for topic in ARM_TOPICS:
            pub = self.create_publisher(JointTrajectory, topic, 1)
            self.arm_publishers.append(pub)

    def send_cmd_vel(self, vectors):
        """
        vectors: list of (x, y) for each cube, already processed.
        len(vectors) should be <= len(self.cube_publishers)
        """
        for i, (vx, vy) in enumerate(vectors):
            if i >= len(self.cube_publishers):
                break
            msg = Twist()
            msg.linear.x = float(vx)
            msg.linear.y = float(vy)
            self.cube_publishers[i].publish(msg)

    def send_random_arm_trajectories(self):
        """
        Send 4 JointTrajectory messages, each with 2 fixed predefined positions.

        Each trajectory:
          joint_names: JOINT_NAMES
          points:
            - PRESET_POSITIONS[0] at t = 1.0 s
            - PRESET_POSITIONS[1] at t = 2.0 s
        """
        for i, pub in enumerate(self.arm_publishers):
            traj = JointTrajectory()
            traj.joint_names = JOINT_NAMES

            # Point 1: first predefined pose at t = 1.0s
            p1 = JointTrajectoryPoint()
            p1.positions = PRESET_POSITIONS[0]
            p1.time_from_start = Duration(sec=1, nanosec=0)
            traj.points.append(p1)

            # Point 2: second predefined pose at t = 2.0s
            p2 = JointTrajectoryPoint()
            p2.positions = PRESET_POSITIONS[1]
            p2.time_from_start = Duration(sec=2, nanosec=0)
            traj.points.append(p2)

            self.get_logger().info(
                f"Sending 2-point trajectory to robot{i}: "
                f"{PRESET_POSITIONS[0]} -> {PRESET_POSITIONS[1]}"
            )

            pub.publish(traj)


class Dialog(QDialog, Ui_Dialog):
    def __init__(self, ros_node: SlidingCubeNode):
        super().__init__()
        self.setupUi(self)

        self.ros_node = ros_node

        # Use the button: send 4 fixed arm trajectories on click
        self.pushButton_testarm.setEnabled(True)
        self.pushButton_testarm.clicked.connect(self.on_test_arm_clicked)

        # Make all dials behave like clocks: 0–359 degrees
        for d in (self.dial_1, self.dial_2, self.dial_3, self.dial_4):
            d.setMinimum(0)
            d.setMaximum(359)

        # Track whether each dial is currently being dragged (pressed)
        # If not active -> its vector is forced to (0, 0)
        self.active = {
            "left": False,
            "right": False,
            "up": False,
            "down": False,
        }

        # Map dials to directions
        # dial_1 -> left, dial_2 -> right, dial_3 -> up, dial_4 -> down
        self._connect_dial(self.dial_1, "left")
        self._connect_dial(self.dial_2, "right")
        self._connect_dial(self.dial_3, "up")
        self._connect_dial(self.dial_4, "down")

        # Initial update (all inactive -> all cubes 0,0)
        self.update_cmd_and_label()

    # ---------- Dials & signals ----------

    def _connect_dial(self, dial_widget, direction_name: str):
        # When value changes, recompute commands
        dial_widget.valueChanged.connect(lambda _v: self.on_dials_changed())

        # When user starts dragging this dial
        dial_widget.sliderPressed.connect(
            lambda: self.set_active(direction_name, True)
        )

        # When user releases this dial ("undrag")
        dial_widget.sliderReleased.connect(
            lambda: self.set_active(direction_name, False)
        )

    def set_active(self, direction_name: str, is_active: bool):
        self.active[direction_name] = is_active
        # Recompute velocities immediately
        self.update_cmd_and_label()

    @Slot()
    def on_dials_changed(self):
        # Any movement while pressed: recompute
        self.update_cmd_and_label()

    # ---------- Test arm button ----------

    @Slot()
    def on_test_arm_clicked(self):
        """
        When the button is clicked, send 4 fixed 2-point JointTrajectory messages.
        """
        self.ros_node.send_random_arm_trajectories()

    # ---------- Core logic for sliding cubes ----------

    def update_cmd_and_label(self):
        """
        For each dial:
          - If not active (not being dragged): (x, y) = (0, 0)
          - If active: convert angle -> (x, y), then snap to dominant axis:
              if |x| > |y| -> y = 0, else x = 0
        Then send 4 vectors to 4 cubes.
        """

        # Angles (deg) from dials
        angles_deg = [
            self.dial_1.value(),  # left
            self.dial_2.value(),  # right
            self.dial_3.value(),  # up
            self.dial_4.value(),  # down
        ]

        directions = ["left", "right", "up", "down"]

        raw_vectors = []
        snapped_vectors = []

        for a_deg, direction_name in zip(angles_deg, directions):
            if not self.active[direction_name]:
                # Not being dragged -> treat as zero
                raw = (0.0, 0.0)
            else:
                # Mapping: add 90°, invert y
                theta = math.radians(a_deg + 90)
                x = math.cos(theta)
                y = -math.sin(theta)
                raw = (x, y)

            raw_vectors.append(raw)

            # Snap to dominant axis: if |x| > |y| -> y = 0, else x = 0
            x, y = raw
            if abs(x) > abs(y):
                y = 0.0
            else:
                x = 0.0
            snapped_vectors.append((x, y))

        # Debug print first cube's vector
        print("raw[0]:", raw_vectors[0], "snapped[0]:", snapped_vectors[0])

        # Update label with all four
        txt_lines = []
        for i, (vx, vy) in enumerate(snapped_vectors):
            txt_lines.append(f"cube{i}: x={vx:.2f}, y={vy:.2f}")
        self.label.setText("\n".join(txt_lines))

        # Send to ROS 2
        self.ros_node.send_cmd_vel(snapped_vectors)

    # ---------- Qt close ----------

    def closeEvent(self, event):
        super().closeEvent(event)


def main():
    rclpy.init()

    ros_node = SlidingCubeNode()

    app = QApplication(sys.argv)
    dlg = Dialog(ros_node)
    dlg.show()

    # GUI loop; we only publish from callbacks, no need for rclpy.spin()
    exit_code = app.exec()

    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
