import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import select
import tty
import termios

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.arm_publisher = self.create_publisher(JointTrajectory, '/Arm_controller/joint_trajectory', 10)
        self.joint_positions = [0.0, 0.785398, -1.5708, 0.0]  # Initialize positions for four joints
        self.joint_velocities = [0.5, 0.5, 0.5, 0.5]  # Initialize velocities for four joints
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def publish_trajectory(self, joint_positions):
        trajectory = JointTrajectory()
        trajectory.joint_names = ['Revolute 1', 'Revolute 2', 'Revolute 3', 'Revolute 4']
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.velocities = self.joint_velocities  # Set velocities
        point.time_from_start.sec = 1
        trajectory.points = [point]
        self.arm_publisher.publish(trajectory)
        self.get_logger().info(f'Published trajectory: {trajectory}')

    def listen_to_keyboard(self):
        try:
            while True:
                key = self.get_key()
                increment = 0.1  # Adjust the increment size as needed for joint movement
                velocity_increment = 0.05
                # Example controls for adjusting velocity of the first joint
                if key == '1':  # Increase velocity for all joints
                    self.joint_velocities = [v + velocity_increment for v in self.joint_velocities]
                elif key == '2':  # Decrease velocity for all joints
                    self.joint_velocities = [v - velocity_increment for v in self.joint_velocities]

                # Control for the first joint
                if key == 'q':  # Increase the first joint
                    self.joint_positions[0] += increment
                elif key == 'a':  # Decrease the first joint
                    self.joint_positions[0] -= increment

                # Control for the second joint
                elif key == 'w':
                    self.joint_positions[1] += increment
                elif key == 's':
                    self.joint_positions[1] -= increment

                # Control for the third joint
                elif key == 'e':  # Increase the third joint
                    self.joint_positions[2] += increment
                elif key == 'd':  # Decrease the third joint
                    self.joint_positions[2] -= increment

                # Control for the fourth joint
                elif key == 'r':  # Increase the fourth joint
                    self.joint_positions[3] += increment
                elif key == 'f':  # Decrease the fourth joint
                    self.joint_positions[3] -= increment

                elif key == '\x03':  # Ctrl-C to break the loop
                    break
                self.publish_trajectory(self.joint_positions)
        except Exception as e:
            self.get_logger().info('Error in keyboard listener: %s' % e)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    node.listen_to_keyboard()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
