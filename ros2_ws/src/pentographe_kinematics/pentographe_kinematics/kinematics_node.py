"""Nœud ROS 2 : reçoit une position cible du crayon, publie les angles moteurs.

Entrée  : topic 'pen_target'     (geometry_msgs/Point, en METRES ; z ignoré)
Sortie  : topic 'joint_commands' (sensor_msgs/JointState, angles q1 et q4 en radians)

Ce nœud est une coquille : tout le calcul est dans kinematics.py.
Les dimensions du robot sont des paramètres ROS, modifiables sans toucher au code.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState

from pentographe_kinematics import kinematics as k


class KinematicsNode(Node):
    def __init__(self):
        super().__init__('pentographe_kinematics')

        # Dimensions en millimètres, déclarées comme paramètres ROS
        d = k.PantographParams()
        self.declare_parameter('l0', d.l0)
        self.declare_parameter('l1', d.l1)
        self.declare_parameter('l2', d.l2)
        self.declare_parameter('l3', d.l3)
        self.declare_parameter('l4', d.l4)
        self.declare_parameter('e_par', d.e_par)
        self.declare_parameter('e_perp', d.e_perp)

        self._sub = self.create_subscription(
            Point, 'pen_target', self._on_target, 10)
        self._pub = self.create_publisher(
            JointState, 'joint_commands', 10)
        self.get_logger().info('Nœud cinématique du pantographe démarré.')

    def _params(self):
        g = lambda n: self.get_parameter(n).get_parameter_value().double_value
        return k.PantographParams(g('l0'), g('l1'), g('l2'), g('l3'),
                                  g('l4'), g('e_par'), g('e_perp'))

    def _on_target(self, msg: Point):
        # le message est en mètres, nos équations en millimètres
        x_mm, y_mm = msg.x * 1000.0, msg.y * 1000.0
        try:
            q1, q4 = k.inverse(x_mm, y_mm, self._params())
        except k.Unreachable:
            self.get_logger().warn(
                f'Point ({x_mm:.1f}, {y_mm:.1f}) mm hors de l\'espace de travail.')
            return
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = ['q1', 'q4']
        out.position = [q1, q4]
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = KinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
