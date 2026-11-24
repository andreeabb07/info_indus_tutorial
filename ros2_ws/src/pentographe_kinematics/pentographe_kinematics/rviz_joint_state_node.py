"""Nœud ROS 2 : publie les 4 angles absolus des barres pour la simulation RViz.

Entrée : topic 'pen_target'       (geometry_msgs/Point, en METRES ; z ignoré)
Sortie : topic 'pantograph_angles' (sensor_msgs/JointState)
         orientations absolues theta1, theta2, theta3, theta4 (radians),
         dans le repère mathématique XY.

Ce nœud fournit TOUS les angles du mécanisme (les 2 bras moteurs et les 2
avant-bras), à partir des mêmes équations que le nœud de pilotage. Le nœud de
visualisation (joint_bridge, côté URDF) les convertit ensuite dans le repère de
l'URDF ; il n'a plus besoin de recalculer le point de fermeture A3.

Les dimensions viennent de son fichier de paramètres, réglé par défaut sur les
valeurs du modèle Creo (celles des meshes de l'URDF).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState

from pentographe_kinematics import kinematics as k


class RvizJointStateNode(Node):
    def __init__(self):
        super().__init__('pentographe_rviz_joint_state')

        d = k.PantographParams()
        for name, val in (('l0', d.l0), ('l1', d.l1), ('l2', d.l2),
                          ('l3', d.l3), ('l4', d.l4),
                          ('e_par', d.e_par), ('e_perp', d.e_perp)):
            self.declare_parameter(name, val)

        self._sub = self.create_subscription(
            Point, 'pen_target', self._on_target, 10)
        self._pub = self.create_publisher(
            JointState, 'pantograph_angles', 10)
        self.get_logger().info('Nœud RViz (4 angles absolus) démarré.')

    def _params(self):
        g = lambda n: self.get_parameter(n).get_parameter_value().double_value
        return k.PantographParams(g('l0'), g('l1'), g('l2'), g('l3'),
                                  g('l4'), g('e_par'), g('e_perp'))

    def _on_target(self, msg: Point):
        x_mm, y_mm = msg.x * 1000.0, msg.y * 1000.0
        try:
            a = k.all_joint_angles(x_mm, y_mm, self._params())
        except k.Unreachable:
            self.get_logger().warn(
                f'Point ({x_mm:.1f}, {y_mm:.1f}) mm hors de l\'espace de travail.')
            return
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = ['theta1', 'theta2', 'theta3', 'theta4']
        out.position = [a['theta1'], a['theta2'], a['theta3'], a['theta4']]
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = RvizJointStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
