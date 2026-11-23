import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointBridge(Node):

    def __init__(self):
        super().__init__('joint_bridge')

        self.subscription = self.create_subscription(
            JointState,
            '/joint_commands',
            self.callback,
            10
        )

        self.publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # Dimensions de la CAO (mm)
        self.l0 = 138.0
        self.l1 = 80.0
        self.l2 = 157.0
        self.l3 = 147.0
        self.l4 = 79.0

        # Coordonnees des articulations dans le repere Creo
        # Plan XZ, en millimetres
        self.MG = (-80.0, 70.0)
        self.MD = (58.0, 70.0)

        self.CG = (-80.0, -10.0)
        self.CD = (90.2, -2.1)

        # Point de jonction mesure dans Creo
        self.P = (19.834, -131.7)

        # Orientations initiales des bras moteurs
        self.alpha1 = math.atan2(
            self.CG[1] - self.MG[1],
            self.CG[0] - self.MG[0]
        )

        self.alpha4 = math.atan2(
            self.CD[1] - self.MD[1],
            self.CD[0] - self.MD[0]
        )

        # Orientations initiales des avant-bras
        self.alpha2 = math.atan2(
            self.P[1] - self.CG[1],
            self.P[0] - self.CG[0]
        )

        self.alpha3 = math.atan2(
            self.P[1] - self.CD[1],
            self.P[0] - self.CD[0]
        )

        self.get_logger().info(
            'Joint bridge demarre - fermeture pantographe'
        )

    @staticmethod
    def normaliser(angle):
        return math.atan2(
            math.sin(angle),
            math.cos(angle)
        )

    def callback(self, msg):

        if 'q1' not in msg.name or 'q4' not in msg.name:
            return

        q1 = msg.position[msg.name.index('q1')]
        q4 = msg.position[msg.name.index('q4')]

        l0 = self.l0
        l1 = self.l1
        l2 = self.l2
        l3 = self.l3
        l4 = self.l4

        # ----------------------------------------
        # 1. Position des coudes
        # Repere mathematique XY d'Andreea
        # ----------------------------------------

        A2 = (
            l1 * math.cos(q1),
            l1 * math.sin(q1)
        )

        A4 = (
            l0 + l4 * math.cos(q4),
            l4 * math.sin(q4)
        )

        # ----------------------------------------
        # 2. Calcul du point de fermeture A3
        # ----------------------------------------

        dx = A4[0] - A2[0]
        dy = A4[1] - A2[1]

        d = math.hypot(dx, dy)

        if (
            d < 1e-9
            or d > l2 + l3
            or d < abs(l2 - l3)
        ):
            self.get_logger().warn(
                'Configuration inaccessible'
            )
            return

        a = (
            l2**2 - l3**2 + d**2
        ) / (2.0 * d)

        h = math.sqrt(
            max(0.0, l2**2 - a**2)
        )

        ux = dx / d
        uy = dy / d

        A3 = (
            A2[0] + a * ux - h * uy,
            A2[1] + a * uy + h * ux
        )

        # ----------------------------------------
        # 3. Orientations absolues
        # ----------------------------------------

        theta1 = q1
        theta4 = q4

        theta2 = math.atan2(
            A3[1] - A2[1],
            A3[0] - A2[0]
        )

        theta3 = math.atan2(
            A3[1] - A4[1],
            A3[0] - A4[0]
        )

        # ----------------------------------------
        # 4. Angles moteurs dans l'URDF
        #
        # Une rotation positive autour de Y
        # diminue l'angle dans le plan XZ.
        # ----------------------------------------

        moteur_gauche = self.normaliser(
            self.alpha1 - theta1
        )

        moteur_droit = self.normaliser(
            self.alpha4 - theta4
        )

        # ----------------------------------------
        # 5. Angles des coudes
        #
        # On tient compte de l'orientation
        # initiale des avant-bras dans Creo.
        # ----------------------------------------

        coude_gauche = self.normaliser(
            self.alpha2 - theta2 - moteur_gauche
        )

        coude_droit = self.normaliser(
            self.alpha3 - theta3 - moteur_droit
        )

        # ----------------------------------------
        # 6. Publication des articulations
        # ----------------------------------------

        sortie = JointState()

        sortie.header.stamp = (
            self.get_clock().now().to_msg()
        )

        sortie.name = [
            'joint_moteur_gauche',
            'joint_coude_gauche',
            'joint_moteur_droit',
            'joint_coude_droit'
        ]

        sortie.position = [
            moteur_gauche,
            coude_gauche,
            moteur_droit,
            coude_droit
        ]

        self.publisher.publish(sortie)

        self.get_logger().info(
            f'Point de fermeture : '
            f'X={A3[0]:.1f} mm, '
            f'Y={A3[1]:.1f} mm'
        )


def main(args=None):

    rclpy.init(args=args)

    node = JointBridge()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
