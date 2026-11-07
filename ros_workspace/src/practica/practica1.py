#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class SuscriptorArticulaciones(Node):
    """Nodo que recibe y muestra el estado de las articulaciones."""

    def __init__(self) -> None:
        super().__init__("suscriptor_articulaciones")
        self.create_subscription(JointState, "/joint_states", self._callback, 10)

    def _callback(self, mensaje: JointState) -> None:
        """Muestra el tiempo, la posición y la velocidad de cada mensaje."""
        sello = mensaje.header.stamp
        tiempo = sello.sec + sello.nanosec / 1e9
        posiciones = dict(zip(mensaje.name, mensaje.position))
        velocidades = dict(zip(mensaje.name, mensaje.velocity))
        print(tiempo, posiciones, velocidades)


if __name__ == '__main__':
    rclpy.init()
    nodo = SuscriptorArticulaciones()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    nodo.destroy_node()
    rclpy.try_shutdown()