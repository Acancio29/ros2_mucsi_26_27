#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from collections import deque

TAMANO_VENTANA_SUAVIZADO: int = 5
TOPIC_ESTADO_ARTICULACIONES: str = "/joint_states"
TAMANO_COLA: int = 10
NANOSEGUNDOS_POR_SEGUNDO: float = 1e9
ARTICULACIONES_BRAZO: tuple[str, ...] = (
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
)


class SuscriptorArticulaciones(Node):
    """Nodo que recibe y muestra el estado de las articulaciones del brazo."""

    def __init__(self) -> None:
        """Crea el nodo y su suscripción al estado de las articulaciones."""
        super().__init__("suscriptor_articulaciones")
        self.create_subscription(
            JointState, TOPIC_ESTADO_ARTICULACIONES, self._callback, TAMANO_COLA
        )
        self._filas: list[list[float]] = []
        self._tiempo_inicial: float | None = None
        self._tiempo_anterior: float | None = None
        self._velocidad_anterior: list[float] | None = None
        self._ventana_velocidades: deque[list[float]] = deque(maxlen=TAMANO_VENTANA_SUAVIZADO)

    def _callback(self, mensaje: JointState) -> None:
        """Lee las posiciones y velocidades del brazo en cada mensaje."""
        # Los mensajes de la pinza no contienen las articulaciones del brazo
        if not all(nombre in mensaje.name for nombre in ARTICULACIONES_BRAZO):
            return

        sello = mensaje.header.stamp
        tiempo = sello.sec + sello.nanosec / NANOSEGUNDOS_POR_SEGUNDO
        # Se indexa por nombre porque el orden de las articulaciones puede variar
        posiciones = dict(zip(mensaje.name, mensaje.position))
        velocidades = dict(zip(mensaje.name, mensaje.velocity))
        # Listas siempre en el mismo orden, para que cada columna sea la misma articulación
        lista_posiciones = [posiciones[nombre] for nombre in ARTICULACIONES_BRAZO]
        lista_velocidades = [velocidades[nombre] for nombre in ARTICULACIONES_BRAZO]

        if self._tiempo_inicial is None:
            self._tiempo_inicial = tiempo
        tiempo_relativo = tiempo - self._tiempo_inicial

        # Un mensaje con un tiempo que no avanza daría una división entre 0 o negativa
        if self._tiempo_anterior is not None and tiempo_relativo <= self._tiempo_anterior:
            return

        if self._tiempo_anterior is None or self._velocidad_anterior is None:
            # Primer mensaje: no hay velocidad anterior con la que comparar
            lista_aceleraciones = [0.0] * len(lista_velocidades)
        else:
            incremento_tiempo = tiempo_relativo - self._tiempo_anterior
            lista_aceleraciones = [
                (velocidad - anterior) / incremento_tiempo
                for velocidad, anterior in zip(lista_velocidades, self._velocidad_anterior)
            ]

        self._filas.append(
            [tiempo_relativo, *lista_posiciones, *lista_velocidades, *lista_aceleraciones]
        )
        self._tiempo_anterior = tiempo_relativo
        self._velocidad_anterior = lista_velocidades


def main() -> None:
    """Arranca ROS 2 y mantiene el nodo escuchando hasta pulsar Ctrl+C."""
    rclpy.init()
    nodo = SuscriptorArticulaciones()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()