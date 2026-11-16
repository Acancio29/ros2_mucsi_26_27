#!/usr/bin/python3
"""Nodo que graba posición, velocidad y aceleración del UR3e por trayectoria."""

import csv
from collections import deque
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from ur3e_interfaces.msg import EstadoCinematico

ARTICULACIONES_BRAZO: tuple[str, ...] = (
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
)
TAMANO_VENTANA_SUAVIZADO: int = 5
CARPETA_SALIDA: Path = Path("datos_trayectorias")


class SuscriptorArticulaciones(Node):
    """Graba los datos cinemáticos del brazo en un CSV por trayectoria."""

    def __init__(self) -> None:
        super().__init__("suscriptor_articulaciones")
        self.create_subscription(JointState, "/joint_states", self._callback, 10)
        self.create_subscription(String, "/trayectoria_actual", self._callback_trayectoria, 10)
        self._publicador_estado = self.create_publisher(EstadoCinematico, "/estado_cinematico", 10)
        self._trayectoria: str = ""
        self._reiniciar_datos()

    def _reiniciar_datos(self) -> None:
        """Vacía los datos acumulados para empezar una trayectoria nueva."""
        self._filas: list[list[float]] = []
        self._tiempo_inicial: float | None = None
        self._tiempo_anterior: float | None = None
        self._velocidad_anterior: list[float] | None = None
        self._ventana_velocidades: deque[list[float]] = deque(maxlen=TAMANO_VENTANA_SUAVIZADO)

    def _callback_trayectoria(self, mensaje: String) -> None:
        """Un nombre inicia una trayectoria; un texto vacío la finaliza."""
        self.guardar_trayectoria()
        self._trayectoria = mensaje.data.strip()
        if self._trayectoria:
            self.get_logger().info(f"Grabando trayectoria '{self._trayectoria}'")

    def guardar_trayectoria(self) -> None:
        """Escribe en un CSV la trayectoria en curso y deja de grabar."""
        if self._trayectoria and self._filas:
            CARPETA_SALIDA.mkdir(parents=True, exist_ok=True)
            ruta = CARPETA_SALIDA / f"{self._trayectoria}.csv"
            with ruta.open("w", newline="") as archivo:
                escritor = csv.writer(archivo)
                escritor.writerow(
                    ["tiempo"]
                    + [f"pos_{nombre}" for nombre in ARTICULACIONES_BRAZO]
                    + [f"vel_{nombre}" for nombre in ARTICULACIONES_BRAZO]
                    + [f"acc_{nombre}" for nombre in ARTICULACIONES_BRAZO]
                )
                escritor.writerows(self._filas)
            self.get_logger().info(f"Guardadas {len(self._filas)} muestras en {ruta}")
        self._trayectoria = ""
        self._reiniciar_datos()

    def _publicar_estado(
        self,
        sello: Time,
        posiciones: list[float],
        velocidades: list[float],
        aceleraciones: list[float],
    ) -> None:
        """Publica en un solo mensaje la posición, velocidad y aceleración calculadas."""
        estado = EstadoCinematico()
        estado.header.stamp = sello
        estado.trayectoria = self._trayectoria
        estado.nombres = list(ARTICULACIONES_BRAZO)
        estado.posiciones = posiciones
        estado.velocidades = velocidades
        estado.aceleraciones = aceleraciones
        self._publicador_estado.publish(estado)

    def _callback(self, mensaje: JointState) -> None:
        """Calcula y guarda posición, velocidad y aceleración del brazo."""
        if not self._trayectoria:
            return
        # Los mensajes de la pinza no contienen las articulaciones del brazo
        if not all(nombre in mensaje.name for nombre in ARTICULACIONES_BRAZO):
            return

        sello = mensaje.header.stamp
        tiempo = sello.sec + sello.nanosec / 1e9
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

        self._ventana_velocidades.append(lista_velocidades)
        # zip(*ventana) agrupa los valores por articulación para promediarlos
        velocidad_suavizada = [
            sum(valores) / len(valores) for valores in zip(*self._ventana_velocidades)
        ]

        if self._tiempo_anterior is None or self._velocidad_anterior is None:
            # Primer mensaje: no hay velocidad anterior con la que comparar
            lista_aceleraciones = [0.0] * len(lista_velocidades)
        else:
            incremento_tiempo = tiempo_relativo - self._tiempo_anterior
            lista_aceleraciones = [
                (velocidad - anterior) / incremento_tiempo
                for velocidad, anterior in zip(velocidad_suavizada, self._velocidad_anterior)
            ]

        self._filas.append(
            [tiempo_relativo, *lista_posiciones, *lista_velocidades, *lista_aceleraciones]
        )
        self._publicar_estado(
            mensaje.header.stamp, lista_posiciones, lista_velocidades, lista_aceleraciones
        )
        self._tiempo_anterior = tiempo_relativo
        self._velocidad_anterior = velocidad_suavizada


def main() -> None:
    """Arranca el nodo y guarda la trayectoria en curso al cerrarlo."""
    rclpy.init()
    nodo = SuscriptorArticulaciones()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.guardar_trayectoria()
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()