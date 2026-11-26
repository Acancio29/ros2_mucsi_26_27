#!/usr/bin/python3
"""Nodo que lanza trayectorias fijas del UR3e eligiéndolas con el teclado.

Teclas:
    1 -> trayectoria Joint     (movimiento en el espacio de articulaciones)
    2 -> trayectoria Lineal    (la punta del robot va en línea recta)
    3 -> trayectoria Circular  (la punta del robot dibuja una circunferencia)
    q -> salir

Cada trayectoria es siempre la misma: primero el robot vuelve a una posición
"casa" (sin grabar) y desde ahí ejecuta el movimiento, que sí se graba.
"""

import copy
import sys
import termios
import time
import tty
from math import cos, pi, sin
from typing import Callable, Dict, List, Tuple

import rclpy
from geometry_msgs.msg import Pose
from std_msgs.msg import String

from ur3e_cinematica.control_robot import ControlRobot

# --- Topic de comunicación con el suscriptor que graba los datos ---
TOPIC_TRAYECTORIA_ACTUAL: str = "/trayectoria_actual"
CALIDAD_QOS: int = 10
TIMEOUT_SUSCRIPTOR_S: float = 5.0
PERIODO_ESPERA_S: float = 0.1

# --- Pausas para que el suscriptor no pierda el principio/final del movimiento ---
PAUSA_INICIO_S: float = 0.5
PAUSA_FINAL_S: float = 0.5

# --- Teclas ---
TECLA_JOINT: str = "1"
TECLA_LINEAL: str = "2"
TECLA_CIRCULAR: str = "3"
TECLA_SALIR: str = "q"
TECLA_CTRL_C: str = "\x03"

# --- Nombres de trayectoria (también son los nombres de los CSV generados) ---
NOMBRE_JOINT: str = "joint"
NOMBRE_LINEAL: str = "lineal"
NOMBRE_CIRCULAR: str = "circular"
TRAYECTORIA_VACIA: str = ""  # Un texto vacío le indica al suscriptor "fin de trayectoria"

# --- Posición "casa" estándar del UR (brazo en alto, herramienta hacia abajo) ---
POSICION_CASA: List[float] = [0.0, -pi / 2, pi / 2, -pi / 2, -pi / 2, 0.0]

# --- Trayectoria JOINT: ángulos objetivo de las 6 articulaciones (rad) ---
OBJETIVO_JOINT: List[float] = [pi / 4, -1.2, 1.3, -1.7, -pi / 2, pi / 2]

# --- Trayectoria LINEAL: desplazamiento de la punta respecto a la posición casa (m) ---
DESPLAZAMIENTO_LINEAL_Y_M: float = 0.10

# --- Trayectoria CIRCULAR ---
RADIO_CIRCULO_M: float = 0.04
NUM_PUNTOS_CIRCULO: int = 36  # Más puntos = círculo más suave (36 -> un punto cada 10 grados)
ANGULO_INICIAL_CIRCULO_RAD: float = -pi / 2  # La punta empieza en el punto "de abajo" del círculo


def leer_tecla() -> str:
    """Lee una tecla sin necesidad de pulsar Enter.

    Se usa el modo cbreak de la terminal: entrega cada carácter al instante
    y restaura los ajustes originales al terminar.
    """
    descriptor = sys.stdin.fileno()
    ajustes_previos = termios.tcgetattr(descriptor)
    try:
        tty.setcbreak(descriptor)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(descriptor, termios.TCSADRAIN, ajustes_previos)


class EjecutorTrayectorias:
    """Ejecuta trayectorias fijas y avisa al suscriptor de cuándo empiezan y acaban."""

    def __init__(self, robot: ControlRobot) -> None:
        """Guarda el robot, crea el publicador y asocia cada tecla a su trayectoria.

        Args:
            robot: controlador del UR3e ya inicializado (MoveIt listo).
        """
        self._robot: ControlRobot = robot
        self._publicador = robot.create_publisher(String, TOPIC_TRAYECTORIA_ACTUAL, CALIDAD_QOS)

        # tecla -> (nombre de la trayectoria, función que la ejecuta)
        self._trayectorias: Dict[str, Tuple[str, Callable[[], bool]]] = {
            TECLA_JOINT: (NOMBRE_JOINT, self._trayectoria_joint),
            TECLA_LINEAL: (NOMBRE_LINEAL, self._trayectoria_lineal),
            TECLA_CIRCULAR: (NOMBRE_CIRCULAR, self._trayectoria_circular),
        }
        self._esperar_suscriptor()

    # ------------------------------------------------------------------
    # Bucle principal
    # ------------------------------------------------------------------
    def ejecutar_menu(self) -> None:
        """Muestra el menú y lanza la trayectoria correspondiente a cada tecla."""
        self._mostrar_menu()
        while True:
            tecla = leer_tecla().lower()
            if tecla in (TECLA_SALIR, TECLA_CTRL_C):
                self._robot.get_logger().info("Saliendo.")
                return
            if tecla in self._trayectorias:
                nombre, movimiento = self._trayectorias[tecla]
                self._ejecutar_trayectoria(nombre, movimiento)
                self._mostrar_menu()

    def _mostrar_menu(self) -> None:
        """Imprime las teclas disponibles."""
        print(
            f"\n[{TECLA_JOINT}] Joint   [{TECLA_LINEAL}] Lineal   "
            f"[{TECLA_CIRCULAR}] Circular   [{TECLA_SALIR}] Salir\n> ",
            end="", flush=True,
        )

    def _ejecutar_trayectoria(self, nombre: str, movimiento: Callable[[], bool]) -> None:
        """Lleva el robot a casa, graba el movimiento y guarda el resultado.

        Args:
            nombre: nombre de la trayectoria (será el nombre del CSV).
            movimiento: función que ejecuta el movimiento y devuelve si tuvo éxito.
        """
        logger = self._robot.get_logger()

        # 1) Punto de partida siempre igual -> trayectorias repetibles (no se graba)
        logger.info("Volviendo a la posición casa...")
        if not self._robot.mover_articulaciones(POSICION_CASA):
            logger.error("No se pudo volver a la posición casa; trayectoria cancelada.")
            return

        # 2) Avisar al suscriptor: empieza a grabar
        self._publicar_trayectoria(nombre)
        time.sleep(PAUSA_INICIO_S)

        # 3) Ejecutar el movimiento. try/finally asegura que SIEMPRE se avisa del fin
        try:
            logger.info(f"Ejecutando trayectoria '{nombre}'...")
            exito = movimiento()
            if exito:
                logger.info(f"Trayectoria '{nombre}' completada.")
            else:
                logger.error(f"MoveIt no pudo ejecutar la trayectoria '{nombre}'.")
        finally:
            # 4) Pequeña pausa para no perder las últimas muestras y cerrar el CSV
            time.sleep(PAUSA_FINAL_S)
            self._publicar_trayectoria(TRAYECTORIA_VACIA)

    # ------------------------------------------------------------------
    # Comunicación con el suscriptor
    # ------------------------------------------------------------------
    def _publicar_trayectoria(self, nombre: str) -> None:
        """Publica el nombre de la trayectoria en curso ("" = fin)."""
        self._publicador.publish(String(data=nombre))

    def _esperar_suscriptor(self) -> None:
        """Espera a que el nodo grabador esté conectado para no perder mensajes."""
        logger = self._robot.get_logger()
        limite = time.monotonic() + TIMEOUT_SUSCRIPTOR_S
        while self._publicador.get_subscription_count() == 0:
            if time.monotonic() > limite:
                logger.warn(
                    f"Nadie escucha en {TOPIC_TRAYECTORIA_ACTUAL}. "
                    "¿Está arrancado suscriptor_articulaciones?"
                )
                return
            time.sleep(PERIODO_ESPERA_S)

    # ------------------------------------------------------------------
    # Las tres trayectorias
    # ------------------------------------------------------------------
    def _trayectoria_joint(self) -> bool:
        """Movimiento Joint: cada articulación va directa a su ángulo objetivo."""
        return self._robot.mover_articulaciones(OBJETIVO_JOINT)

    def _trayectoria_lineal(self) -> bool:
        """Movimiento Lineal: la punta avanza en línea recta en el eje Y."""
        pose_final = copy.deepcopy(self._robot.pose_actual())
        pose_final.position.y += DESPLAZAMIENTO_LINEAL_Y_M
        return self._robot.mover_trayectoria([pose_final])

    def _trayectoria_circular(self) -> bool:
        """Movimiento Circular: la punta recorre una circunferencia horizontal.

        MoveIt no tiene una primitiva "círculo", así que se aproxima con muchos
        puntos intermedios (waypoints) unidos por tramos rectos muy cortos.
        """
        return self._robot.mover_trayectoria(self._generar_waypoints_circulo())

    def _generar_waypoints_circulo(self) -> List[Pose]:
        """Calcula los puntos de una circunferencia que pasa por la pose actual.

        El círculo está en un plano horizontal (z constante) y la orientación
        de la herramienta no cambia. El último punto coincide con el primero,
        por lo que la trayectoria se cierra.
        """
        pose_inicial = self._robot.pose_actual()
        # El centro se coloca para que la posición actual sea un punto del círculo
        centro_x = pose_inicial.position.x - RADIO_CIRCULO_M * cos(ANGULO_INICIAL_CIRCULO_RAD)
        centro_y = pose_inicial.position.y - RADIO_CIRCULO_M * sin(ANGULO_INICIAL_CIRCULO_RAD)

        waypoints: List[Pose] = []
        for indice in range(1, NUM_PUNTOS_CIRCULO + 1):
            angulo = ANGULO_INICIAL_CIRCULO_RAD + 2.0 * pi * indice / NUM_PUNTOS_CIRCULO
            pose = copy.deepcopy(pose_inicial)
            pose.position.x = centro_x + RADIO_CIRCULO_M * cos(angulo)
            pose.position.y = centro_y + RADIO_CIRCULO_M * sin(angulo)
            waypoints.append(pose)
        return waypoints


def main() -> None:
    """Arranca ROS 2, crea el robot y abre el menú de teclado."""
    rclpy.init()
    robot = None
    try:
        robot = ControlRobot()
        EjecutorTrayectorias(robot).ejecutar_menu()
    except KeyboardInterrupt:
        pass
    finally:
        if robot is not None:
            robot.ejecutor.shutdown()
            robot.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
