"""Lanza el publicador de imágenes de la cámara USB y un visor para mostrarlas."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

NOMBRE_PAQUETE: str = "vision_camara"
# Espacio de nombres para que las imágenes se publiquen en /usb_cam/image_raw
ESPACIO_NOMBRES_CAMARA: str = "usb_cam"
TOPIC_IMAGENES: str = f"/{ESPACIO_NOMBRES_CAMARA}/image_raw"


def generate_launch_description() -> LaunchDescription:
    """Define los nodos que se arrancan: el publicador de la cámara y el visor."""
    ruta_parametros = (
        Path(get_package_share_directory(NOMBRE_PAQUETE)) / "config" / "usb_cam_params.yaml"
    )

    publicador_camara = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam",
        namespace=ESPACIO_NOMBRES_CAMARA,
        parameters=[str(ruta_parametros)],
        output="screen",
    )

    visor_imagenes = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        arguments=[TOPIC_IMAGENES],
    )

    return LaunchDescription([publicador_camara, visor_imagenes])