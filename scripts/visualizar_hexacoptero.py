#!/usr/bin/env python3
"""Visualiza el STL CAD mediante la pose de Simulink (ROS 2 Jazzy).

Guardar en scripts/visualizar_hexacoptero.py dentro del repositorio.
Guardar el STL reexportado, en metros y sin traslado al espacio positivo,
en meshes/hexacoptero.stl. Ejecutar desde Ubuntu/WSL con ROS 2 cargado:

    python3 scripts/visualizar_hexacoptero.py

RViz: Fixed Frame = world; Add > Marker; Topic = /hexacopter/mesh.
Agregar otro Marker con Topic = /hexacopter/trail para la trayectoria recorrida.
Para seguimiento de camara: Views > Target Frame = hexacopter.
El programa no modifica el STL ni el controlador y no calcula dinamica.
Supuesto: PoseStamped representa el centro de masa y la actitud del cuerpo.
CAD -> cuerpo: (x, y, z) -> (-x, z, y), despues de restar el CM CAD.
CM procedente de las propiedades CAD reportadas para el ensamble actualizado.
Si se reexporta con otro origen, deben actualizarse estos parametros.
"""

import math
from pathlib import Path


def multiply(a, b):
    """Producto de cuaterniones en orden (x, y, z, w)."""
    x, y, z, w = a
    X, Y, Z, W = b
    return (w*X + x*W + y*Z - z*Y,
            w*Y - x*Z + y*W + z*X,
            w*Z + x*Y - y*X + z*W,
            w*W - x*X - y*Y - z*Z)


def rotate(q, v):
    return multiply(multiply(q, (*v, 0.0)),
                    (-q[0], -q[1], -q[2], q[3]))[:3]


def mesh_pose(position, quaternion, cad_cm):
    """Compone T_world_body * T_body_CAD; devuelve posicion y cuaternion."""
    if not all(math.isfinite(v) for v in (*position, *quaternion, *cad_cm)):
        raise ValueError('La pose contiene valores no finitos')
    norm = math.sqrt(sum(v*v for v in quaternion))
    if norm < 1e-12:
        raise ValueError('Cuaternion de norma cero')
    q = tuple(v / norm for v in quaternion)
    # R = [[-1,0,0], [0,0,1], [0,1,0]], det(R) = +1.
    q_cad = (0.0, math.sqrt(0.5), math.sqrt(0.5), 0.0)
    # Traslacion local = -R * CM_CAD.
    offset = (cad_cm[0], -cad_cm[2], -cad_cm[1])
    shift = rotate(q, offset)
    return (tuple(position[i] + shift[i] for i in range(3)),
            multiply(q, q_cad))


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                          qos_profile_sensor_data)
    from rclpy.executors import ExternalShutdownException
    from geometry_msgs.msg import Point, PoseStamped, TransformStamped
    from tf2_ros import TransformBroadcaster
    from visualization_msgs.msg import Marker

    class MeshNode(Node):
        def __init__(self):
            super().__init__('visualizar_hexacoptero')
            default_mesh = Path(__file__).resolve().parent.parent / 'meshes' / 'hexacoptero.stl'
            self.declare_parameter('mesh_path', str(default_mesh))
            self.declare_parameter('cad_cm', [-0.00541, -0.04088, 1.08888])
            self.declare_parameter('pose_topic', '/hexacopter/pose')
            self.declare_parameter('marker_topic', '/hexacopter/mesh')
            self.declare_parameter('trail_topic', '/hexacopter/trail')
            self.declare_parameter('tracking_frame', 'hexacopter')
            self.declare_parameter('trail_min_distance', 0.10)
            self.declare_parameter('trail_width', 0.08)
            self.declare_parameter('trail_max_points', 20000)
            path = Path(self.get_parameter('mesh_path').value).expanduser().resolve()
            if not path.is_file():
                raise FileNotFoundError(f'No se encuentra el STL: {path}')
            self.cm = tuple(self.get_parameter('cad_cm').value)
            if len(self.cm) != 3 or not all(math.isfinite(v) for v in self.cm):
                raise ValueError('cad_cm debe contener tres coordenadas finitas en metros')
            self.uri = path.as_uri()
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.pub = self.create_publisher(
                Marker, self.get_parameter('marker_topic').value, qos)
            self.trail_pub = self.create_publisher(
                Marker, self.get_parameter('trail_topic').value, qos)
            self.tf_broadcaster = TransformBroadcaster(self)
            self.sub = self.create_subscription(
                PoseStamped, self.get_parameter('pose_topic').value,
                self.receive, qos_profile_sensor_data)
            self.last_marker = None
            self.last_transform = None
            self.trail = None
            self.last_trail_position = None
            self.trail_min_distance = float(
                self.get_parameter('trail_min_distance').value)
            self.trail_width = float(self.get_parameter('trail_width').value)
            self.trail_max_points = int(
                self.get_parameter('trail_max_points').value)
            if self.trail_min_distance <= 0.0:
                raise ValueError('trail_min_distance debe ser mayor que cero')
            if self.trail_width <= 0.0:
                raise ValueError('trail_width debe ser mayor que cero')
            if self.trail_max_points < 2:
                raise ValueError('trail_max_points debe ser al menos 2')
            self.warned = False
            # Repetir la ultima pose permite abrir RViz despues de parar Simulink.
            # No representa datos nuevos: se conserva el sello temporal recibido.
            self.timer = self.create_timer(1.0, self.repeat)
            self.get_logger().info(f'Malla: {path}')
            self.get_logger().info('Esperando /hexacopter/pose; inicia Simulink.')

        def receive(self, msg):
            p, q = msg.pose.position, msg.pose.orientation
            try:
                if not msg.header.frame_id:
                    raise ValueError('PoseStamped no tiene frame_id')
                pos, ori = mesh_pose((p.x, p.y, p.z), (q.x, q.y, q.z, q.w), self.cm)
            except ValueError as error:
                if not self.warned:
                    self.get_logger().warning(str(error))
                    self.warned = True
                return
            self.warned = False

            # Publicar el frame del cuerpo, sin el desplazamiento aplicado al
            # STL. De esta forma, la camara sigue el centro del UAV.
            norm_q = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = msg.header.frame_id
            transform.child_frame_id = self.get_parameter('tracking_frame').value
            transform.transform.translation.x = p.x
            transform.transform.translation.y = p.y
            transform.transform.translation.z = p.z
            transform.transform.rotation.x = q.x / norm_q
            transform.transform.rotation.y = q.y / norm_q
            transform.transform.rotation.z = q.z / norm_q
            transform.transform.rotation.w = q.w / norm_q
            self.last_transform = transform
            self.tf_broadcaster.sendTransform(transform)

            # Registrar la posicion real del cuerpo en coordenadas del mundo.
            # Los puntos casi repetidos se descartan para limitar el tamano
            # del mensaje y evitar una linea excesivamente densa.
            actual = (p.x, p.y, p.z)
            agregar_punto = self.last_trail_position is None
            if self.last_trail_position is not None:
                distancia = math.dist(actual, self.last_trail_position)
                agregar_punto = distancia >= self.trail_min_distance

            if agregar_punto:
                if self.trail is None:
                    self.trail = Marker()
                    self.trail.header.frame_id = msg.header.frame_id
                    self.trail.ns = 'trayectoria_recorrida'
                    self.trail.id = 0
                    self.trail.type = Marker.LINE_STRIP
                    self.trail.action = Marker.ADD
                    self.trail.pose.orientation.w = 1.0
                    self.trail.scale.x = self.trail_width
                    self.trail.color.r = 0.10
                    self.trail.color.g = 0.85
                    self.trail.color.b = 1.00
                    self.trail.color.a = 1.0

                punto = Point()
                punto.x, punto.y, punto.z = actual
                self.trail.points.append(punto)
                if len(self.trail.points) > self.trail_max_points:
                    self.trail.points.pop(0)
                self.last_trail_position = actual
                self.trail.header.stamp = self.get_clock().now().to_msg()
                self.trail_pub.publish(self.trail)

            marker = Marker()
            marker.header = msg.header
            marker.ns = 'hexacoptero'
            marker.id = 0
            marker.type = Marker.MESH_RESOURCE
            marker.action = Marker.ADD
            marker.mesh_resource = self.uri
            marker.mesh_use_embedded_materials = False
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = pos
            (marker.pose.orientation.x, marker.pose.orientation.y,
             marker.pose.orientation.z, marker.pose.orientation.w) = ori
            marker.scale.x = marker.scale.y = marker.scale.z = 1.0
            marker.color.r, marker.color.g, marker.color.b = 0.65, 0.72, 0.82
            marker.color.a = 1.0
            if self.last_marker is None:
                self.get_logger().info(f'Pose recibida en {msg.header.frame_id}. Publicando malla.')
            self.last_marker = marker
            self.pub.publish(marker)

        def repeat(self):
            if self.last_marker is not None:
                self.pub.publish(self.last_marker)
            if self.last_transform is not None:
                # Actualizar solo el sello temporal mantiene disponible el TF
                # al detener Simulink sin inventar una nueva pose.
                self.last_transform.header.stamp = self.get_clock().now().to_msg()
                self.tf_broadcaster.sendTransform(self.last_transform)
            if self.trail is not None:
                self.trail.header.stamp = self.get_clock().now().to_msg()
                self.trail_pub.publish(self.trail)

    rclpy.init()
    node = None
    try:
        node = MeshNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
