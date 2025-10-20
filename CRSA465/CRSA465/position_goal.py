import time
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from pymoveit2 import MoveIt2, MoveIt2State
from CRSA465 import crsa465_config as robot
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import Pose

def move_to_pose_goal(node, positions, quats, cartesian=False, synchronous=True, cancel_after_secs=0.0, timeout=30.0):
    """
    Mueve el robot CRS A465 a una o varias poses usando pymoveit2.
    - positions: lista de [x, y, z] o un solo [x, y, z]
    - quats: lista de [x, y, z, w] o un solo [x, y, z, w]
    Devuelve un diccionario con resultados:
        {"status": "ok" / "error" / "timeout" / "canceled",
         "message": "...",
         "details": [{"pose": [...], "status": "...", "message": "..."}]}
    """

    # --- Suscripción a joint states ---
    callback_group = ReentrantCallbackGroup()
    joint_positions_received = {}

    def joint_state_callback(msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            joint_positions_received[name] = pos

    node.create_subscription(JointState, "/joint_states", joint_state_callback, 20)

    # --- Configuración de MoveIt2 ---
    moveit2 = node.moveit2
    moveit2.max_velocity = 0.5
    moveit2.max_acceleration = 0.5
    planning_timeout = 2.0

    # Asegurarse de que positions y quats sean listas de poses
    if isinstance(positions[0], (float, int)):
        positions = [positions]
    if isinstance(quats[0], (float, int)):
        quats = [quats]

    # Esperar a joint_states iniciales
    node.get_logger().info("Esperando joint_states...")
    while not all(j in joint_positions_received for j in robot.CRSA465Config.joint_names()):
        rclpy.spin_once(node, timeout_sec=0.1)
    node.get_logger().info("Joint states iniciales recibidos.")

    results = []
    try:
        # --- Movimiento individual o secuencial ---
        for pos, quat in zip(positions, quats):
            node.get_logger().info(f"Moviendo a pose {pos} con quat {quat}")
            moveit2.move_to_pose(position=pos, quat_xyzw=quat, cartesian=cartesian)

            start_time = time.time()
            rate = node.create_rate(10)

            if synchronous:
                moveit2.wait_until_executed()
                results.append({"pose": pos, "status": "ok", "message": "Movimiento completado correctamente"})
            else:
                # Esperar a que comience ejecución
                while moveit2.query_state() not in [MoveIt2State.EXECUTING]:
                    rate.sleep()
                    if time.time() - start_time > planning_timeout:
                        raise TimeoutError("Timeout esperando a que el movimiento se planee")

                future = moveit2.get_execution_future()

                # Cancelar si se pide
                if cancel_after_secs > 0.0:
                    node.create_rate(cancel_after_secs).sleep()
                    moveit2.cancel_execution()
                    return {"status": "canceled", "message": f"Movimiento cancelado después de {cancel_after_secs}s"}

                # Esperar hasta terminar
                start_time = time.time()
                while not future.done():
                    rate.sleep()
                    if time.time() - start_time > timeout:
                        raise TimeoutError("Timeout esperando a que el movimiento termine")

                result = future.result()
                if result and hasattr(result.result, "error_code"):
                    code = result.result.error_code.val
                    if code != MoveItErrorCodes.SUCCESS:
                        moveit2.cancel_execution()
                        results.append({"pose": pos, "status": "error", "message": f"Código de error {code}"})
                        return {"status": "error", "message": f"Error ejecutando pose {pos}", "details": results}

                results.append({"pose": pos, "status": "ok", "message": "Movimiento completado correctamente"})

        return {"status": "ok", "message": "Todas las poses ejecutadas correctamente", "details": results}

    except TimeoutError as te:
        moveit2.cancel_execution()
        return {"status": "timeout", "message": str(te), "details": results}

    except Exception as e:
        moveit2.cancel_execution()
        return {"status": "error", "message": str(e), "details": results}
