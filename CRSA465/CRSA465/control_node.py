# --- main_control.py (corregido y optimizado) ---
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from fastapi import FastAPI, HTTPException, Request
import uvicorn
import threading
import sys
import logging
from logging.handlers import TimedRotatingFileHandler
from fastapi.middleware.cors import CORSMiddleware
import sqlite3
from datetime import datetime
from typing import Dict
import json
from CRSA465.position_goal import move_to_pose_goal
from rclpy.callback_groups import ReentrantCallbackGroup
from CRSA465 import crsa465_config as robot
from trajectory_msgs.msg import JointTrajectory
import time
import os

try:
    from pymoveit2 import MoveIt2, MoveIt2State
except ImportError:
    print("\n--- WARNING: pymoveit2 not found. Install with: pip install pymoveit2 ---\n", file=sys.stderr)

# ---------------------------------------------------
# Configuración general
# ---------------------------------------------------
PASSWORD = "1234"  #secrets.token_urlsafe(16)
PLANNING_GROUP = "arm"
origins = ["http://localhost:4200", "*"]
DB_FILE = "CRSA465/database/trajectories.db"
stored_trajectories: list[JointTrajectory] = []

app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

ros_node = None

# ---------------------------------------------------
# Configuración logging
# ---------------------------------------------------

log_dir = os.getenv('ROS_LOG_DIR', os.path.join(os.getcwd(), 'logs'))
os.makedirs(log_dir, exist_ok=True)
log_path = os.path.join(log_dir, 'control_node.log')

logger = logging.getLogger()
logger.setLevel(logging.INFO)

# Configurar manejador de rotación diaria con backup de 7 archivos (7 días)
handler = TimedRotatingFileHandler(log_path, when="D", interval=1, backupCount=7)
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s', datefmt='%Y-%m-%d %H:%M:%S')
handler.setFormatter(formatter)

logger.addHandler(handler)

# ---------------------------------------------------
# Variables de threading
# ---------------------------------------------------
move_lock = threading.Lock()
move_thread = None
cancel_requested = threading.Event()

# ---------------------------------------------------
# Base de datos
# ---------------------------------------------------
def init_db():
    conn = sqlite3.connect(DB_FILE, check_same_thread=False)
    cursor = conn.cursor()
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS trajectories (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            name TEXT,
            created_at TEXT NOT NULL
        )
    ''')
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS trajectory_points (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            trajectory_id INTEGER NOT NULL,
            angles TEXT NOT NULL,
            time REAL NOT NULL,
            FOREIGN KEY (trajectory_id) REFERENCES trajectories(id) ON DELETE CASCADE
        )
    ''')
    conn.commit()
    conn.close()
    logging.info("Database initialized with trajectories and trajectory_points.")

def dict_from_trajectory(row) -> Dict:
    return {"id": row[0], "name": row[1], "created_at": row[2]}

def dict_from_point(row) -> Dict:
    return {
        "id": row[0],
        "trajectory_id": row[1],
        "angles": json.loads(row[2]),
        "time": row[3],
    }

# ---------------------------------------------------
# Nodo ROS2 de control
# ---------------------------------------------------
class ControlNode(Node):
    def __init__(self):
        super().__init__("secure_joint_controller")
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.joint6_pub = self.create_publisher(Float64, "/joint6_command", 10)
        self.joint6_sub = self.create_subscription(Float64, "joint6_state", self.joint6_state_callback, 10)
        self.current_angle = 0.0
        logging.info("[ROS] ControlNode initialized and ready.")

    def send_joint6_command(self, grados: float):
        msg = Float64()
        msg.data = grados * 3.14159 / 180.0
        self.joint6_pub.publish(msg)
        logging.info(f"[ROS] Enviando comando a junta_5_6: {grados}° ({msg.data:.3f} rad)")

    def joint6_state_callback(self, msg: Float64):
        self.current_angle = msg.data
        logging.debug(f"[ROS] Ángulo actual de junta_5_6 recibido: {self.current_angle:.3f} rad")

    def publish_joint_state(self, names, positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = [float(p) for p in positions]
        self.publisher.publish(msg)
        logging.info(f"[ROS] Published joint states: {dict(zip(names, positions))}")
        
# ---------------------------------------------------
# Nodo ROS2 con control cartesiano (MoveIt2)
# ---------------------------------------------------
class CartesianControlNode(ControlNode):
    def __init__(self):
        super().__init__()
        try:
            self.PLANNING_GROUP = "arm"
            self.moveit2 = MoveIt2(
                node=self,
                joint_names=robot.CRSA465Config.joint_names(),
                base_link_name=robot.CRSA465Config.base_link_name(),
                end_effector_name=robot.CRSA465Config.end_effector_name(),
                group_name=self.PLANNING_GROUP,
                callback_group=(ReentrantCallbackGroup())
            )
            self.moveit2.max_velocity = 0.5
            self.moveit2.max_acceleration = 0.5
            logging.info(f"[ROS] MoveIt2 group '{self.PLANNING_GROUP}' listo.")
        except Exception as e:
            logging.warning(f"[ROS] No se pudo inicializar pymoveit2: {e}")

# ---------------------------------------------------
# Funciones de ejecución de movimiento
# ---------------------------------------------------

def execute_movement(positions, quats, cartesian, synchronous, cancel_after_secs):
    global cancel_requested
    try:
        # Borra posible flag previo
        cancel_requested.clear()

        result = move_to_pose_goal(
            ros_node,
            positions=positions,
            quats=quats,
            cartesian=cartesian,
            synchronous=synchronous,
            cancel_after_secs=cancel_after_secs,
        )
        logging.info(f"Movimiento finalizado con estado: {result['status']} {result['message']}")
        return result
    except Exception as e:
        logging.error(f"Error durante ejecución: {e}")
        return {"status": "error", "message": str(e)}
    finally:
        # Al terminar o cancelar, libera el lock
        if move_lock.locked():
            move_lock.release()

def execute_all_trajectories():
    global stored_trajectories, cancel_requested, move_lock

    if move_lock.locked():
        logging.warn("[ROS - EX] Movimiento ya en progreso, saliendo.")
        return

    move_lock.acquire()
    try:
        for traj in stored_trajectories:
            if cancel_requested.is_set():
                logging.info("[ROS - EX] Ejecución cancelada por usuario antes de iniciar trayectoria.")
                break
            try:
                logging.info("[ROS - EX] Empezando trayectoria")
                ros_node.moveit2.execute(traj)
            except Exception as traj_exc:
                logging.error(f"[ROS - EX] Error ejecutando trayectoria: {traj_exc}")
                break
    except Exception as e:
        logging.error(f"[ROS - EX] Error inesperado en ejecución: {e}")

    finally:
        stored_trajectories.clear()
        cancel_requested.clear()
        if move_lock.locked():
            move_lock.release()
        logging.info("[ROS - EX] Ejecución de trayectorias finalizada.")

def plan_robot_trajectory(first_pose: JointState, poses: list, cartesian: bool):
    """Planifica la trayectoria del robot dado un estado inicial y una lista de poses o ángulos."""
    global stored_trajectories
    stored_trajectories = []
    trajectory = []
    current_joint_state = first_pose

    logging.info(f"[ROS - PT] Estado inicial: {dict(zip(current_joint_state.name, current_joint_state.position))}")

    for i, p in enumerate(poses):
        is_articular = all(k in p for k in ["angle1", "angle2", "angle3", "angle4", "angle5", "angle6"])
        points = []
        # ------------------------------
        # CASO 1: Se envían ángulos articulares
        # ------------------------------
        if is_articular:
            joint_positions = [
                float(p["angle1"]),
                float(p["angle2"]),
                float(p["angle3"]),
                float(p["angle4"]),
                float(p["angle5"]),
                float(p["angle6"]),
            ]
            
            joint_state = JointState()
            joint_state.name = robot.CRSA465Config.joint_names()
            joint_state.position = joint_positions
            
            fk_result = ros_node.moveit2.compute_fk(joint_positions)
            if fk_result is None:
                logging.error(f"[ROS - PT] Falló el cálculo de FK para posición {i}")
                raise HTTPException(status_code=500, detail=f"FK failed for pose {i}")

            pose = fk_result.pose
            p = {
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z,
                "qx": pose.orientation.x,
                "qy": pose.orientation.y,
                "qz": pose.orientation.z,
                "qw": pose.orientation.w,
            }
            logging.info(f"[ROS - PT] FK[{i}] → x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}")  

        # ------------------------------
        # CASO 2: Se envían poses cartesianas
        # ------------------------------
        else:
            if not all(k in p for k in ["x", "y", "z", "qx", "qy", "qz", "qw"]):
                logging.error(f"[ROS - PT] Pose mal definida en {i}: {p}")
                raise HTTPException(status_code=400, detail=f"Pose missing fields: {p}")
            position = [float(p["x"]), float(p["y"]), float(p["z"])]
            quat_xyzw = [float(p["qx"]), float(p["qy"]), float(p["qz"]), float(p["qw"])]

            joint_state = ros_node.moveit2.compute_ik(position, quat_xyzw, start_joint_state=current_joint_state)
            if joint_state is None:
                raise HTTPException(status_code=500, detail=f"IK failed for pose {p}")
        
        # Estado actual    
        joint_positions = list(joint_state.position)
        jt_names = list(joint_state.name)

        # ------------------------------
        # Planificación
        # ------------------------------
        position = [float(p["x"]), float(p["y"]), float(p["z"])]
        quat_xyzw = [float(p["qx"]), float(p["qy"]), float(p["qz"]), float(p["qw"])]
        
        if i != 0 or cartesian:
            if cartesian:
                traj = ros_node.moveit2.plan(
                    position=position,
                    quat_xyzw=quat_xyzw,
                    start_joint_state=current_joint_state,
                    cartesian=cartesian,
                    max_step=0.0025,
                    cartesian_fraction_threshold=0.0,
                )
            else:
                traj = ros_node.moveit2.plan(
                    joint_positions = joint_positions,
                    joint_names = jt_names,
                    start_joint_state=current_joint_state,
                    cartesian=cartesian,
                    max_step=0.0025,
                    cartesian_fraction_threshold=0.0,
                )

            if traj is None:
                logging.error(f"[ROS - PT] No se pudo planear la trayectoria para el punto {i}")
                raise HTTPException(status_code=500, detail=f"Planning failed for pose {i}")

            stored_trajectories.append(traj)

            for point in traj.points:
                points.append({
                    "positions": list(point.positions),
                    "velocities": list(point.velocities) if point.velocities else [0.0]*len(point.positions),
                    "time_from_start": point.time_from_start.sec + point.time_from_start.nanosec*1e-9
                })

            trajectory.append({"pose": p, "points": points, "status": "ok"})
            current_joint_state.position = list(traj.points[-1].positions)
        else:
            current_joint_state.position = list(joint_positions)

    return trajectory

# ---------------------------------------------------
# Endpoints API REST
# ---------------------------------------------------
@app.post("/send_command")
async def send_command(request: Request):
    data = await request.json()
    if data.get("password") != PASSWORD:
        raise HTTPException(status_code=401, detail="Unauthorized")

    names, positions = data.get("name", []), data.get("position", [])
    if not isinstance(names, list) or not isinstance(positions, list) or len(names) != len(positions):
        raise HTTPException(status_code=400, detail="Invalid input")

    ros_node.publish_joint_state(names, positions)
    return {"status": "ok"}

@app.post("/send_joint6_command")
async def send_joint6_command(request: Request):
    data = await request.json()
    if data.get("password") != PASSWORD:
        raise HTTPException(status_code=401, detail="Unauthorized")
    if "value" not in data:
        raise HTTPException(status_code=400, detail="Missing 'value'")

    value = float(data["value"])
    ros_node.send_joint6_command(value)
    return {"status": "ok", "joint6_value": value, "current_angle": ros_node.current_angle}

@app.post("/send_positions")
async def send_multiple_positions(request: Request):
    global move_thread
    
    data = await request.json()
    if data.get("password") != PASSWORD:
        raise HTTPException(status_code=401, detail="Unauthorized")

    poses = data.get("poses", [])
    if not isinstance(poses, list) or len(poses) == 0:
        raise HTTPException(status_code=400, detail="Missing or invalid poses list")

    cartesian = bool(data.get("cartesian", False))
    synchronous = bool(data.get("synchronous", True))
    cancel_after_secs = float(data.get("cancel_after_secs", 0.0))

    positions = []
    quats = []
    for p in poses:
        if not all(k in p for k in ["x", "y", "z", "qx", "qy", "qz", "qw"]):
            raise HTTPException(status_code=400, detail=f"Parámetro pose faltante: {p}")
        positions.append([float(p["x"]), float(p["y"]), float(p["z"])])
        quats.append([float(p["qx"]), float(p["qy"]), float(p["qz"]), float(p["qw"])])

    acquired = move_lock.acquire(blocking=False)
    if not acquired:
        raise HTTPException(status_code=409, detail="Movimiento en progreso. Intente más tarde.")
    
    def target():
        execute_movement(positions, quats, cartesian, synchronous, cancel_after_secs)

    # Lanza la ejecución en un thread separado para no bloquear el servidor
    move_thread = threading.Thread(target=target, daemon=True)
    move_thread.start()
    return {"status": "ok", "message": "Movimiento iniciado."}

@app.get("/stop_motion")
async def stop_motion():
    global cancel_requested
    try:
        ros_node.moveit2.cancel_execution()
        cancel_requested.set()
        logging.info("[ROS - SM] Stop motion solicitado")
    except Exception as e:
                logging.error(f"[ROS - SM] Error al detener el movimiento: {e}")
                raise HTTPException(status_code=500, detail=str(e))
    return {"status": "ok", "message": "Stop Motion solicitado"}

@app.post("/plan_trajectory")
async def plan_trajectory(request: Request):
    """Endpoint principal para planear una trayectoria (articular o cartesiana)."""
    global stored_trajectories

    data = await request.json()

    if data.get("password") != PASSWORD:
        logging.error("[ROS - PT] La contraseña otorgada no es correcta")
        raise HTTPException(status_code=401, detail="Unauthorized")

    first_position = data.get("first_pose", None)
    if first_position is None:
        logging.error("[ROS - PT] Falta la primera posición del robot")
        raise HTTPException(status_code=400, detail="Missing first_pose")

    poses = data.get("poses", [])
    if not isinstance(poses, list) or len(poses) == 0:
        logging.error("[ROS - PT] Faltan posiciones para la trayectoria")
        raise HTTPException(status_code=400, detail="Missing or invalid poses list")

    cartesian = bool(data.get("cartesian", False))

    # Crear el JointState inicial
    first_pose = JointState()
    first_pose.name = robot.CRSA465Config.joint_names()
    first_pose.position = [
        float(first_position["angle1"]),
        float(first_position["angle2"]),
        float(first_position["angle3"]),
        float(first_position["angle4"]),
        float(first_position["angle5"]),
        float(first_position["angle6"]),
    ]

    # Llamamos a la función auxiliar que hace toda la lógica
    result = plan_robot_trajectory(first_pose, poses, cartesian)

    return {"status": "ok", "trajectory": result}

@app.post("/execute_trajectory")
async def execute_trajectory(request: Request):
    global move_thread, cancel_requested

    data = await request.json()

    if data.get("password") != PASSWORD:
        raise HTTPException(status_code=401, detail="Unauthorized")

    if not stored_trajectories:
        logging.warning("[ROS - ET] No hay trayectoria guardada para ejecutar")
        raise HTTPException(status_code=400, detail="No hay trayectoria guardad para ejecutar")

    if move_thread is not None and move_thread.is_alive():
        raise HTTPException(status_code=409, detail="Ejecución en progreso")

    cancel_requested.clear()
    logging.info("[ROS - ET] Empezando ejecución de trayectorias")
    move_thread = threading.Thread(target=execute_all_trajectories, daemon=True)
    move_thread.start()

    return {"status": "ok", "message": "Ejecución de trayectoria iniciada"}


# ---------------------------------------------------
# CRUD Trajectories
# ---------------------------------------------------
@app.post("/trajectories")
async def create_trajectory(request: Request):
    data = await request.json()
    
    # Verificar contraseña
    if data.get("password") != PASSWORD:
        raise HTTPException(status_code=401, detail="Unauthorized")

    name = data.get("name", f"Trajectory-{datetime.utcnow().isoformat()}")
    points = data.get("points", [])

    if not isinstance(points, list):
        raise HTTPException(status_code=400, detail="Invalid points format")

    conn = sqlite3.connect(DB_FILE, check_same_thread=False)
    cursor = conn.cursor()

    # Verificar si ya existe una trayectoria con ese nombre
    cursor.execute("SELECT id FROM trajectories WHERE name = ?", (name,))
    if cursor.fetchone() is not None:
        conn.close()
        raise HTTPException(status_code=409, detail="Trajectory name already exists")

    # Crear nueva trayectoria
    created_at = datetime.utcnow().isoformat()
    cursor.execute("INSERT INTO trajectories (name, created_at) VALUES (?, ?)", (name, created_at))
    trajectory_id = cursor.lastrowid

    for p in points:
        cursor.execute(
            "INSERT INTO trajectory_points (trajectory_id, angles, time) VALUES (?, ?, ?)",
            (trajectory_id, json.dumps(p["angles"]), p["time"])
        )

    conn.commit()
    conn.close()
    logging.info(f"[DB] Trajectory {trajectory_id} created.")
    return {"id": trajectory_id, "name": name, "points": len(points), "action": "created"}

@app.put("/trajectories/{trajectory_id}")
async def update_trajectory(trajectory_id: int, request: Request):
    data = await request.json()

    # Validar contraseña
    if data.get("password") != PASSWORD:
        raise HTTPException(status_code=401, detail="Unauthorized")

    name = data.get("name")
    points = data.get("points", [])

    if not isinstance(points, list):
        raise HTTPException(status_code=400, detail="Invalid points format")

    conn = sqlite3.connect(DB_FILE, check_same_thread=False)
    cursor = conn.cursor()

    # Verificar si la trayectoria existe
    cursor.execute("SELECT id FROM trajectories WHERE id = ?", (trajectory_id,))
    if cursor.fetchone() is None:
        conn.close()
        raise HTTPException(status_code=404, detail="Trajectory not found")

    # Verificar que el nuevo nombre no esté siendo usado por otra trayectoria
    if name:
        cursor.execute("SELECT id FROM trajectories WHERE name = ? AND id != ?", (name, trajectory_id))
        if cursor.fetchone() is not None:
            conn.close()
            raise HTTPException(status_code=409, detail="Trajectory name already exists")

        # Actualizar nombre
        cursor.execute("UPDATE trajectories SET name = ? WHERE id = ?", (name, trajectory_id))

    # Actualizar puntos
    cursor.execute("DELETE FROM trajectory_points WHERE trajectory_id = ?", (trajectory_id,))
    for p in points:
        cursor.execute(
            "INSERT INTO trajectory_points (trajectory_id, angles, time) VALUES (?, ?, ?)",
            (trajectory_id, json.dumps(p["angles"]), p["time"])
        )

    conn.commit()
    conn.close()
    logging.info(f"[DB] Trajectory {trajectory_id} updated.")

    return {
        "id": trajectory_id,
        "name": name,
        "points_updated": len(points),
        "action": "updated"
    }

@app.delete("/trajectories/{trajectory_id}")
async def delete_trajectory(trajectory_id: int):
    conn = sqlite3.connect(DB_FILE, check_same_thread=False)
    cursor = conn.cursor()
    cursor.execute("DELETE FROM trajectories WHERE id=?", (trajectory_id,))
    conn.commit()
    conn.close()
    logging.info(f"[DB] Trajectory {trajectory_id} deleted.")
    return {"status": "ok", "message": f"Trajectory {trajectory_id} deleted."}

@app.get("/trajectories")
async def list_trajectories():
    conn = sqlite3.connect(DB_FILE, check_same_thread=False)
    cursor = conn.cursor()
    cursor.execute("SELECT * FROM trajectories")
    rows = cursor.fetchall()
    conn.close()
    return [dict_from_trajectory(r) for r in rows]

@app.get("/trajectories/{trajectory_id}")
async def get_trajectory(trajectory_id: int):
    conn = sqlite3.connect(DB_FILE, check_same_thread=False)
    cursor = conn.cursor()
    cursor.execute("SELECT * FROM trajectories WHERE id=?", (trajectory_id,))
    row = cursor.fetchone()
    if not row:
        conn.close()
        raise HTTPException(status_code=404, detail="Trajectory not found")
    cursor.execute("SELECT * FROM trajectory_points WHERE trajectory_id=?", (trajectory_id,))
    points = cursor.fetchall()
    conn.close()
    trajectory = dict_from_trajectory(row)
    trajectory["points"] = [dict_from_point(p) for p in points]
    return trajectory

# ---------------------------------------------------
# ROS2 Thread y ejecución principal
# ---------------------------------------------------
def ros_thread():
    global ros_node
    rclpy.init()
    ros_node = CartesianControlNode()
    rclpy.spin(ros_node)
    ros_node.destroy_node()
    rclpy.shutdown()

def main():
    print(f"Generated password for control access: {PASSWORD}", flush=True)
    init_db()
    t = threading.Thread(target=ros_thread, daemon=True)
    t.start()
    uvicorn.run(app, host="0.0.0.0", port=8000)

if __name__ == "__main__":
    main()
