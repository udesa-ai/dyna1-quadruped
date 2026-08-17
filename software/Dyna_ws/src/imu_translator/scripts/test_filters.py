"""

Uso (correr con cwd = software/Dyna_ws/):
    python3 src/imu_translator/imu_translator/simulate_fixed_kalman.py
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from ahrs.filters import Madgwick

DATA_DIR = Path.cwd() / "src" / "imu_translator" / "data"
PATH_CSV = DATA_DIR / "filter_validation_20260807_150343.csv"#"filter_validation_20260807_150721.csv" #"filter_validation_20260811_141950.csv" 
SAVE_DIR = DATA_DIR / "rotation"

GRAVITY = 9.81
G_WORLD = np.array([0.0, 0.0, -GRAVITY])
IMU_OFFSET = np.array([0.24, 0.0, 0.0])  

REQUIRED_COLUMNS = [
    "timestamp", "ax", "ay", "az",
    "imu_wx", "imu_wy", "imu_wz",
    "mocap_qw", "mocap_qx", "mocap_qy", "mocap_qz",
    "kalman_qw", "kalman_qx", "kalman_qy", "kalman_qz",
    "madgwick_qw", "madgwick_qx", "madgwick_qy", "madgwick_qz",
    "mocap_wx", "mocap_wy", "mocap_wz",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Simula el Kalman de filter_comparison2.py con los Jacobianos F y H corregidos."
    )
    parser.add_argument(
        "csv_path", type=Path, nargs="?", default=PATH_CSV,
        help=f"CSV de filter_validation.py. Default: {PATH_CSV}",
    )
    parser.add_argument(
        "--plot-output", type=Path, default=SAVE_DIR / "projected_gravity_fixed_kalman.png",
        help="Ruta donde guardar el gráfico.",
    )
    parser.add_argument("--max-plot-points", type=int, default=12000)
    parser.add_argument(
        "--align-samples", type=int, default=None,
        help="Si se especifica, ajusta la R de mounting/yaw de cada filtro sólo con las "
             "primeras N muestras en vez de toda la trayectoria (igual que convert_proj_grav.py).",
    )
    parser.add_argument(
        "--output-dir", type=Path, default=SAVE_DIR,
        help="Carpeta donde guardar la R estimada para el Kalman arreglado (.npy).",
    )
    parser.add_argument(
        "--imu-to-mocap-r", type=Path, default=SAVE_DIR / "ang_vel_R_imu_to_mocap.npy",
        help="R (frame IMU -> frame Mocap) ya calibrada con estimate_ang_vel_R.py, "
             "aplicada al accel antes del update del KF.",
    )
    return parser.parse_args()


def load_csv(csv_path: Path) -> dict[str, np.ndarray]:
    if not csv_path.exists():
        raise FileNotFoundError(f"No existe el archivo: {csv_path}")
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        missing = [c for c in REQUIRED_COLUMNS if c not in (reader.fieldnames or [])]
        if missing:
            raise ValueError(f"{csv_path}: faltan columnas {missing}")
        rows = list(reader)
    return {col: np.array([float(row[col]) for row in rows]) for col in REQUIRED_COLUMNS}


# ---- Cuaterniones (mismas convenciones que filter_comparison2.py) ----

def quat_normalize(q: np.ndarray) -> np.ndarray:
    return q / np.linalg.norm(q)


def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_to_rotmat(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y**2 + z**2),     2 * (x * y - z * w),       2 * (x * z + y * w)],
        [    2 * (x * y + z * w), 1 - 2 * (x**2 + z**2),       2 * (y * z - x * w)],
        [    2 * (x * z - y * w),     2 * (y * z + x * w),   1 - 2 * (x**2 + y**2)],
    ])


def quat_to_rotmat_batch(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    n = len(q)
    R = np.empty((n, 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z)
    R[:, 0, 1] = 2 * (x * y - z * w)
    R[:, 0, 2] = 2 * (x * z + y * w)
    R[:, 1, 0] = 2 * (x * y + z * w)
    R[:, 1, 1] = 1 - 2 * (x * x + z * z)
    R[:, 1, 2] = 2 * (y * z - x * w)
    R[:, 2, 0] = 2 * (x * z - y * w)
    R[:, 2, 1] = 2 * (y * z + x * w)
    R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def project_gravity_batch(q: np.ndarray) -> np.ndarray:
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    R = quat_to_rotmat_batch(q)
    return np.einsum("nij,j->ni", R.transpose(0, 2, 1), G_WORLD)


def quat_from_two_vectors(u: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Cuaternión de rotación de menor arco que lleva u -> v (ambos
    unitarios): rota sólo lo necesario para alinearlos, sin ningún giro
    extra alrededor de v. Usado para inicializar el filtro con el
    roll/pitch que ya da la primera lectura de accel, en vez de arrancar
    en identidad y depender de que converja."""
    dot = np.dot(u, v)
    if dot < -1.0 + 1e-8:
        # u y v casi opuestos: cualquier eje perpendicular a u sirve de eje de giro (180°)
        axis = np.cross(u, np.array([1.0, 0.0, 0.0]))
        if np.linalg.norm(axis) < 1e-6:
            axis = np.cross(u, np.array([0.0, 1.0, 0.0]))
        axis = axis / np.linalg.norm(axis)
        return np.array([0.0, axis[0], axis[1], axis[2]])
    xyz = np.cross(u, v)
    return quat_normalize(np.array([1.0 + dot, xyz[0], xyz[1], xyz[2]]))


def initial_quaternion_from_accel(accel_corrected: np.ndarray) -> np.ndarray:
    """q inicial (sólo roll/pitch; yaw en 0, no observable sin
    magnetómetro) tal que R(q)^T @ G_WORLD ya coincide con esta lectura de
    accel, para arrancar el filtro alineado con la gravedad medida en vez
    de en la identidad."""
    accel_dir = accel_corrected / np.linalg.norm(accel_corrected)
    g_dir = G_WORLD / np.linalg.norm(G_WORLD)
    return quat_from_two_vectors(accel_dir, g_dir)


# ---- EKF con los Jacobianos F y H corregidos ----

class FixedKalman:
    """Mismo EKF que kalman_predict()/kalman_update() en filter_comparison2.py,
    salvo por F y H, que ahí estaban aproximados a la identidad. Acá:

    - F = I + 0.5*dt*Omega(gyro), la Jacobiana real de la cinemática de
      cuaterniones q_dot = 0.5 * Omega(gyro) @ q (equivalente a
      quat_multiply(q, [0,*gyro]) del original, sólo que ahora también
      se propaga a P).
    - H = derivada analítica de h(q) = R(q)^T @ g_world (normalizada)
      respecto de q, en vez de np.eye(4)[:3, :].
    """

    def __init__(
        self,
        dt_ref: float,
        q_scale: float = 0.001,
        r_scale: float = 30.0,
        adaptive_gain: float = 80.0,
        q_init: np.ndarray | None = None,
    ):
        self.q = np.array([1.0, 0.0, 0.0, 0.0]) if q_init is None else q_init
        self.P = np.eye(4) * 0.1
        self.Q = np.eye(4) * q_scale
        self.R = np.eye(3) * r_scale
        self.dt_ref = dt_ref
        # Rechazo adaptativo: cuanto más se aleja |accel| de g, menos se
        # confía en la medición (probablemente es aceleración lineal del
        # movimiento contaminando la referencia de gravedad, no sólo ruido).
        self.adaptive_gain = adaptive_gain

    def predict(self, gyro: np.ndarray, dt: float) -> None:
        if dt <= 0:
            return
        omega_quat = np.array([0.0, gyro[0], gyro[1], gyro[2]])
        q_dot = 0.5 * quat_multiply(self.q, omega_quat)
        self.q = quat_normalize(self.q + q_dot * dt)

        wx, wy, wz = gyro
        Omega = np.array([
            [0.0, -wx, -wy, -wz],
            [wx,   0.0,  wz, -wy],
            [wy,  -wz,  0.0,  wx],
            [wz,   wy, -wx,  0.0],
        ])
        F = np.eye(4) + 0.5 * dt * Omega
        self.P = F @ self.P @ F.T + self.Q * (dt / self.dt_ref)

    def update(self, accel_corrected: np.ndarray) -> None:
        R_body = quat_to_rotmat(self.q)
        g_expected = R_body.T @ G_WORLD

        accel_mag = np.linalg.norm(accel_corrected)
        accel_norm = accel_corrected / (accel_mag + 1e-8)
        h_norm = g_expected / (np.linalg.norm(g_expected) + 1e-8)
        innov = accel_norm - h_norm

        w, x, y, z = self.q
        # d(h_norm)/dq ; |h(q)| = 9.81 exacto para cualquier q unitario
        # (rotación preserva norma), así que dividir por g normaliza sin
        # necesidad de la regla del cociente completa.
        H = 2.0 * np.array([
            [ y, -z,  w, -x],
            [-x, -w, -z, -y],
            [0.0, 2 * x, 2 * y, 0.0],
        ])

        deviation = abs(accel_mag - GRAVITY) / GRAVITY
        R_eff = self.R * (1.0 + self.adaptive_gain * deviation**2)

        S = H @ self.P @ H.T + R_eff
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            K = np.zeros((4, 3))

        self.q = quat_normalize(self.q + K @ innov)
        self.P = (np.eye(4) - K @ H) @ self.P


def preprocess_imu_sample( data: dict[str, np.ndarray], i: int,
    offset: np.ndarray, R_imu_to_mocap: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    
    gyro = np.array([data["imu_wx"][i], data["imu_wy"][i], data["imu_wz"][i]])
    accel = np.array([-data["az"][i], -data["ax"][i], -data["ay"][i]]) * GRAVITY

    gyro = R_imu_to_mocap @ gyro
    accel = R_imu_to_mocap @ accel
    a_centripetal = np.cross(gyro, np.cross(gyro, offset))
    accel_corrected = accel - a_centripetal

    return gyro, accel_corrected


def run_simulation(data: dict[str, np.ndarray], dt_nominal: float, q_scale: float = 0.001, r_scale: float = 30.0,
    adaptive_gain: float = 80.0, offset=np.array([0.0, 0.0, 0.0]), R_imu_to_mocap: np.ndarray = np.eye(3)) -> np.ndarray:

    n = len(data["timestamp"])
    _, accel0 = preprocess_imu_sample(data, 0, offset, R_imu_to_mocap)
    q_init = initial_quaternion_from_accel(accel0)
    kf = FixedKalman(dt_ref=dt_nominal, q_scale=q_scale, r_scale=r_scale, adaptive_gain=adaptive_gain, q_init=q_init)
    q_out = np.empty((n, 4))

    for i in range(n):
        gyro, accel_corrected = preprocess_imu_sample(data, i, offset, R_imu_to_mocap)
        kf.predict(gyro, dt_nominal)
        kf.update(accel_corrected)
        q_out[i] = kf.q

    return q_out


def run_madgwick_simulation(data: dict[str, np.ndarray],offset=np.array([0.0, 0.0, 0.0]),
    R_imu_to_mocap: np.ndarray = np.eye(3)) -> np.ndarray:

    n = len(data["timestamp"])
    madgwick = Madgwick(sampleperiod=1 / 100)
    _, accel0 = preprocess_imu_sample(data, 0, offset, R_imu_to_mocap)
    q = initial_quaternion_from_accel(accel0)
    q_out = np.empty((n, 4))

    for i in range(n):
        gyro, accel_corrected = preprocess_imu_sample(data, i, offset, R_imu_to_mocap)
        q = madgwick.updateIMU(q, gyr=gyro, acc=-accel_corrected)
        q_out[i] = q

    return q_out


def estimate_rotation(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Kabsch/Procrustes ortogonal (SVD), igual método que convert_proj_grav.py:
        min_R  sum_i || target_i - R @ source_i ||^2   s.a. R^T R = I, det(R) = 1
    """
    H = source.T @ target
    U, _, Vt = np.linalg.svd(H)
    V = Vt.T
    d = np.sign(np.linalg.det(V @ U.T))
    D = np.diag([1.0, 1.0, d])
    return V @ D @ U.T


def rotation_angle_deg(R: np.ndarray) -> float:
    cos_angle = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(cos_angle)))


def rmse(reference: np.ndarray, estimate: np.ndarray) -> float:
    error = estimate - reference
    return float(np.sqrt(np.mean(np.sum(error**2, axis=1))))


def choose_plot_indices(n_rows: int, max_points: int) -> np.ndarray:
    if n_rows <= max_points:
        return np.arange(n_rows)
    return np.linspace(0, n_rows - 1, max_points, dtype=int)


def save_csv_data(time_s: np.ndarray, g_mocap: np.ndarray, g_kalman_offset: np.ndarray,
    g_madgwick_offset: np.ndarray, output_path: Path) -> None:
    with open(output_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'timestamp',
            'mocap_gx', 'mocap_gy', 'mocap_gz',
            'kalman_offset_gx', 'kalman_offset_gy', 'kalman_offset_gz',
            'madgwick_offset_gx', 'madgwick_offset_gy', 'madgwick_offset_gz'
        ])
        for i in range(len(time_s)):
            writer.writerow([
                f"{time_s[i]:.6f}",
                f"{g_mocap[i, 0]:.6f}", f"{g_mocap[i, 1]:.6f}", f"{g_mocap[i, 2]:.6f}",
                f"{g_kalman_offset[i, 0]:.6f}", f"{g_kalman_offset[i, 1]:.6f}", f"{g_kalman_offset[i, 2]:.6f}",
                f"{g_madgwick_offset[i, 0]:.6f}", f"{g_madgwick_offset[i, 1]:.6f}", f"{g_madgwick_offset[i, 2]:.6f}"
            ])


def save_angular_velocity_csv(time_s: np.ndarray, data: dict[str, np.ndarray],
    R_imu_to_mocap: np.ndarray, output_path: Path) -> None:
    n = len(time_s)
    with open(output_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'timestamp',
            'imu_wx_corrected', 'imu_wy_corrected', 'imu_wz_corrected',
            'mocap_wx', 'mocap_wy', 'mocap_wz'
        ])
        for i in range(n):
            imu_w = np.array([data["imu_wx"][i], data["imu_wy"][i], data["imu_wz"][i]])
            imu_w_corrected = R_imu_to_mocap @ imu_w
            writer.writerow([
                f"{time_s[i]:.6f}",
                f"{imu_w_corrected[0]:.6f}", f"{imu_w_corrected[1]:.6f}", f"{imu_w_corrected[2]:.6f}",
                f"{data['mocap_wx'][i]:.6f}", f"{data['mocap_wy'][i]:.6f}", f"{data['mocap_wz'][i]:.6f}"
            ])


def save_plot(time_s: np.ndarray, g_mocap: np.ndarray, g_series: dict[str, np.ndarray],
    max_points: int, output_path: Path) -> None:
    axes_names = ("x", "y", "z")
    indices = choose_plot_indices(len(time_s), max_points)
    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    for i, (axis, axis_name) in enumerate(zip(axes, axes_names)):
        axis.plot(time_s[indices], g_mocap[indices, i], label="Mocap")
        for name, g in g_series.items():
            if name not in ['Kalman original', 'Madgwick original']:
                axis.plot(time_s[indices], g[indices, i], label=name, alpha=0.8)
        axis.set_ylabel(f"g_{axis_name} [m/s²]")
        axis.set_title(f"Gravedad proyectada sobre el eje {axis_name.upper()}")
        axis.grid(True, alpha=0.3)
        axis.legend()
    axes[-1].set_xlabel("Tiempo [s]")
    fig.suptitle("Gravedad proyectada: Mocap vs. Kalman (con y sin offset) vs. Madgwick (con offset)")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    args = parse_args()
    data = load_csv(args.csv_path)
    n = len(data["timestamp"])
    dt_nominal = (data["timestamp"][-1] - data["timestamp"][0]) / (n - 1)
    time_s = np.arange(n) * dt_nominal

    mocap_q = np.stack([data["mocap_qw"], data["mocap_qx"], data["mocap_qy"], data["mocap_qz"]], axis=1)
    kalman_q_old = np.stack([data["kalman_qw"], data["kalman_qx"], data["kalman_qy"], data["kalman_qz"]], axis=1)
    madgwick_q = np.stack([data["madgwick_qw"], data["madgwick_qx"], data["madgwick_qy"], data["madgwick_qz"]], axis=1)

    if not args.imu_to_mocap_r.exists():
        raise FileNotFoundError(
            f"No existe {args.imu_to_mocap_r}: correr antes estimate_ang_vel_R.py "
            "para calibrar la R IMU -> Mocap."
        )
    R_imu_to_mocap = np.load(args.imu_to_mocap_r)
    print(f"R (IMU -> Mocap) cargada de: {args.imu_to_mocap_r}")
    print(np.array2string(R_imu_to_mocap, precision=6, suppress_small=True))

    print(f"Muestras: {n}  |  duración: {time_s[-1]:.2f} s  |  dt nominal: {dt_nominal:.5f} s")
    print("Simulando Kalman aditivo con F y H corregidos...")
    kalman_q_fixed = run_simulation(data, dt_nominal, R_imu_to_mocap=R_imu_to_mocap)
    print("Simulando Kalman fixed...")
    kalman_q_fixed_offset = run_simulation(data, dt_nominal, offset=IMU_OFFSET, R_imu_to_mocap=R_imu_to_mocap)
    print("Simulando Kalman fixed offset...")
    madgwick_q_fixed = run_madgwick_simulation(data, offset=IMU_OFFSET, R_imu_to_mocap=R_imu_to_mocap)
    print("Simulando Madgwick arreglado...")


    g_mocap = project_gravity_batch(mocap_q)
    g_kalman_old = project_gravity_batch(kalman_q_old)
    g_madgwick = project_gravity_batch(madgwick_q)
    g_kalman_fixed = project_gravity_batch(kalman_q_fixed)
    g_kalman_offset =project_gravity_batch(kalman_q_fixed_offset)
    g_madgwick_fixed = project_gravity_batch(madgwick_q_fixed)


    print("\nRMSE vs. Mocap (sin corregir mounting/yaw, sólo para comparar el efecto del fix):")
    raw_series = {
        "Kalman original": g_kalman_old,
        "Madgwick original": g_madgwick,
        "Kalman arreglado": g_kalman_fixed,
        "Kalman offset": g_kalman_offset,
        "Madgwick offset": g_madgwick_fixed,
    }
    for name, g in raw_series.items():
        print(f"  {name:<18}: {rmse(g_mocap, g):.4f} m/s²")

    # Mismo paso que convert_proj_grav.py: ajustar una R fija por filtro
    # (Kabsch) que compensa el desalineamiento de montaje/yaw, y recién ahí
    # comparar contra Mocap en igualdad de condiciones.
    fit_slice = slice(None, args.align_samples)
    print(f"\nAjustando R de mounting/yaw por filtro con: "
          f"{'toda la trayectoria' if args.align_samples is None else f'primeras {args.align_samples} muestras'}")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    # corrected_series: dict[str, np.ndarray] = {}
    # for name, g_raw in raw_series.items():
    #     g_corrected = g_raw @ R.T
    #     corrected_series[name] = g_corrected

    #     print(f"\n[{name}] R (gravedad {name} -> Mocap):")
    #     print(np.array2string(R, precision=6, suppress_small=True))
    #     print(f"  Ángulo de R  : {rotation_angle_deg(R):.2f} deg")
    #     print(f"  RMSE sin corregir: {rmse(g_mocap, g_raw):.4f} m/s²")
    #     print(f"  RMSE corregido   : {rmse(g_mocap, g_corrected):.4f} m/s²")

    #     if name == "Kalman arreglado":
    #         output_path = args.output_dir / "grav_R_kalman_fixed_to_mocap.npy"
    #         np.save(output_path, R)
    #         print(f"  R guardada en: {output_path.resolve()}")
        # elif name == "Kalman error-state":
        #     output_path = args.output_dir / "grav_R_kalman_eskf_to_mocap.npy"
        #     np.save(output_path, R)
        #     print(f"  R guardada en: {output_path.resolve()}")
        # elif name == "Kalman error-state (offset re-estimado)":
        #     output_path = args.output_dir / "grav_R_kalman_eskf_fit_to_mocap.npy"
        #     np.save(output_path, R)
        #     print(f"  R guardada en: {output_path.resolve()}")

    args.plot_output.parent.mkdir(parents=True, exist_ok=True)
    save_plot(time_s, g_mocap, raw_series, args.max_plot_points, args.plot_output)
    print(f"\nGráfico guardado en: {args.plot_output.resolve()}")

    csv_output = args.plot_output.parent / (args.plot_output.stem + '.csv')
    save_csv_data(time_s, g_mocap, g_kalman_offset, g_madgwick_fixed, csv_output)
    print(f"CSV guardado en: {csv_output.resolve()}")

    ang_vel_csv = args.plot_output.parent / "angular_velocity_comparison.csv"
    save_angular_velocity_csv(time_s, data, R_imu_to_mocap, ang_vel_csv)
    print(f"CSV velocidad angular guardado en: {ang_vel_csv.resolve()}")


if __name__ == "__main__":
    main()
