"""
Convierte orientación (Mocap, Kalman, Madgwick) a gravedad proyectada en el
frame del cuerpo, estima una R propia por filtro (Kabsch, igual método que
estimate_ang_vel_R.py pero ajustada directo sobre los vectores de gravedad
en vez de reusar la R de velocidad angular) y grafica la comparación
Mocap vs. Kalman vs. Madgwick.

Cómo se pasa de orientación a gravedad proyectada (misma cuenta en los
tres casos):

    g_body(t) = R(q(t))^T @ [0, 0, -g]

R(q(t)) es la rotación de cuerpo->mundo de quien generó q(t). Transponerla
proyecta el vector de gravedad (fijo, vertical, en el frame "mundo") al
frame del cuerpo en el instante t. La diferencia entre Mocap y los
filtros está en QUÉ "mundo" usa cada q(t), no en la fórmula:

  - Mocap: q_mocap(t) es la orientación del rigid body respecto del frame
    global de OptiTrack, calibrado con Z alineado a la vertical real.
    g_body_mocap(t) queda directo en el frame de marcadores del rigid
    body - no necesita corrección.

  - Kalman/Madgwick: q_filtro(t) es la orientación respecto del propio
    "mundo" del filtro, que también tiene Z alineado a gravedad (ambos
    usan el acelerómetro para corregir roll/pitch) pero cuyo yaw de
    referencia es arbitrario (no hay magnetómetro). Ese yaw es una
    rotación alrededor del mismo eje de gravedad, así que en teoría no
    debería afectar la proyección - pero en la práctica el eje Z de cada
    filtro no queda perfectamente alineado a gravedad, así que reusar la
    R de velocidad angular ajusta peor que estimar una R dedicada
    directo sobre estos vectores (mismo problema de mínimos cuadrados
    con restricción de rotación que en estimate_ang_vel_R.py, resuelto
    igual por Kabsch/SVD):

        min_R  sum_t || g_mocap(t) - R @ g_filtro_crudo(t) ||^2   s.a. R rotación

En teoría, al ser instantánea (no integrada en el tiempo), la gravedad
proyectada no debería arrastrar drift como sí le pasa a la orientación
completa - por eso al principio se ajustó R con toda la trayectoria. En
la práctica, si el filtro tiene drift real (ej. por aceleración lineal
del robot corrompiendo la referencia de gravedad del acelerómetro), un
ajuste sobre toda la grabación termina siendo un compromiso entre el
tramo bueno y el tramo con drift, y ni siquiera el arranque queda bien
alineado. --align-samples permite ajustar R usando solo una ventana
inicial (antes de que el drift se note), igual que estimate_orientation_R.py.

Uso:
    python src/imu_translator/scripts/convert_proj_grav.py --align-samples 100
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


DATA_DIR = Path.cwd() / "src" / "imu_translator" / "data"
SAVE_DIR = DATA_DIR / "rotation"
PATH_CSV = DATA_DIR / "filter_validation_20260807_150343.csv" #"filter_validation_20260807_150721.csv"

GRAVITY = 9.81
G_WORLD = np.array([0.0, 0.0, -GRAVITY])

REQUIRED_COLUMNS = [
    "timestamp",
    "mocap_qw", "mocap_qx", "mocap_qy", "mocap_qz",
    "kalman_qw", "kalman_qx", "kalman_qy", "kalman_qz",
    "madgwick_qw", "madgwick_qx", "madgwick_qy", "madgwick_qz",
]

FILTERS = ("kalman", "madgwick")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convierte orientación a gravedad proyectada y compara Mocap vs. Kalman vs. Madgwick."
    )
    parser.add_argument(
        "csv_path", type=Path, nargs="?", default=PATH_CSV,
        help=f"CSV de filter_validation con columnas *_q*. Default: {PATH_CSV}",
    )
    parser.add_argument(
        "--align-samples", type=int, default=None,
        help="Si se especifica, ajusta R solo con las primeras N muestras en vez de toda "
             "la trayectoria (evita que un drift real contamine el ajuste).",
    )
    parser.add_argument(
        "--output-dir", type=Path, default=SAVE_DIR,
        help="Carpeta donde guardar las R estimadas (.npy), una por filtro.",
    )
    parser.add_argument(
        "--plot-output", type=Path, default=SAVE_DIR / "projected_gravity_comparison.png",
        help="Ruta donde guardar el gráfico.",
    )
    parser.add_argument(
        "--max-plot-points", type=int, default=12000,
        help="Máximo de puntos usados en el gráfico. Las métricas usan todos los datos.",
    )
    return parser.parse_args()


def normalize_quaternions(q: np.ndarray) -> np.ndarray:
    norms = np.linalg.norm(q, axis=-1, keepdims=True)
    return q / norms


def quat_to_rotmat_batch(q: np.ndarray) -> np.ndarray:
    """Matrices de rotación (N,3,3) a partir de cuaterniones (N,4) en orden [w,x,y,z]."""
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


def orientation_to_projected_gravity(q: np.ndarray) -> np.ndarray:
    """g_body(t) = R(q(t))^T @ g_world, para cada fila de q (N,4) -> (N,3)."""
    rot_mats = quat_to_rotmat_batch(q)
    return np.einsum("nij,j->ni", rot_mats.transpose(0, 2, 1), G_WORLD)


def load_data(csv_path: Path) -> tuple[np.ndarray, dict[str, np.ndarray], np.ndarray]:
    if not csv_path.exists():
        raise FileNotFoundError(f"No existe el archivo: {csv_path}")
    df = pd.read_csv(csv_path)
    missing = [c for c in REQUIRED_COLUMNS if c not in df.columns]
    if missing:
        raise ValueError(f"{csv_path}: faltan columnas {missing}")

    timestamp = pd.to_numeric(df["timestamp"], errors="coerce").to_numpy(dtype=float)
    if not np.all(np.isfinite(timestamp)):
        raise ValueError("La columna timestamp contiene valores no numéricos o NaN.")
    time_s = timestamp - timestamp[0]

    mocap_q = normalize_quaternions(
        df[["mocap_qw", "mocap_qx", "mocap_qy", "mocap_qz"]].to_numpy(dtype=float)
    )
    filter_q = {
        name: normalize_quaternions(
            df[[f"{name}_qw", f"{name}_qx", f"{name}_qy", f"{name}_qz"]].to_numpy(dtype=float)
        )
        for name in FILTERS
    }
    return mocap_q, filter_q, time_s


def estimate_rotation(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    """
    Resuelve con solución cerrada (Kabsch/Procrustes ortogonal, SVD):
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


def save_projected_gravity_plot(
    time_s: np.ndarray,
    g_mocap: np.ndarray,
    g_filters: dict[str, np.ndarray],
    max_points: int,
    output_path: Path,
) -> None:
    axes_names = ("x", "y", "z")
    indices = choose_plot_indices(len(time_s), max_points)

    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)

    for i, (axis, axis_name) in enumerate(zip(axes, axes_names)):
        axis.plot(time_s[indices], g_mocap[indices, i], label="Mocap")
        for name, g_filter in g_filters.items():
            axis.plot(time_s[indices], g_filter[indices, i], label=name.capitalize(), alpha=0.8)
        axis.set_ylabel(f"g_{axis_name} [m/s²]")
        axis.set_title(f"Gravedad proyectada sobre el eje {axis_name.upper()}")
        axis.grid(True, alpha=0.3)
        axis.legend()

    axes[-1].set_xlabel("Tiempo [s]")
    fig.suptitle("Gravedad proyectada: Mocap vs. Kalman vs. Madgwick (corregidos con R propia)")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    args = parse_args()

    mocap_q, filter_q, time_s = load_data(args.csv_path)
    g_mocap = orientation_to_projected_gravity(mocap_q)

    n_rows = len(time_s)
    fit_slice = slice(None, args.align_samples)
    print(f"Muestras: {n_rows}  |  ajuste de R con: "
          f"{'toda la trayectoria' if args.align_samples is None else f'primeras {args.align_samples}'}")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    g_filters_corrected: dict[str, np.ndarray] = {}
    for name in FILTERS:
        g_raw = orientation_to_projected_gravity(filter_q[name])
        R = estimate_rotation(g_raw[fit_slice], g_mocap[fit_slice])
        g_corrected = g_raw @ R.T   # fila i = R @ g_raw[i]
        g_filters_corrected[name] = g_corrected

        output_path = args.output_dir / f"grav_R_{name}_to_mocap.npy"
        np.save(output_path, R)

        print(f"\n[{name}] R (gravedad {name} -> Mocap):")
        print(np.array2string(R, precision=6, suppress_small=True))
        print(f"  det(R)          = {np.linalg.det(R):.6f}  (debe ser ~1)")
        print(f"  ||R^T R - I||   = {np.linalg.norm(R.T @ R - np.eye(3)):.2e}  (debe ser ~0)")
        print(f"  Ángulo de R     = {rotation_angle_deg(R):.2f} deg")
        print(f"  RMSE sin corregir (total):   {rmse(g_mocap, g_raw):.4f} m/s²")
        print(f"  RMSE corregido    (total):   {rmse(g_mocap, g_corrected):.4f} m/s²")
        print(f"  RMSE corregido    (ventana): {rmse(g_mocap[fit_slice], g_corrected[fit_slice]):.4f} m/s²")
        print(f"  R guardada en: {output_path.resolve()}")

    args.plot_output.parent.mkdir(parents=True, exist_ok=True)
    save_projected_gravity_plot(
        time_s, g_mocap, g_filters_corrected, args.max_plot_points, args.plot_output
    )
    print(f"\nGráfico guardado en: {args.plot_output.resolve()}")


if __name__ == "__main__":
    main()
