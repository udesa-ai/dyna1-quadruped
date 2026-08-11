"""
Estima R, la matriz de rotación que lleva las velocidades angulares medidas
por la IMU al frame de OptiTrack/Mocap:

    omega_mocap ≈ R @ omega_imu

Resuelve mínimos cuadrados con la restricción de que R sea una matriz de
rotación (R^T R = I, det(R) = 1) - es el problema de Procrustes ortogonal
(Wahba), resuelto en forma cerrada vía SVD (algoritmo de Kabsch).

Uso:
    python3 estimate_R.py archivo.csv [archivo2.csv ...]

Opcional:
    python3 estimate_R.py archivo.csv --output R.npy
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
REQUIRED_COLUMNS = ["imu_wx", "imu_wy", "imu_wz", "mocap_wx", "mocap_wy", "mocap_wz"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Estima la rotación IMU->Mocap de velocidades angulares por mínimos cuadrados."
    )
    parser.add_argument(
        "csv_paths", type=Path, nargs="*", default=[PATH_CSV],
        help=f"Uno o más CSV con columnas imu_w*/mocap_w*. Default: {PATH_CSV}",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=SAVE_DIR / "ang_vel_R_imu_to_mocap.npy",
        help="Ruta donde guardar la matriz R estimada (.npy).",
    )
    parser.add_argument(
        "--plot-output",
        type=Path,
        default=SAVE_DIR / "ang_vel_R_calibration_plot.png",
        help="Ruta donde guardar el gráfico de velocidad angular (Mocap vs IMU crudo/corregido).",
    )
    parser.add_argument(
        "--max-plot-points",
        type=int,
        default=12000,
        help="Máximo de puntos usados en el gráfico. Las métricas usan todos los datos.",
    )
    return parser.parse_args()


def load_angular_velocities(csv_paths: list[Path]) -> tuple[np.ndarray, np.ndarray]:
    imu_chunks = []
    mocap_chunks = []
    for csv_path in csv_paths:
        if not csv_path.exists():
            raise FileNotFoundError(f"No existe el archivo: {csv_path}")
        df = pd.read_csv(csv_path)
        missing = [c for c in REQUIRED_COLUMNS if c not in df.columns]
        if missing:
            raise ValueError(f"{csv_path}: faltan columnas {missing}")
        imu_chunks.append(df[["imu_wx", "imu_wy", "imu_wz"]].to_numpy(dtype=float))
        mocap_chunks.append(df[["mocap_wx", "mocap_wy", "mocap_wz"]].to_numpy(dtype=float))

    imu_w = np.concatenate(imu_chunks, axis=0)
    mocap_w = np.concatenate(mocap_chunks, axis=0)

    valid = np.all(np.isfinite(imu_w), axis=1) & np.all(np.isfinite(mocap_w), axis=1)
    return imu_w[valid], mocap_w[valid]


def estimate_rotation(imu_w: np.ndarray, mocap_w: np.ndarray) -> np.ndarray:
    """
    Resuelve con solución cerrada:
        min_R  sum_i || mocap_w_i - R @ imu_w_i ||^2   s.a. R^T R = I, det(R) = 1
    """
    H = imu_w.T @ mocap_w
    U, _, Vt = np.linalg.svd(H)
    V = Vt.T
    d = np.sign(np.linalg.det(V @ U.T))
    D = np.diag([1.0, 1.0, d])
    R = V @ D @ U.T
    return R


def rmse(reference_w: np.ndarray, estimate_w: np.ndarray) -> float:
    error = estimate_w - reference_w
    return float(np.sqrt(np.mean(np.sum(error**2, axis=1))))


def rotation_angle_deg(R: np.ndarray) -> float:
    """Ángulo de la rotación representada por R, para ver qué tan lejos está de la identidad."""
    cos_angle = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(cos_angle)))


def choose_plot_indices(n_rows: int, max_points: int) -> np.ndarray:
    if n_rows <= max_points:
        return np.arange(n_rows)
    return np.linspace(0, n_rows - 1, max_points, dtype=int)


def save_angular_velocity_plot(imu_w: np.ndarray, predicted_w: np.ndarray, mocap_w: np.ndarray, max_points: int, output_path: Path) -> None:
    axes_names = ("x", "y", "z")
    indices = choose_plot_indices(len(mocap_w), max_points)
    sample = indices

    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)

    for i, (axis, axis_name) in enumerate(zip(axes, axes_names)):
        axis.plot(sample, mocap_w[indices, i], label=f"Mocap ω{axis_name}")
        axis.plot(sample, imu_w[indices, i], label=f"IMU crudo ω{axis_name}", alpha=0.6)
        axis.plot(sample, predicted_w[indices, i], label=f"IMU corregido (R·ω) {axis_name}", alpha=0.9)
        axis.set_ylabel("rad/s")
        axis.set_title(f"Velocidad angular sobre el eje {axis_name.upper()}")
        axis.grid(True, alpha=0.3)
        axis.legend()

    axes[-1].set_xlabel("Muestra")
    fig.suptitle("Velocidad angular: Mocap vs. IMU (crudo y corregido con R)")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    args = parse_args()

    imu_w, mocap_w = load_angular_velocities(args.csv_paths)
    if len(imu_w) < 3:
        raise ValueError("Se necesitan al menos 3 muestras válidas para estimar R.")

    R = estimate_rotation(imu_w, mocap_w)
    predicted_w = imu_w @ R.T   # fila i = R @ imu_w[i]

    print(f"Muestras usadas: {len(imu_w)}")
    print("\nR (IMU -> Mocap):")
    print(np.array2string(R, precision=6, suppress_small=True))
    print(f"\ndet(R)          = {np.linalg.det(R):.6f}  (debe ser ~1)")
    print(f"||R^T R - I||   = {np.linalg.norm(R.T @ R - np.eye(3)):.2e}  (debe ser ~0)")
    print(f"Ángulo de R     = {rotation_angle_deg(R):.2f} deg")
    print(f"\nRMSE sin calibrar (omega_imu   vs omega_mocap): {rmse(mocap_w, imu_w):.4f} rad/s")
    print(f"RMSE calibrado    (R@omega_imu vs omega_mocap): {rmse(mocap_w, predicted_w):.4f} rad/s")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.save(args.output, R)
    print(f"\nR guardada en: {args.output.resolve()}")

    args.plot_output.parent.mkdir(parents=True, exist_ok=True)
    save_angular_velocity_plot(
        imu_w, predicted_w, mocap_w, args.max_plot_points, args.plot_output
    )
    print(f"Gráfico guardado en: {args.plot_output.resolve()}")


if __name__ == "__main__":
    main()
