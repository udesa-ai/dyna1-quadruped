#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Compara:
1) Velocidades angulares IMU vs. OptiTrack/Mocap.
2) Cuaternión Mocap vs. Kalman.
3) Cuaternión Mocap vs. Madgwick.

Además:
- normaliza los cuaterniones;
- corrige la ambigüedad de signo q == -q;
- calcula métricas;
- calcula el error angular real de orientación;
- guarda gráficos y métricas en una carpeta de resultados.

Uso:
    python3 compare_filter_validation.py archivo.csv

Opcional:
    python3 compare_filter_validation.py archivo.csv --output resultados --show
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


DATA_DIR = Path.cwd() / "src" / "imu_translator" / "data" / "comparison_results"

REQUIRED_COLUMNS = [
    "timestamp",
    "imu_wx", "imu_wy", "imu_wz",
    "mocap_wx", "mocap_wy", "mocap_wz",
    "mocap_qw", "mocap_qx", "mocap_qy", "mocap_qz",
    "kalman_qw", "kalman_qx", "kalman_qy", "kalman_qz",
    "madgwick_qw", "madgwick_qx", "madgwick_qy", "madgwick_qz",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compara IMU, Mocap, Kalman y Madgwick desde un CSV."
    )
    parser.add_argument("csv_path", type=Path, help="Ruta al archivo CSV.")
    parser.add_argument(
        "--output",
        type=Path,
        default=DATA_DIR,
        help="Carpeta donde guardar gráficos y métricas.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Mostrar los gráficos además de guardarlos.",
    )
    parser.add_argument(
        "--max-plot-points",
        type=int,
        default=12000,
        help="Máximo de puntos usados en cada gráfico. Las métricas usan todos los datos.",
    )
    return parser.parse_args()


def validate_columns(df: pd.DataFrame) -> None:
    missing = [column for column in REQUIRED_COLUMNS if column not in df.columns]
    if missing:
        raise ValueError(
            "Faltan las siguientes columnas requeridas:\n- "
            + "\n- ".join(missing)
        )


def normalized_quaternions(values: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    Normaliza cuaterniones con orden [w, x, y, z].

    Retorna:
        normalized: array (N, 4), NaN donde el cuaternión no sea válido.
        valid: máscara booleana de filas válidas.
    """
    values = np.asarray(values, dtype=float)
    norms = np.linalg.norm(values, axis=1)

    valid = np.all(np.isfinite(values), axis=1) & (norms > 1e-12)
    normalized = np.full_like(values, np.nan, dtype=float)
    normalized[valid] = values[valid] / norms[valid, None]

    return normalized, valid


def align_quaternion_sign(
    reference: np.ndarray,
    estimate: np.ndarray,
) -> np.ndarray:
    """
    Cambia el signo del estimado cuando es necesario para que quede en el
    mismo hemisferio que la referencia.

    Esto evita aparentes saltos de componentes, ya que q y -q representan
    la misma orientación.
    """
    aligned = estimate.copy()
    dots = np.sum(reference * estimate, axis=1)
    flip = np.isfinite(dots) & (dots < 0.0)
    aligned[flip] *= -1.0
    return aligned


def quaternion_angular_error_deg(
    reference: np.ndarray,
    estimate: np.ndarray,
) -> np.ndarray:
    """
    Error angular mínimo entre dos orientaciones, expresado en grados.

    Para cuaterniones unitarios:
        error = 2 * acos(|q_ref dot q_est|)
    """
    dots = np.sum(reference * estimate, axis=1)
    dots = np.clip(np.abs(dots), 0.0, 1.0)
    return np.degrees(2.0 * np.arccos(dots))


def scalar_metrics(reference: np.ndarray, estimate: np.ndarray) -> dict[str, float]:
    mask = np.isfinite(reference) & np.isfinite(estimate)
    if not np.any(mask):
        return {"mae": np.nan, "rmse": np.nan, "bias": np.nan, "correlation": np.nan}

    ref = reference[mask]
    est = estimate[mask]
    error = est - ref

    if len(ref) > 1 and np.std(ref) > 0.0 and np.std(est) > 0.0:
        correlation = float(np.corrcoef(ref, est)[0, 1])
    else:
        correlation = np.nan

    return {
        "mae": float(np.mean(np.abs(error))),
        "rmse": float(np.sqrt(np.mean(error**2))),
        "bias": float(np.mean(error)),
        "correlation": correlation,
    }


def orientation_metrics(error_deg: np.ndarray) -> dict[str, float]:
    valid = error_deg[np.isfinite(error_deg)]
    if len(valid) == 0:
        return {
            "mean_deg": np.nan,
            "rmse_deg": np.nan,
            "median_deg": np.nan,
            "p95_deg": np.nan,
            "max_deg": np.nan,
        }

    return {
        "mean_deg": float(np.mean(valid)),
        "rmse_deg": float(np.sqrt(np.mean(valid**2))),
        "median_deg": float(np.median(valid)),
        "p95_deg": float(np.percentile(valid, 95)),
        "max_deg": float(np.max(valid)),
    }


def choose_plot_indices(n_rows: int, max_points: int) -> np.ndarray:
    if n_rows <= max_points:
        return np.arange(n_rows)
    return np.linspace(0, n_rows - 1, max_points, dtype=int)


def save_angular_velocity_plot(
    time_s: np.ndarray,
    df: pd.DataFrame,
    indices: np.ndarray,
    output_path: Path,
) -> None:
    axes_names = ("x", "y", "z")
    imu_columns = ("imu_wx", "imu_wy", "imu_wz")
    mocap_columns = ("mocap_wx", "mocap_wy", "mocap_wz")

    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)

    for axis, axis_name, imu_col, mocap_col in zip(
        axes, axes_names, imu_columns, mocap_columns
    ):
        axis.plot(time_s[indices], df[imu_col].to_numpy()[indices], label=f"IMU ω{axis_name}")
        axis.plot(
            time_s[indices],
            df[mocap_col].to_numpy()[indices],
            label=f"Mocap ω{axis_name}",
            alpha=0.85,
        )
        axis.set_ylabel("rad/s")
        axis.set_title(f"Velocidad angular sobre el eje {axis_name.upper()}")
        axis.grid(True, alpha=0.3)
        axis.legend()

    axes[-1].set_xlabel("Tiempo [s]")
    fig.suptitle("Comparación de velocidad angular: IMU vs. Mocap")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160, bbox_inches="tight")
    plt.close(fig)


def save_quaternion_component_plot(
    time_s: np.ndarray,
    reference: np.ndarray,
    estimate: np.ndarray,
    estimate_name: str,
    indices: np.ndarray,
    output_path: Path,
) -> None:
    component_names = ("w", "x", "y", "z")
    fig, axes = plt.subplots(4, 1, figsize=(13, 11), sharex=True)

    for component_index, (axis, component_name) in enumerate(
        zip(axes, component_names)
    ):
        axis.plot(
            time_s[indices],
            reference[indices, component_index],
            label=f"Mocap q{component_name}",
        )
        axis.plot(
            time_s[indices],
            estimate[indices, component_index],
            label=f"{estimate_name} q{component_name}",
            alpha=0.85,
        )
        axis.set_ylabel(f"q{component_name}")
        axis.set_ylim(-1.05, 1.05)
        axis.grid(True, alpha=0.3)
        axis.legend()

    axes[-1].set_xlabel("Tiempo [s]")
    fig.suptitle(f"Comparación de cuaterniones: Mocap vs. {estimate_name}")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160, bbox_inches="tight")
    plt.close(fig)


def save_orientation_error_plot(
    time_s: np.ndarray,
    kalman_error_deg: np.ndarray,
    madgwick_error_deg: np.ndarray,
    indices: np.ndarray,
    output_path: Path,
) -> None:
    fig, axis = plt.subplots(figsize=(13, 5))

    axis.plot(
        time_s[indices],
        kalman_error_deg[indices],
        label="Error angular Kalman",
    )
    axis.plot(
        time_s[indices],
        madgwick_error_deg[indices],
        label="Error angular Madgwick",
        alpha=0.85,
    )

    axis.set_xlabel("Tiempo [s]")
    axis.set_ylabel("Error angular [grados]")
    axis.set_title("Error de orientación respecto de Mocap")
    axis.grid(True, alpha=0.3)
    axis.legend()

    fig.tight_layout()
    fig.savefig(output_path, dpi=160, bbox_inches="tight")
    plt.close(fig)


def build_metrics(
    df: pd.DataFrame,
    mocap_q: np.ndarray,
    kalman_q: np.ndarray,
    madgwick_q: np.ndarray,
    kalman_error_deg: np.ndarray,
    madgwick_error_deg: np.ndarray,
) -> pd.DataFrame:
    rows: list[dict[str, object]] = []

    for axis_name, imu_col, mocap_col in zip(
        ("x", "y", "z"),
        ("imu_wx", "imu_wy", "imu_wz"),
        ("mocap_wx", "mocap_wy", "mocap_wz"),
    ):
        metrics = scalar_metrics(
            df[mocap_col].to_numpy(dtype=float),
            df[imu_col].to_numpy(dtype=float),
        )
        rows.append(
            {
                "comparison": "angular_velocity_imu_vs_mocap",
                "signal": f"omega_{axis_name}",
                **metrics,
            }
        )

    for filter_name, estimate in (
        ("kalman", kalman_q),
        ("madgwick", madgwick_q),
    ):
        for index, component_name in enumerate(("qw", "qx", "qy", "qz")):
            metrics = scalar_metrics(mocap_q[:, index], estimate[:, index])
            rows.append(
                {
                    "comparison": f"quaternion_components_{filter_name}_vs_mocap",
                    "signal": component_name,
                    **metrics,
                }
            )

    for filter_name, error_deg in (
        ("kalman", kalman_error_deg),
        ("madgwick", madgwick_error_deg),
    ):
        metrics = orientation_metrics(error_deg)
        rows.append(
            {
                "comparison": f"orientation_error_{filter_name}_vs_mocap",
                "signal": "angle_deg",
                **metrics,
            }
        )

    return pd.DataFrame(rows)


def main() -> None:
    args = parse_args()

    if not args.csv_path.exists():
        raise FileNotFoundError(f"No existe el archivo: {args.csv_path}")

    args.output.mkdir(parents=True, exist_ok=True)

    df = pd.read_csv(args.csv_path)
    validate_columns(df)

    # Tiempo relativo para que los gráficos comiencen en t = 0 s.
    timestamp = pd.to_numeric(df["timestamp"], errors="coerce").to_numpy(dtype=float)
    if not np.all(np.isfinite(timestamp)):
        raise ValueError("La columna timestamp contiene valores no numéricos o NaN.")
    time_s = timestamp - timestamp[0]

    mocap_raw = df[
        ["mocap_qw", "mocap_qx", "mocap_qy", "mocap_qz"]
    ].to_numpy(dtype=float)
    kalman_raw = df[
        ["kalman_qw", "kalman_qx", "kalman_qy", "kalman_qz"]
    ].to_numpy(dtype=float)
    madgwick_raw = df[
        ["madgwick_qw", "madgwick_qx", "madgwick_qy", "madgwick_qz"]
    ].to_numpy(dtype=float)

    mocap_q, mocap_valid = normalized_quaternions(mocap_raw)
    kalman_q, kalman_valid = normalized_quaternions(kalman_raw)
    madgwick_q, madgwick_valid = normalized_quaternions(madgwick_raw)

    valid_kalman = mocap_valid & kalman_valid
    valid_madgwick = mocap_valid & madgwick_valid

    # Signo alineado únicamente para visualizar y comparar componentes.
    kalman_aligned = align_quaternion_sign(mocap_q, kalman_q)
    madgwick_aligned = align_quaternion_sign(mocap_q, madgwick_q)

    kalman_error_deg = np.full(len(df), np.nan)
    madgwick_error_deg = np.full(len(df), np.nan)

    kalman_error_deg[valid_kalman] = quaternion_angular_error_deg(
        mocap_q[valid_kalman],
        kalman_q[valid_kalman],
    )
    madgwick_error_deg[valid_madgwick] = quaternion_angular_error_deg(
        mocap_q[valid_madgwick],
        madgwick_q[valid_madgwick],
    )

    plot_indices = choose_plot_indices(len(df), args.max_plot_points)

    save_angular_velocity_plot(
        time_s,
        df,
        plot_indices,
        args.output / "01_angular_velocity_imu_vs_mocap.png",
    )
    save_quaternion_component_plot(
        time_s,
        mocap_q,
        kalman_aligned,
        "Kalman",
        plot_indices,
        args.output / "02_quaternion_mocap_vs_kalman.png",
    )
    save_quaternion_component_plot(
        time_s,
        mocap_q,
        madgwick_aligned,
        "Madgwick",
        plot_indices,
        args.output / "03_quaternion_mocap_vs_madgwick.png",
    )
    save_orientation_error_plot(
        time_s,
        kalman_error_deg,
        madgwick_error_deg,
        plot_indices,
        args.output / "04_orientation_error.png",
    )

    metrics_df = build_metrics(
        df,
        mocap_q,
        kalman_aligned,
        madgwick_aligned,
        kalman_error_deg,
        madgwick_error_deg,
    )
    metrics_path = args.output / "metrics.csv"
    metrics_df.to_csv(metrics_path, index=False)

    errors_df = pd.DataFrame(
        {
            "time_s": time_s,
            "kalman_orientation_error_deg": kalman_error_deg,
            "madgwick_orientation_error_deg": madgwick_error_deg,
        }
    )
    errors_df.to_csv(args.output / "orientation_errors.csv", index=False)

    print(f"Filas procesadas: {len(df)}")
    print(f"Duración aproximada: {time_s[-1]:.3f} s")
    print(f"Resultados guardados en: {args.output.resolve()}")
    print("\nMétricas de error angular:")
    print(
        metrics_df[
            metrics_df["comparison"].str.startswith("orientation_error")
        ].to_string(index=False)
    )

    if args.show:
        # Volvemos a abrir las imágenes guardadas en ventanas separadas.
        for image_name in (
            "01_angular_velocity_imu_vs_mocap.png",
            "02_quaternion_mocap_vs_kalman.png",
            "03_quaternion_mocap_vs_madgwick.png",
            "04_orientation_error.png",
        ):
            image = plt.imread(args.output / image_name)
            fig, axis = plt.subplots(figsize=(13, 7))
            axis.imshow(image)
            axis.axis("off")
            axis.set_title(image_name)
        plt.show()


if __name__ == "__main__":
    main()
