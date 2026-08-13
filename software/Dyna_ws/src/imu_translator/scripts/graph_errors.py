import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


DATA_DIR = Path.cwd() / "src" / "imu_translator" / "data" / "rotation"


def load_gravity_data(csv_path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Carga datos de gravedad proyectada desde CSV."""
    df = pd.read_csv(csv_path)
    mocap_g = df[['mocap_gx', 'mocap_gy', 'mocap_gz']].values
    kalman_g = df[['kalman_offset_gx', 'kalman_offset_gy', 'kalman_offset_gz']].values
    madgwick_g = df[['madgwick_offset_gx', 'madgwick_offset_gy', 'madgwick_offset_gz']].values
    return mocap_g, kalman_g, madgwick_g


def load_angular_velocity_data(csv_path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Carga datos de velocidad angular desde CSV."""
    df = pd.read_csv(csv_path)
    imu_w = df[['imu_wx_corrected', 'imu_wy_corrected', 'imu_wz_corrected']].values
    mocap_w = df[['mocap_wx', 'mocap_wy', 'mocap_wz']].values
    return imu_w, mocap_w


def plot_error_histograms(errors: np.ndarray, titles: list[str], ylabel: str, output_path: Path) -> None:
    """Grafica histogramas de errores por componente (x, y, z)."""
    fig, axes = plt.subplots(1, 3, figsize=(15, 4))
    axes_names = ('x', 'y', 'z')

    for i, (ax, axis_name, title) in enumerate(zip(axes, axes_names, titles)):
        error = errors[:, i]
        mean_error = np.mean(error)
        std_error = np.std(error)
        rmse = np.sqrt(np.mean(error**2))

        ax.hist(error, bins=50, alpha=0.7, edgecolor='black')
        ax.axvline(mean_error, color='red', linestyle='--', linewidth=2, label=f'Media: {mean_error:.4f}')
        ax.axvline(0, color='green', linestyle='-', linewidth=1, alpha=0.5, label='Cero')
        ax.set_xlabel(f'Error en {axis_name.upper()} [{ylabel}]')
        ax.set_ylabel('Frecuencia')
        ax.set_title(title)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=9)

        stats_text = f'σ={std_error:.4f}\nRMSE={rmse:.4f}'
        ax.text(0.98, 0.97, stats_text, transform=ax.transAxes,
                verticalalignment='top', horizontalalignment='right',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5), fontsize=9)

    fig.tight_layout()
    fig.savefig(output_path, dpi=160, bbox_inches='tight')
    plt.close(fig)


def plot_combined_error_comparison(gravity_errors: np.ndarray, angular_errors: np.ndarray, output_path: Path) -> None:
    """Grafica comparación de errores de gravedad vs velocidad angular."""
    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    axes_names = ('x', 'y', 'z')

    for i, axis_name in enumerate(axes_names):
        g_error = gravity_errors[:, i]
        w_error = angular_errors[:, i]

        ax_g = axes[0, i]
        ax_w = axes[1, i]

        ax_g.hist(g_error, bins=50, alpha=0.7, color='blue', edgecolor='black')
        ax_g.axvline(np.mean(g_error), color='red', linestyle='--', linewidth=2)
        ax_g.set_title(f'Error Gravedad - Eje {axis_name.upper()}')
        ax_g.set_ylabel('Frecuencia')
        ax_g.grid(True, alpha=0.3)
        ax_g.text(0.98, 0.97, f'μ={np.mean(g_error):.4f}\nσ={np.std(g_error):.4f}',
                 transform=ax_g.transAxes, verticalalignment='top', horizontalalignment='right',
                 bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5), fontsize=9)

        ax_w.hist(w_error, bins=50, alpha=0.7, color='orange', edgecolor='black')
        ax_w.axvline(np.mean(w_error), color='red', linestyle='--', linewidth=2)
        ax_w.set_title(f'Error Vel. Angular - Eje {axis_name.upper()}')
        ax_w.set_xlabel(f'Error en {axis_name.upper()}')
        ax_w.set_ylabel('Frecuencia')
        ax_w.grid(True, alpha=0.3)
        ax_w.text(0.98, 0.97, f'μ={np.mean(w_error):.4f}\nσ={np.std(w_error):.4f}',
                 transform=ax_w.transAxes, verticalalignment='top', horizontalalignment='right',
                 bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.5), fontsize=9)

    fig.suptitle('Comparación de Errores: Gravedad Proyectada vs Velocidad Angular')
    fig.tight_layout()
    fig.savefig(output_path, dpi=160, bbox_inches='tight')
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description='Grafica histogramas de errores de gravedad y velocidad angular.')
    parser.add_argument('--gravity-csv', type=Path, default=DATA_DIR / 'projected_gravity_fixed_kalman.csv',
                       help='CSV con datos de gravedad proyectada.')
    parser.add_argument('--angular-csv', type=Path, default=DATA_DIR / 'angular_velocity_comparison.csv',
                       help='CSV con datos de velocidad angular.')
    parser.add_argument('--output-dir', type=Path, default=DATA_DIR,
                       help='Carpeta donde guardar los gráficos.')
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)

    print(f"Cargando datos de gravedad: {args.gravity_csv}")
    mocap_g, kalman_g, madgwick_g = load_gravity_data(args.gravity_csv)

    print(f"Cargando datos de velocidad angular: {args.angular_csv}")
    imu_w, mocap_w = load_angular_velocity_data(args.angular_csv)

    # Calcular errores
    kalman_error_g = kalman_g - mocap_g
    madgwick_error_g = madgwick_g - mocap_g
    imu_error_w = imu_w - mocap_w

    # Graficar errores de gravedad
    gravity_output = args.output_dir / 'gravity_errors_histogram.png'
    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    axes_names = ('x', 'y', 'z')

    for i, axis_name in enumerate(axes_names):
        kalman_err = kalman_error_g[:, i]
        madgwick_err = madgwick_error_g[:, i]

        ax_kalman = axes[0, i]
        ax_madgwick = axes[1, i]

        ax_kalman.hist(kalman_err, bins=50, alpha=0.7, color='blue', edgecolor='black')
        ax_kalman.axvline(np.mean(kalman_err), color='red', linestyle='--', linewidth=2)
        ax_kalman.axvline(0, color='green', linestyle='-', linewidth=1, alpha=0.5)
        ax_kalman.set_title(f'Error Kalman Gravedad - Eje {axis_name.upper()}')
        ax_kalman.set_ylabel('Frecuencia')
        ax_kalman.grid(True, alpha=0.3)
        stats = f'μ={np.mean(kalman_err):.4f}\nσ={np.std(kalman_err):.4f}'
        ax_kalman.text(0.98, 0.97, stats, transform=ax_kalman.transAxes,
                      verticalalignment='top', horizontalalignment='right',
                      bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5), fontsize=9)

        ax_madgwick.hist(madgwick_err, bins=50, alpha=0.7, color='orange', edgecolor='black')
        ax_madgwick.axvline(np.mean(madgwick_err), color='red', linestyle='--', linewidth=2)
        ax_madgwick.axvline(0, color='green', linestyle='-', linewidth=1, alpha=0.5)
        ax_madgwick.set_title(f'Error Madgwick Gravedad - Eje {axis_name.upper()}')
        ax_madgwick.set_xlabel(f'Error en {axis_name.upper()} [m/s²]')
        ax_madgwick.set_ylabel('Frecuencia')
        ax_madgwick.grid(True, alpha=0.3)
        stats = f'μ={np.mean(madgwick_err):.4f}\nσ={np.std(madgwick_err):.4f}'
        ax_madgwick.text(0.98, 0.97, stats, transform=ax_madgwick.transAxes,
                        verticalalignment='top', horizontalalignment='right',
                        bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.5), fontsize=9)

    fig.suptitle('Errores de Gravedad Proyectada respecto a Mocap')
    fig.tight_layout()
    fig.savefig(gravity_output, dpi=160, bbox_inches='tight')
    plt.close(fig)
    print(f"Gráfico de errores de gravedad guardado en: {gravity_output.resolve()}")

    # Graficar errores de velocidad angular
    angular_output = args.output_dir / 'angular_velocity_errors_histogram.png'
    plot_error_histograms(imu_error_w, [f'Error IMU Vel. Angular - Eje {n.upper()}' for n in axes_names],
                         'rad/s', angular_output)
    print(f"Gráfico de errores de velocidad angular guardado en: {angular_output.resolve()}")

    # Graficar comparación combinada
    combined_output = args.output_dir / 'combined_errors_comparison.png'
    plot_combined_error_comparison(kalman_error_g, imu_error_w, combined_output)
    print(f"Gráfico de comparación combinada guardado en: {combined_output.resolve()}")

    # Imprimir estadísticas
    print("\n=== ESTADÍSTICAS DE ERROR ===")
    print("\nGravedad Proyectada (Kalman con offset):")
    for i, name in enumerate(axes_names):
        err = kalman_error_g[:, i]
        print(f"  Eje {name.upper()}: μ={np.mean(err):8.4f} m/s² | σ={np.std(err):8.4f} | RMSE={np.sqrt(np.mean(err**2)):8.4f}")

    print("\nGravedad Proyectada (Madgwick con offset):")
    for i, name in enumerate(axes_names):
        err = madgwick_error_g[:, i]
        print(f"  Eje {name.upper()}: μ={np.mean(err):8.4f} m/s² | σ={np.std(err):8.4f} | RMSE={np.sqrt(np.mean(err**2)):8.4f}")

    print("\nVelocidad Angular (IMU corregido):")
    for i, name in enumerate(axes_names):
        err = imu_error_w[:, i]
        print(f"  Eje {name.upper()}: μ={np.mean(err):8.4f} rad/s | σ={np.std(err):8.4f} | RMSE={np.sqrt(np.mean(err**2)):8.4f}")


if __name__ == "__main__":
    main()
