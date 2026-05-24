#!/usr/bin/env python3
"""Comparador simple de 2 o 3 analysis_report.json

Genera gráficas (PNG) y un CSV resumen con las métricas principales:
- tiempo total
- distancia recorrida
- sobrerrecorrido
- velocidad media
- velocidad máxima
- eficiencia
- precisión final

Uso:
    python3 compare_reports.py report1.json report2.json [report3.json] [--labels a,b[,c]] [--out-dir out]
"""
import json
import sys
from pathlib import Path
from typing import Dict, Any, List, Tuple
import csv
import time

try:
    import matplotlib.pyplot as plt
except Exception:
    plt = None


METRIC_KEYS = {
    "time": ["total_time_all_segments", "total_time_seconds"],
    "distance": ["total_distance_all_segments", "distance_traveled_meters"],
    "overshoot": ["overshoot_meters"],
    "mean_speed": ["mean_velocity_ms", "mean_cmd_velocity_ms"],
    "max_speed": ["max_velocity_ms"],
    "efficiency": ["efficiency"],
    "precision": ["real_precision_meters"],
}


def display_label(name: str) -> str:
    lower_name = name.lower()
    if "mppi" in lower_name:
        return "mppi"
    if "mpc" in lower_name:
        return "mpc"
    if "easynav" in lower_name:
        return "serest"
    if "nav2" in lower_name:
        return "nav2"
    return name


def load_report(path: Path) -> Dict[str, Any]:
    with open(path, "r") as f:
        return json.load(f)


def extract_metrics(report: Dict[str, Any]) -> Dict[str, float]:
    segments = report.get("segments", [])

    # total_time
    total_time = report.get("total_time_all_segments")
    if total_time is None:
        total_time = sum(s.get("total_time_seconds", 0.0) for s in segments)

    # total_distance
    total_distance = report.get("total_distance_all_segments")
    if total_distance is None:
        total_distance = sum(s.get("distance_traveled_meters", 0.0) for s in segments)

    # overshoot: sum of overshoots
    overshoot = sum(s.get("overshoot_meters", 0.0) for s in segments)

    # mean speed: prefer total_distance/total_time
    mean_speed = (total_distance / total_time) if total_time > 0 else 0.0

    # max speed across segments
    max_speed = 0.0
    for s in segments:
        max_speed = max(max_speed, s.get("max_velocity_ms", 0.0))

    # efficiency: weighted by distance
    efficiency = 0.0
    if total_distance > 0:
        efficiency = sum(s.get("efficiency", 0.0) * s.get("distance_traveled_meters", 0.0) for s in segments) / total_distance
    else:
        efficiency = float(segments[0].get("efficiency", 0.0)) if segments else 0.0

    # precision: take last segment value if available
    precision = None
    if segments:
        precision = segments[-1].get("real_precision_meters")
    if precision is None:
        precision = min((s.get("real_precision_meters", float("inf")) for s in segments), default=0.0)

    return {
        "time_s": float(total_time),
        "distance_m": float(total_distance),
        "overshoot_m": float(overshoot),
        "mean_speed_ms": float(mean_speed),
        "max_speed_ms": float(max_speed),
        "efficiency": float(efficiency),
        "precision_m": float(precision),
    }


def plot_comparison(labels: List[str], metrics: List[Dict[str, float]], out_dir: Path) -> None:
    if plt is None:
        print("matplotlib no disponible. Instala con: pip install matplotlib")
        return

    keys = ["time_s", "distance_m", "overshoot_m", "mean_speed_ms", "max_speed_ms", "efficiency", "precision_m"]
    nice = {
        "time_s": "Tiempo (s)",
        "distance_m": "Distancia (m)",
        "overshoot_m": "Sobrerrecorrido (m)",
        "mean_speed_ms": "Vel. media (m/s)",
        "max_speed_ms": "Vel. máx (m/s)",
        "efficiency": "Eficiencia",
        "precision_m": "Precisión (m)",
    }

    # Crear figura con subplots
    fig, axes = plt.subplots(2, 4, figsize=(16, 8))
    axes = axes.flatten()

    for i, k in enumerate(keys):
        ax = axes[i]
        values = [m[k] for m in metrics]
        colors = ["#C03950", "#3EAFCC", "#bb6de3"]
        ax.bar(labels, values, color=colors[: len(labels)])
        ax.set_title(nice.get(k, k))
        if k == "efficiency":
            ax.set_ylim(0, 1)
        ax.grid(True, linestyle="--", alpha=0.3)

    # Eliminar el último subplot vacio si existe
    if len(keys) < len(axes):
        for ax in axes[len(keys):]:
            fig.delaxes(ax)

    fig.tight_layout()
    out_png = out_dir / f"comparison_{int(time.time())}.png"
    fig.savefig(out_png)
    print(f"Gráfica guardada en: {out_png}")


def save_csv(labels: List[str], metrics: List[Dict[str, float]], out_dir: Path) -> None:
    out_csv = out_dir / "comparison_summary.csv"
    keys = ["time_s", "distance_m", "overshoot_m", "mean_speed_ms", "max_speed_ms", "efficiency", "precision_m"]
    with open(out_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["label"] + keys)
        for label, m in zip(labels, metrics):
            writer.writerow([label] + [m[k] for k in keys])
    print(f"CSV guardado en: {out_csv}")


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)

    # required first two reports
    reports = [Path(sys.argv[1]), Path(sys.argv[2])]

    # optional third report
    if len(sys.argv) >= 4 and not sys.argv[3].startswith("--"):
        reports.append(Path(sys.argv[3]))

    labels = None
    out_dir = Path("comparisons")

    # Parse optional args (remaining argv entries)
    for arg in sys.argv[3:]:
        if arg.startswith("--labels="):
            labels = arg.split("=", 1)[1].split(",")
        elif arg.startswith("--out-dir="):
            out_dir = Path(arg.split("=", 1)[1])

    # Default labels from file paths if not provided
    if labels is None:
        labels = [display_label(p.parent.name or p.stem) for p in reports]
    # Ensure we have as many labels as reports (truncate or pad)
    if len(labels) < len(reports):
        labels = labels + [f"report_{i+1}" for i in range(len(labels), len(reports))]
    labels = [display_label(label) for label in labels[: len(reports)]]

    out_dir.mkdir(parents=True, exist_ok=True)

    # Load and extract metrics for all reports
    loaded = [load_report(p) for p in reports]
    metrics = [extract_metrics(r) for r in loaded]

    save_csv(labels, metrics, out_dir)
    plot_comparison(labels, metrics, out_dir)


if __name__ == "__main__":
    main()
