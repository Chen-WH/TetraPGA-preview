#!/usr/bin/env python3
"""Render and plot one Stanford TidyBot closed-loop MPC trajectory.

The script intentionally avoids Python Pinocchio because the current machine's
Pinocchio Python bindings are built against NumPy 1.x while the active Python
environment uses NumPy 2.x. MuJoCo is used both for kinematics and rendering.
"""

from __future__ import annotations

import argparse
import csv
import datetime as _dt
import math
import os
from pathlib import Path
from typing import Iterable

os.environ.setdefault("MPLCONFIGDIR", "/tmp/tetrapga_mpl_cache")
os.environ.setdefault("MUJOCO_GL", "egl")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CYCLES_CSV = (
    REPO_ROOT
    / "reviewer_revision_experiments"
    / "04_closed_loop_mpc_metrics"
    / "reference_batch_20260628_223000"
    / "reference_tidybot_tetrapga_cycles.csv"
)
DEFAULT_MJCF = (
    Path("/home/chenwh/ros2_ws/src/robot-assets")
    / "stanford_tidybot"
    / "mjcf"
    / "scene.xml"
)
DEFAULT_OUTPUT_ROOT = (
    REPO_ROOT
    / "reviewer_revision_experiments"
    / "09_tidybot_trajectory_visualization"
)
PLANNED_JOINTS = [
    "joint_x",
    "joint_y",
    "joint_th",
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
    "joint_7",
]


def parse_vector(value: str) -> np.ndarray:
    if value is None:
        return np.array([], dtype=float)
    value = value.strip()
    if not value:
        return np.array([], dtype=float)
    return np.fromstring(value, sep=" ", dtype=float)


def load_rows(path: Path, state_column: str, max_rows: int | None) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        if state_column not in (reader.fieldnames or []):
            raise ValueError(f"State column '{state_column}' is not present in {path}")
        for raw in reader:
            q = parse_vector(raw[state_column])
            if q.size != len(PLANNED_JOINTS):
                raise ValueError(
                    f"Expected {len(PLANNED_JOINTS)} values in column '{state_column}', "
                    f"got {q.size} at t={raw.get('t')}"
                )
            rows.append(
                {
                    "t": float(raw["t"]),
                    "q": q,
                    "q_ref": parse_vector(raw.get("q_ref", "")),
                    "tracking_error": float(raw.get("tracking_error", "nan")),
                    "solve_time_ms": float(raw.get("solve_time_ms", "nan")),
                    "iterations": int(float(raw.get("iterations", "0"))),
                    "converged": int(float(raw.get("converged", "0"))),
                }
            )
            if max_rows is not None and len(rows) >= max_rows:
                break
    if not rows:
        raise ValueError(f"No trajectory rows loaded from {path}")
    return rows


def joint_qpos_addresses(model: mujoco.MjModel, joint_names: Iterable[str]) -> list[int]:
    addresses: list[int] = []
    for name in joint_names:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise ValueError(f"Joint not found in MuJoCo model: {name}")
        addresses.append(int(model.jnt_qposadr[joint_id]))
    return addresses


def home_qpos(model: mujoco.MjModel) -> np.ndarray:
    qpos = np.zeros(model.nq, dtype=float)
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if key_id >= 0:
        qpos[:] = model.key_qpos[key_id]
    return qpos


def set_planned_qpos(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    base_qpos: np.ndarray,
    addresses: list[int],
    q: np.ndarray,
) -> None:
    data.qpos[:] = base_qpos
    data.qvel[:] = 0.0
    for value, address in zip(q, addresses):
        data.qpos[address] = value
    mujoco.mj_forward(model, data)


def collect_trajectory(
    model: mujoco.MjModel,
    rows: list[dict[str, object]],
    state_column: str,
    site_name: str,
) -> tuple[np.ndarray, list[dict[str, float]], np.ndarray, list[int]]:
    data = mujoco.MjData(model)
    qpos0 = home_qpos(model)
    addresses = joint_qpos_addresses(model, PLANNED_JOINTS)

    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
    if site_id < 0:
        raise ValueError(f"Site not found in MuJoCo model: {site_name}")
    base_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    if base_body_id < 0:
        raise ValueError("Body 'base_link' not found in MuJoCo model")

    records: list[dict[str, float]] = []
    q_matrix = []
    for row in rows:
        q = np.asarray(row["q"], dtype=float)
        set_planned_qpos(model, data, qpos0, addresses, q)
        site = np.array(data.site_xpos[site_id], dtype=float)
        base = np.array(data.xpos[base_body_id], dtype=float)
        q_matrix.append(q)
        records.append(
            {
                "t": float(row["t"]),
                "base_x": float(base[0]),
                "base_y": float(base[1]),
                "base_z": float(base[2]),
                "base_yaw": float(q[2]),
                "site_x": float(site[0]),
                "site_y": float(site[1]),
                "site_z": float(site[2]),
                "tracking_error": float(row["tracking_error"]),
                "solve_time_ms": float(row["solve_time_ms"]),
                "iterations": float(row["iterations"]),
                "converged": float(row["converged"]),
            }
        )
    return np.array(q_matrix), records, qpos0, addresses


def write_trajectory_csv(path: Path, records: list[dict[str, float]]) -> None:
    fields = [
        "t",
        "base_x",
        "base_y",
        "base_z",
        "base_yaw",
        "site_x",
        "site_y",
        "site_z",
        "tracking_error",
        "solve_time_ms",
        "iterations",
        "converged",
    ]
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in records:
            writer.writerow(row)


def make_camera(records: list[dict[str, float]]) -> mujoco.MjvCamera:
    xs = np.array([r["base_x"] for r in records] + [r["site_x"] for r in records])
    ys = np.array([r["base_y"] for r in records] + [r["site_y"] for r in records])
    zs = np.array([r["site_z"] for r in records])
    extent = max(float(np.ptp(xs)), float(np.ptp(ys)), 0.9)
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.azimuth = 135.0
    cam.elevation = -25.0
    cam.distance = max(1.6, 1.25 * extent + 0.9)
    cam.lookat[:] = [0.5 * (xs.min() + xs.max()), 0.5 * (ys.min() + ys.max()), max(0.35, float(np.mean(zs)))]
    return cam


def render_state(
    model: mujoco.MjModel,
    qpos0: np.ndarray,
    addresses: list[int],
    q: np.ndarray,
    camera: mujoco.MjvCamera,
    width: int,
    height: int,
) -> Image.Image:
    data = mujoco.MjData(model)
    set_planned_qpos(model, data, qpos0, addresses, q)
    model.vis.global_.offwidth = max(int(model.vis.global_.offwidth), int(width))
    model.vis.global_.offheight = max(int(model.vis.global_.offheight), int(height))
    renderer = mujoco.Renderer(model, height=height, width=width)
    try:
        renderer.update_scene(data, camera=camera)
        image = renderer.render()
    finally:
        renderer.close()
    return Image.fromarray(image)


def label_image(image: Image.Image, label: str) -> Image.Image:
    out = image.copy()
    draw = ImageDraw.Draw(out, "RGBA")
    try:
        font = ImageFont.truetype("DejaVuSans.ttf", 24)
    except OSError:
        font = ImageFont.load_default()
    draw.rounded_rectangle((12, 12, 210, 54), radius=8, fill=(255, 255, 255, 190))
    draw.text((24, 22), label, fill=(20, 28, 35, 255), font=font)
    return out


def save_montage(images: list[Image.Image], labels: list[str], path: Path) -> None:
    labeled = [label_image(img, label) for img, label in zip(images, labels)]
    w, h = labeled[0].size
    canvas = Image.new("RGB", (2 * w, 2 * h), "white")
    for i, img in enumerate(labeled):
        canvas.paste(img.convert("RGB"), ((i % 2) * w, (i // 2) * h))
    canvas.save(path, quality=95)


def render_snapshots(
    model: mujoco.MjModel,
    q_matrix: np.ndarray,
    qpos0: np.ndarray,
    addresses: list[int],
    records: list[dict[str, float]],
    output_dir: Path,
    width: int,
    height: int,
) -> tuple[Path | None, Path | None]:
    if q_matrix.shape[0] < 4:
        indices = list(range(q_matrix.shape[0]))
    else:
        indices = [0, q_matrix.shape[0] // 3, 2 * q_matrix.shape[0] // 3, q_matrix.shape[0] - 1]
    camera = make_camera(records)
    images: list[Image.Image] = []
    labels: list[str] = []
    for idx in indices:
        image = render_state(model, qpos0, addresses, q_matrix[idx], camera, width, height)
        images.append(image)
        labels.append(f"t = {records[idx]['t']:.1f} s")
    final_path = output_dir / "tidybot_mujoco_final.png"
    montage_path = output_dir / "tidybot_mujoco_snapshots.png"
    images[-1].save(final_path, quality=95)
    if len(images) == 4:
        save_montage(images, labels, montage_path)
    else:
        images[-1].save(montage_path, quality=95)
    return final_path, montage_path


def plot_paper_figure(
    records: list[dict[str, float]],
    final_render_path: Path | None,
    output_path: Path,
) -> None:
    t = np.array([r["t"] for r in records])
    base_x = np.array([r["base_x"] for r in records])
    base_y = np.array([r["base_y"] for r in records])
    site_x = np.array([r["site_x"] for r in records])
    site_y = np.array([r["site_y"] for r in records])
    site_z = np.array([r["site_z"] for r in records])
    err = np.array([r["tracking_error"] for r in records])
    solve = np.array([r["solve_time_ms"] for r in records])

    fig = plt.figure(figsize=(10.5, 4.7), dpi=220, constrained_layout=True)
    gs = fig.add_gridspec(2, 3, width_ratios=[1.35, 1.0, 1.0])

    ax_render = fig.add_subplot(gs[:, 0])
    if final_render_path is not None and final_render_path.exists():
        ax_render.imshow(plt.imread(final_render_path))
        ax_render.set_title("MuJoCo view at final time", fontsize=10)
    else:
        ax_render.text(0.5, 0.5, "MuJoCo render unavailable", ha="center", va="center")
    ax_render.axis("off")

    ax_xy = fig.add_subplot(gs[:, 1])
    time_color = t - t[0]
    scatter = ax_xy.scatter(site_x, site_y, c=time_color, s=7, cmap="viridis", label="end-effector")
    ax_xy.plot(base_x, base_y, color="#1f2937", linewidth=1.4, label="mobile base")
    ax_xy.scatter(site_x[0], site_y[0], marker="o", s=42, color="#16a34a", edgecolor="white", linewidth=0.6)
    ax_xy.scatter(site_x[-1], site_y[-1], marker="*", s=90, color="#dc2626", edgecolor="white", linewidth=0.6)
    ax_xy.set_aspect("equal", adjustable="box")
    ax_xy.set_xlabel("x [m]")
    ax_xy.set_ylabel("y [m]")
    ax_xy.set_title("Top-down trajectory", fontsize=10)
    ax_xy.grid(True, linewidth=0.35, alpha=0.45)
    ax_xy.legend(loc="best", fontsize=7, frameon=False)
    cbar = fig.colorbar(scatter, ax=ax_xy, fraction=0.046, pad=0.04)
    cbar.set_label("time [s]", fontsize=8)
    cbar.ax.tick_params(labelsize=7)

    ax_z = fig.add_subplot(gs[0, 2])
    ax_z.plot(t, site_z, color="#2563eb", linewidth=1.2)
    ax_z.set_ylabel("z [m]")
    ax_z.set_title("End-effector height", fontsize=10)
    ax_z.grid(True, linewidth=0.35, alpha=0.45)

    ax_metric = fig.add_subplot(gs[1, 2])
    ax_metric.plot(t, err, color="#ea580c", linewidth=1.1, label="tracking error")
    ax_metric.set_xlabel("time [s]")
    ax_metric.set_ylabel("tracking error")
    ax_metric.grid(True, linewidth=0.35, alpha=0.45)
    ax_solve = ax_metric.twinx()
    ax_solve.plot(t, solve, color="#6b7280", linewidth=0.8, alpha=0.75, label="solve time")
    ax_solve.set_ylabel("solve time [ms]")
    lines = ax_metric.get_lines() + ax_solve.get_lines()
    labels = [line.get_label() for line in lines]
    ax_metric.legend(lines, labels, loc="upper right", fontsize=7, frameon=False)

    fig.suptitle("Stanford TidyBot receding-horizon MPC trajectory", fontsize=12)
    fig.savefig(output_path, bbox_inches="tight")
    plt.close(fig)


def write_readme(
    path: Path,
    args: argparse.Namespace,
    records: list[dict[str, float]],
    render_error: str | None,
) -> None:
    duration = records[-1]["t"] - records[0]["t"]
    final_err = records[-1]["tracking_error"]
    p95_solve = float(np.percentile([r["solve_time_ms"] for r in records], 95))
    text = f"""# Stanford TidyBot trajectory visualization

Source trajectory: `{args.cycles_csv}`

State column: `{args.state_column}`

MuJoCo model: `{args.mjcf}`

Rows: {len(records)}

Duration: {duration:.3f} s

Final tracking error: {final_err:.6g}

Solve-time p95: {p95_solve:.3f} ms

Generated files:

- `tidybot_trajectory.csv`: time, mobile-base pose, end-effector site pose, and MPC metrics.
- `tidybot_mujoco_final.png`: MuJoCo final-state render.
- `tidybot_mujoco_snapshots.png`: four MuJoCo snapshots along the trajectory.
- `tidybot_trajectory_figure.png`: paper/response-letter figure.

Suggested caption:

> Stanford TidyBot closed-loop receding-horizon MPC trajectory in MuJoCo. The mobile base and Gen3 arm are planned jointly over 10 generalized coordinates; the plot shows the mobile-base path, the end-effector path, and solver/tracking metrics over a 20 s rollout.
"""
    if render_error:
        text += f"\nMuJoCo rendering fallback/error: `{render_error}`\n"
    path.write_text(text)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cycles-csv", type=Path, default=DEFAULT_CYCLES_CSV)
    parser.add_argument("--mjcf", type=Path, default=DEFAULT_MJCF)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--state-column", choices=["q", "q_cmd", "q_ref"], default="q")
    parser.add_argument("--site-name", default="pinch_site")
    parser.add_argument("--max-rows", type=int, default=None)
    parser.add_argument("--render-width", type=int, default=960)
    parser.add_argument("--render-height", type=int, default=640)
    parser.add_argument("--skip-render", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.output_dir is None:
        stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        args.output_dir = DEFAULT_OUTPUT_ROOT / f"tidybot_trajectory_{stamp}"
    args.output_dir.mkdir(parents=True, exist_ok=True)

    rows = load_rows(args.cycles_csv, args.state_column, args.max_rows)
    model = mujoco.MjModel.from_xml_path(str(args.mjcf))
    q_matrix, records, qpos0, addresses = collect_trajectory(
        model, rows, args.state_column, args.site_name
    )

    trajectory_csv = args.output_dir / "tidybot_trajectory.csv"
    write_trajectory_csv(trajectory_csv, records)

    final_render_path: Path | None = None
    montage_path: Path | None = None
    render_error: str | None = None
    if not args.skip_render:
        try:
            final_render_path, montage_path = render_snapshots(
                model,
                q_matrix,
                qpos0,
                addresses,
                records,
                args.output_dir,
                args.render_width,
                args.render_height,
            )
        except Exception as exc:  # Keep the plot usable on machines without EGL.
            render_error = f"{type(exc).__name__}: {exc}"

    figure_path = args.output_dir / "tidybot_trajectory_figure.png"
    plot_paper_figure(records, final_render_path, figure_path)
    write_readme(args.output_dir / "README.md", args, records, render_error)

    print(f"Output directory: {args.output_dir}")
    print(f"Trajectory CSV: {trajectory_csv}")
    if final_render_path:
        print(f"MuJoCo final render: {final_render_path}")
    if montage_path:
        print(f"MuJoCo snapshots: {montage_path}")
    if render_error:
        print(f"MuJoCo render warning: {render_error}")
    print(f"Paper figure: {figure_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
