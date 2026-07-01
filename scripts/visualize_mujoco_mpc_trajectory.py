#!/usr/bin/env python3
"""Render and plot MuJoCo closed-loop MPC trajectories for revision figures."""

from __future__ import annotations

import argparse
import csv
import datetime as _dt
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

os.environ.setdefault("MPLCONFIGDIR", "/tmp/tetrapga_mpl_cache")
os.environ.setdefault("MUJOCO_GL", "egl")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont


REPO_ROOT = Path(__file__).resolve().parents[1]
ASSETS_ROOT = Path("/home/chenwh/ros2_ws/src/robot-assets")
REFERENCE_ROOT = (
    REPO_ROOT
    / "reviewer_revision_experiments"
    / "04_closed_loop_mpc_metrics"
    / "reference_batch_20260628_223000"
)
DEFAULT_OUTPUT_ROOT = (
    REPO_ROOT / "reviewer_revision_experiments" / "09_mujoco_trajectory_visualization"
)
TargetKind = Literal["site", "body", "geom"]


@dataclass(frozen=True)
class TargetSpec:
    kind: TargetKind
    name: str
    label: str


@dataclass(frozen=True)
class RobotSpec:
    robot: str
    label: str
    default_cycles_csv: Path
    mjcf: Path
    joint_names: tuple[str, ...]
    targets: tuple[TargetSpec, ...]
    primary_label: str
    camera_azimuth: float
    camera_elevation: float
    camera_distance_scale: float
    render_width: int = 960
    render_height: int = 640


ROBOT_SPECS: dict[str, RobotSpec] = {
    "ur": RobotSpec(
        robot="ur",
        label="UR10",
        default_cycles_csv=REFERENCE_ROOT / "reference_ur_tetrapga_cycles.csv",
        mjcf=ASSETS_ROOT / "ur10" / "mjcf" / "ur10.xml",
        joint_names=("joint1", "joint2", "joint3", "joint4", "joint5", "joint6"),
        targets=(TargetSpec("body", "attachment", "tool frame"),),
        primary_label="tool frame",
        camera_azimuth=135.0,
        camera_elevation=-22.0,
        camera_distance_scale=1.7,
    ),
    "stanford_tidybot": RobotSpec(
        robot="stanford_tidybot",
        label="Stanford TidyBot",
        default_cycles_csv=REFERENCE_ROOT / "reference_tidybot_tetrapga_cycles.csv",
        mjcf=ASSETS_ROOT / "stanford_tidybot" / "mjcf" / "scene.xml",
        joint_names=(
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
        ),
        targets=(TargetSpec("site", "pinch_site", "pinch site"),),
        primary_label="pinch site",
        camera_azimuth=135.0,
        camera_elevation=-25.0,
        camera_distance_scale=1.2,
    ),
    "leap_left": RobotSpec(
        robot="leap_left",
        label="LEAP Hand",
        default_cycles_csv=REFERENCE_ROOT / "reference_leap_tetrapga_cycles.csv",
        mjcf=ASSETS_ROOT / "leap_hand" / "mjcf" / "left_hand.xml",
        joint_names=(
            "if_mcp",
            "if_rot",
            "if_pip",
            "if_dip",
            "mf_mcp",
            "mf_rot",
            "mf_pip",
            "mf_dip",
            "rf_mcp",
            "rf_rot",
            "rf_pip",
            "rf_dip",
            "th_cmc",
            "th_axl",
            "th_mcp",
            "th_ipl",
        ),
        targets=(
            TargetSpec("geom", "if_tip", "index tip"),
            TargetSpec("geom", "mf_tip", "middle tip"),
            TargetSpec("geom", "rf_tip", "ring tip"),
            TargetSpec("geom", "th_tip", "thumb tip"),
        ),
        primary_label="fingertip centroid",
        camera_azimuth=130.0,
        camera_elevation=-28.0,
        camera_distance_scale=1.8,
        render_width=960,
        render_height=720,
    ),
}
ROBOT_ALIASES = {
    "ur10": "ur",
    "tidybot": "stanford_tidybot",
    "leap": "leap_left",
    "leap_hand": "leap_left",
}


def parse_vector(value: str) -> np.ndarray:
    value = (value or "").strip()
    if not value:
        return np.array([], dtype=float)
    return np.fromstring(value, sep=" ", dtype=float)


def canonical_robot(name: str) -> str:
    return ROBOT_ALIASES.get(name, name)


def infer_robot(cycles_csv: Path) -> str:
    with cycles_csv.open(newline="") as f:
        row = next(csv.DictReader(f))
    return canonical_robot(str(row["robot"]))


def load_rows(
    path: Path,
    state_column: str,
    expected_dof: int,
    max_rows: int | None,
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        if state_column not in (reader.fieldnames or []):
            raise ValueError(f"State column '{state_column}' is not present in {path}")
        for raw in reader:
            q = parse_vector(raw[state_column])
            if q.size != expected_dof:
                raise ValueError(
                    f"Expected {expected_dof} values in column '{state_column}', "
                    f"got {q.size} at t={raw.get('t')}"
                )
            rows.append(
                {
                    "t": float(raw["t"]),
                    "q": q,
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


def joint_qpos_addresses(model: mujoco.MjModel, joint_names: tuple[str, ...]) -> list[int]:
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


def target_ids(model: mujoco.MjModel, targets: tuple[TargetSpec, ...]) -> list[int]:
    ids: list[int] = []
    for target in targets:
        if target.kind == "site":
            obj = mujoco.mjtObj.mjOBJ_SITE
        elif target.kind == "body":
            obj = mujoco.mjtObj.mjOBJ_BODY
        elif target.kind == "geom":
            obj = mujoco.mjtObj.mjOBJ_GEOM
        else:
            raise ValueError(f"Unknown target kind: {target.kind}")
        target_id = mujoco.mj_name2id(model, obj, target.name)
        if target_id < 0:
            raise ValueError(f"{target.kind} target not found: {target.name}")
        ids.append(target_id)
    return ids


def target_position(
    data: mujoco.MjData,
    target: TargetSpec,
    target_id: int,
) -> np.ndarray:
    if target.kind == "site":
        return np.array(data.site_xpos[target_id], dtype=float)
    if target.kind == "body":
        return np.array(data.xpos[target_id], dtype=float)
    if target.kind == "geom":
        return np.array(data.geom_xpos[target_id], dtype=float)
    raise ValueError(f"Unknown target kind: {target.kind}")


def collect_trajectory(
    model: mujoco.MjModel,
    spec: RobotSpec,
    rows: list[dict[str, object]],
) -> tuple[np.ndarray, list[dict[str, float]], dict[str, np.ndarray], np.ndarray, list[int]]:
    data = mujoco.MjData(model)
    qpos0 = home_qpos(model)
    addresses = joint_qpos_addresses(model, spec.joint_names)
    ids = target_ids(model, spec.targets)

    target_paths = {target.label: [] for target in spec.targets}
    records: list[dict[str, float]] = []
    q_matrix = []
    for row in rows:
        q = np.asarray(row["q"], dtype=float)
        set_planned_qpos(model, data, qpos0, addresses, q)
        positions = []
        for target, target_id in zip(spec.targets, ids):
            pos = target_position(data, target, target_id)
            target_paths[target.label].append(pos)
            positions.append(pos)
        primary = np.mean(np.vstack(positions), axis=0)
        q_matrix.append(q)
        records.append(
            {
                "t": float(row["t"]),
                "primary_x": float(primary[0]),
                "primary_y": float(primary[1]),
                "primary_z": float(primary[2]),
                "tracking_error": float(row["tracking_error"]),
                "solve_time_ms": float(row["solve_time_ms"]),
                "iterations": float(row["iterations"]),
                "converged": float(row["converged"]),
            }
        )
    return (
        np.array(q_matrix),
        records,
        {label: np.vstack(points) for label, points in target_paths.items()},
        qpos0,
        addresses,
    )


def write_trajectory_csv(
    path: Path,
    records: list[dict[str, float]],
    target_paths: dict[str, np.ndarray],
) -> None:
    fields = [
        "t",
        "primary_x",
        "primary_y",
        "primary_z",
        "tracking_error",
        "solve_time_ms",
        "iterations",
        "converged",
    ]
    for label in target_paths:
        safe = label.replace(" ", "_").replace("-", "_")
        fields.extend([f"{safe}_x", f"{safe}_y", f"{safe}_z"])
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for idx, row in enumerate(records):
            out = dict(row)
            for label, values in target_paths.items():
                safe = label.replace(" ", "_").replace("-", "_")
                out[f"{safe}_x"] = float(values[idx, 0])
                out[f"{safe}_y"] = float(values[idx, 1])
                out[f"{safe}_z"] = float(values[idx, 2])
            writer.writerow(out)


def make_camera(
    model: mujoco.MjModel,
    spec: RobotSpec,
    records: list[dict[str, float]],
    target_paths: dict[str, np.ndarray],
) -> mujoco.MjvCamera:
    all_points = [np.column_stack(([r["primary_x"] for r in records], [r["primary_y"] for r in records], [r["primary_z"] for r in records]))]
    all_points.extend(target_paths.values())
    points = np.vstack(all_points)
    span = np.ptp(points, axis=0)
    extent = max(float(span[0]), float(span[1]), float(span[2]), float(model.stat.extent), 0.12)
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.azimuth = spec.camera_azimuth
    cam.elevation = spec.camera_elevation
    cam.distance = spec.camera_distance_scale * extent + 0.25
    cam.lookat[:] = np.mean(points, axis=0)
    if spec.robot == "ur":
        cam.lookat[2] = max(cam.lookat[2], 0.85)
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
    draw.rounded_rectangle((12, 12, 220, 54), radius=8, fill=(255, 255, 255, 190))
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
    spec: RobotSpec,
    q_matrix: np.ndarray,
    qpos0: np.ndarray,
    addresses: list[int],
    records: list[dict[str, float]],
    target_paths: dict[str, np.ndarray],
    output_dir: Path,
    width: int,
    height: int,
) -> tuple[Path, Path]:
    indices = [0, q_matrix.shape[0] // 3, 2 * q_matrix.shape[0] // 3, q_matrix.shape[0] - 1]
    camera = make_camera(model, spec, records, target_paths)
    images: list[Image.Image] = []
    labels: list[str] = []
    for idx in indices:
        image = render_state(model, qpos0, addresses, q_matrix[idx], camera, width, height)
        images.append(image)
        labels.append(f"t = {records[idx]['t']:.1f} s")
    final_path = output_dir / f"{spec.robot}_mujoco_final.png"
    montage_path = output_dir / f"{spec.robot}_mujoco_snapshots.png"
    images[-1].save(final_path, quality=95)
    save_montage(images, labels, montage_path)
    return final_path, montage_path


def plot_paper_figure(
    spec: RobotSpec,
    records: list[dict[str, float]],
    target_paths: dict[str, np.ndarray],
    final_render_path: Path | None,
    output_path: Path,
) -> None:
    t = np.array([r["t"] for r in records])
    primary = np.column_stack(
        (
            [r["primary_x"] for r in records],
            [r["primary_y"] for r in records],
            [r["primary_z"] for r in records],
        )
    )
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
    cmap = plt.get_cmap("tab10")
    for idx, (label, values) in enumerate(target_paths.items()):
        if len(target_paths) == 1:
            scatter = ax_xy.scatter(
                values[:, 0], values[:, 1], c=t - t[0], s=7, cmap="viridis", label=label
            )
        else:
            ax_xy.plot(values[:, 0], values[:, 1], linewidth=1.0, color=cmap(idx), label=label)
            scatter = ax_xy.scatter(
                values[:: max(1, len(values) // 80), 0],
                values[:: max(1, len(values) // 80), 1],
                c=t[:: max(1, len(values) // 80)] - t[0],
                s=5,
                cmap="viridis",
                alpha=0.75,
            )
    if len(target_paths) > 1:
        ax_xy.plot(primary[:, 0], primary[:, 1], color="#111827", linewidth=1.6, label=spec.primary_label)
    ax_xy.scatter(primary[0, 0], primary[0, 1], marker="o", s=42, color="#16a34a", edgecolor="white", linewidth=0.6)
    ax_xy.scatter(primary[-1, 0], primary[-1, 1], marker="*", s=90, color="#dc2626", edgecolor="white", linewidth=0.6)
    ax_xy.set_aspect("equal", adjustable="box")
    ax_xy.set_xlabel("x [m]")
    ax_xy.set_ylabel("y [m]")
    ax_xy.set_title("Top-down trajectory", fontsize=10)
    ax_xy.grid(True, linewidth=0.35, alpha=0.45)
    ax_xy.legend(loc="best", fontsize=6.8, frameon=False)
    cbar = fig.colorbar(scatter, ax=ax_xy, fraction=0.046, pad=0.04)
    cbar.set_label("time [s]", fontsize=8)
    cbar.ax.tick_params(labelsize=7)

    ax_z = fig.add_subplot(gs[0, 2])
    ax_z.plot(t, primary[:, 2], color="#2563eb", linewidth=1.2)
    ax_z.set_ylabel("z [m]")
    ax_z.set_title(f"{spec.primary_label} height", fontsize=10)
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

    fig.suptitle(f"{spec.label} receding-horizon MPC trajectory", fontsize=12)
    fig.savefig(output_path, bbox_inches="tight")
    plt.close(fig)


def write_readme(
    path: Path,
    spec: RobotSpec,
    args: argparse.Namespace,
    records: list[dict[str, float]],
    render_error: str | None,
) -> None:
    duration = records[-1]["t"] - records[0]["t"]
    final_err = records[-1]["tracking_error"]
    p95_solve = float(np.percentile([r["solve_time_ms"] for r in records], 95))
    target_desc = ", ".join(f"{t.kind}:{t.name} ({t.label})" for t in spec.targets)
    text = f"""# {spec.label} trajectory visualization

Source trajectory: `{args.cycles_csv}`

State column: `{args.state_column}`

MuJoCo model: `{args.mjcf}`

Tracked target(s): {target_desc}

Rows: {len(records)}

Duration: {duration:.3f} s

Final tracking error: {final_err:.6g}

Solve-time p95: {p95_solve:.3f} ms

Generated files:

- `{spec.robot}_trajectory.csv`: target positions and MPC metrics.
- `{spec.robot}_mujoco_final.png`: MuJoCo final-state render.
- `{spec.robot}_mujoco_snapshots.png`: four MuJoCo snapshots along the trajectory.
- `{spec.robot}_trajectory_figure.png`: paper/response-letter figure.

Suggested caption:

> {spec.label} closed-loop receding-horizon MPC trajectory in MuJoCo. The plot shows the rendered robot state, the tracked Cartesian target path, and solver/tracking metrics over a 20 s rollout.
"""
    if spec.robot == "leap_left":
        text += "\nFor LEAP hand, the Cartesian path is computed from the four named fingertip geoms and their centroid.\n"
    if render_error:
        text += f"\nMuJoCo rendering fallback/error: `{render_error}`\n"
    path.write_text(text)


def prepare_mjcf_for_rendering(spec: RobotSpec, mjcf: Path, output_dir: Path) -> Path:
    if spec.robot not in {"ur", "leap_left"} or mjcf != spec.mjcf:
        return mjcf
    scene_path = output_dir / f"{spec.robot}_render_scene.xml"
    text = spec.mjcf.read_text()
    if spec.robot == "ur":
        text = text.replace('<mujoco model="ur10">', '<mujoco model="ur10_render_scene">')
        text = text.replace(
            'meshdir="../assets"',
            f'meshdir="{(spec.mjcf.parent.parent / "assets").as_posix()}"',
        )
        statistic = '<statistic center="0.15 0.0 1.25" extent="1.2"/>'
        light = '<light pos="0 0 3.0" dir="0 0 -1" directional="true"/>'
    else:
        text = text.replace('<mujoco model="leap_rh">', '<mujoco model="leap_left_render_scene">')
        text = text.replace(
            'meshdir="../assets/"',
            f'meshdir="{(spec.mjcf.parent.parent / "assets").as_posix()}/"',
        )
        statistic = '<statistic center="-0.06 0.08 0.08" extent="0.28"/>'
        light = '<light pos="0 0 0.7" dir="0 0 -1" directional="true"/>'
    visual_block = f"""  {statistic}

  <visual>
    <headlight diffuse="0.8 0.8 0.8" ambient="0.35 0.35 0.35" specular="0.1 0.1 0.1"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="125" elevation="-22"/>
  </visual>

"""
    ground_assets = """  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge"
             rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8"
             width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
"""
    text = text.replace("  <asset>\n", visual_block + ground_assets, 1)
    text = text.replace(
        "  <worldbody>\n",
        f"""  <worldbody>
    {light}
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
""",
        1,
    )
    scene_path.write_text(text)
    return scene_path

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", choices=sorted(ROBOT_SPECS), required=True)
    parser.add_argument("--cycles-csv", type=Path, default=None)
    parser.add_argument("--mjcf", type=Path, default=None)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--state-column", choices=["q", "q_cmd", "q_ref"], default="q")
    parser.add_argument("--max-rows", type=int, default=None)
    parser.add_argument("--render-width", type=int, default=None)
    parser.add_argument("--render-height", type=int, default=None)
    parser.add_argument("--skip-render", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    spec = ROBOT_SPECS[args.robot]
    args.cycles_csv = args.cycles_csv or spec.default_cycles_csv
    args.mjcf = args.mjcf or spec.mjcf
    if args.output_dir is None:
        stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        args.output_dir = DEFAULT_OUTPUT_ROOT / f"{spec.robot}_trajectory_{stamp}"
    args.output_dir.mkdir(parents=True, exist_ok=True)
    args.mjcf = prepare_mjcf_for_rendering(spec, args.mjcf, args.output_dir)

    rows = load_rows(args.cycles_csv, args.state_column, len(spec.joint_names), args.max_rows)
    model = mujoco.MjModel.from_xml_path(str(args.mjcf))
    q_matrix, records, target_paths, qpos0, addresses = collect_trajectory(model, spec, rows)

    trajectory_csv = args.output_dir / f"{spec.robot}_trajectory.csv"
    write_trajectory_csv(trajectory_csv, records, target_paths)

    final_render_path: Path | None = None
    montage_path: Path | None = None
    render_error: str | None = None
    width = args.render_width or spec.render_width
    height = args.render_height or spec.render_height
    if not args.skip_render:
        try:
            final_render_path, montage_path = render_snapshots(
                model,
                spec,
                q_matrix,
                qpos0,
                addresses,
                records,
                target_paths,
                args.output_dir,
                width,
                height,
            )
        except Exception as exc:
            render_error = f"{type(exc).__name__}: {exc}"

    figure_path = args.output_dir / f"{spec.robot}_trajectory_figure.png"
    plot_paper_figure(spec, records, target_paths, final_render_path, figure_path)
    write_readme(args.output_dir / "README.md", spec, args, records, render_error)

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
