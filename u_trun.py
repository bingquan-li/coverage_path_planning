"""U 形路径候选评分：神经网络提出建议，几何/任务规则决定是否接受。

运行：
    python -m pip install numpy matplotlib scikit-learn
    python u_shape_nn_demo.py
    python u_shape_nn_demo.py --show

输出：u_shape_nn_result.png 和 u_shape_nn_metrics.json。

教学边界：
1. 输入是一段已经选好的候选路径，首尾就是候选入口/出口；本例不学习端点定位。
2. 标签来自合成模板：深 U 为 1，直线/浅弯/L/S/V 为 0。这是人为定义的策略，
   不是“客观上所有应该跳过的路径”，也不是已证明优于几何规则的方法。
3. 网络只看形状，不看障碍物和任务，因此输出不能解释为安全通行概率。
4. 几何检查仅针对圆形机器人、静态圆障碍物和已知自由平面。
   假设机器人允许停车原地转向；未处理速度、加速度和非完整约束。
5. 合成测试集与训练集来自同一生成器；测试分数不代表真实环境性能。
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
import numpy as np
from numpy.typing import NDArray
from sklearn.metrics import (
    accuracy_score,
    confusion_matrix,
    precision_score,
    recall_score,
)
from sklearn.neural_network import MLPClassifier
from threadpoolctl import threadpool_limits

FloatArray = NDArray[np.float64]
POINT_COUNT = 32
SCORE_THRESHOLD = 0.85  # 教学默认值；真实项目应在验证集上按误判成本选取。


def resample_path(points: FloatArray, count: int = POINT_COUNT) -> FloatArray:
    """按照弧长均匀采样，使输入维度不依赖原始路径点数量。"""
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 2:
        raise ValueError("路径必须是至少两个点的 N×2 数组")
    if not np.isfinite(points).all():
        raise ValueError("路径包含非有限数值")

    # 删除连续重复点，保证插值用的累计弧长严格递增。
    distances = np.linalg.norm(np.diff(points, axis=0), axis=1)
    points = points[np.r_[True, distances > 1e-10]]
    if len(points) < 2:
        raise ValueError("路径长度为零")
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    arc = np.r_[0.0, np.cumsum(lengths)]
    sample_arc = np.linspace(0.0, arc[-1], count)
    x = np.interp(sample_arc, arc, points[:, 0])
    y = np.interp(sample_arc, arc, points[:, 1])
    return np.column_stack((x, y))


def encode_path(points: FloatArray) -> FloatArray:
    """平移、旋转、等比例缩放，再展开为 64 个数。

    不分别缩放 x/y，否则深 U 和浅 U 会被压成相同的形状。
    本例刻意去掉绝对尺寸；真实项目可额外输入米制长度/机器人尺寸等特征。
    """
    sampled = resample_path(points)
    chord = sampled[-1] - sampled[0]
    chord_length = float(np.linalg.norm(chord))
    if chord_length < 1e-8:
        raise ValueError("本例不处理首尾重合的闭环候选")

    forward = chord / chord_length
    lateral = np.array([-forward[1], forward[0]])
    relative = sampled - sampled[0]
    local = np.column_stack((relative @ forward, relative @ lateral))
    # 用整体尺寸统一缩放两个坐标，保留宽深比例。
    scale = float(np.max(np.linalg.norm(local, axis=1)))
    local = local / max(scale, 1e-8)
    # 镜像归一化：本例策略不区分朝左/朝右凹陷。
    if float(np.mean(local[:, 1])) > 0.0:
        local[:, 1] *= -1.0
    return local.reshape(-1)


def u_path(width: float, depth: float) -> FloatArray:
    """构造带圆底的 U：下降直线、半椭圆底部、上升直线。"""
    rounding = min(width * 0.45, depth * 0.4)
    left = np.column_stack((np.zeros(20), np.linspace(0, -depth + rounding, 20)))
    theta = np.linspace(np.pi, 2 * np.pi, 35)
    bottom = np.column_stack(
        (
            width / 2 + width / 2 * np.cos(theta),
            -depth + rounding + rounding * np.sin(theta),
        )
    )
    right = np.column_stack((np.full(20, width), np.linspace(-depth + rounding, 0, 20)))
    return np.vstack((left, bottom[1:], right[1:]))


def synthetic_path(rng: np.random.Generator, label: int) -> FloatArray:
    """一个样本独立生成；训练/测试使用不同随机种子。"""
    width = rng.uniform(0.6, 2.0)
    if label == 1:
        points = u_path(width, width * rng.uniform(1.25, 3.5))
    else:
        kind = int(rng.integers(0, 5))
        t = np.linspace(0.0, 1.0, 70)
        if kind == 0:  # 近似直线。
            points = np.column_stack((width * t, 0.03 * width * np.sin(2 * np.pi * t)))
        elif kind == 1:  # 浅 U：策略要求保留。
            points = u_path(width, width * rng.uniform(0.1, 0.6))
        elif kind == 2:  # 普通 L 弯。
            points = resample_path(np.array([[0, 0], [0, -width], [width, -width]]), 70)
        elif kind == 3:  # S 形：本例只学习 U，因此标 0。
            points = np.column_stack(
                (width * t, width * rng.uniform(0.3, 1.1) * np.sin(2 * np.pi * t))
            )
        else:  # 深 V 是一个较难负例，不能仅凭绕行比判断标签。
            depth = width * rng.uniform(1.25, 3.5)
            points = resample_path(
                np.array([[0, 0], [width / 2, -depth], [width, 0]]), 70
            )

    # 平滑横向形变和轻微噪声，避免所有样本都是完美模板。
    t = np.linspace(0, 1, len(points))
    points[:, 0] += rng.uniform(-0.12, 0.12) * width * np.sin(np.pi * t)
    points += rng.normal(0, 0.005 * width, points.shape)
    if rng.random() < 0.5:
        points = points[::-1].copy()
    if rng.random() < 0.5:
        points[:, 1] *= -1
    angle = rng.uniform(-np.pi, np.pi)
    rotation = np.array(
        [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]
    )
    return points @ rotation.T + rng.uniform(-10, 10, 2)


def make_dataset(count: int, seed: int) -> tuple[FloatArray, NDArray[np.int64]]:
    rng = np.random.default_rng(seed)
    labels = np.arange(count, dtype=np.int64) % 2
    rng.shuffle(labels)
    features = np.stack(
        [encode_path(synthetic_path(rng, int(label))) for label in labels]
    )
    return features, labels


def segment_circle_clearance(
    a: FloatArray,
    b: FloatArray,
    obstacle: tuple[float, float, float],
    robot_radius: float,
) -> float:
    """线段扫过的圆形机器人到圆障碍物的最小间隙，解析计算，不靠离散采样。

    大于 0 表示有间隙，等于/小于 0 表示相切/碰撞。
    """
    cx, cy, obstacle_radius = obstacle
    center = np.array([cx, cy])
    ab = b - a
    t = float(np.dot(center - a, ab) / max(float(np.dot(ab, ab)), 1e-12))
    nearest = a + np.clip(t, 0, 1) * ab
    return float(np.linalg.norm(center - nearest)) - obstacle_radius - robot_radius


def decide(
    model: MLPClassifier,
    path: FloatArray,
    obstacles: list[tuple[float, float, float]],
    must_visit: bool = False,
    robot_radius: float = 0.2,
) -> dict[str, object]:
    """形状分数、收益、任务约束、碰撞检查共同决定是否替换。"""
    class_index = int(np.flatnonzero(model.classes_ == 1)[0])
    score = float(model.predict_proba(encode_path(path)[None, :])[0, class_index])
    length = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
    chord_length = float(np.linalg.norm(path[-1] - path[0]))
    gain = length - chord_length
    clearance = min(
        (
            segment_circle_clearance(path[0], path[-1], o, robot_radius)
            for o in obstacles
        ),
        default=float("inf"),
    )
    if score < SCORE_THRESHOLD:
        accepted, reason = False, "KEEP: low U-shape score"
    elif must_visit:
        accepted, reason = False, "KEEP: required visit"
    elif gain < 0.5:
        accepted, reason = False, "KEEP: small length saving"
    elif clearance <= 0.05:  # 额外 5 cm 间隙。
        accepted, reason = False, "KEEP: shortcut blocked"
    else:
        accepted, reason = True, "ACCEPT: use shortcut"
    return {
        "score": score,
        "accepted": accepted,
        "reason": reason,
        "length_saved_m": gain,
        "clearance_m": clearance if np.isfinite(clearance) else None,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--show", action="store_true", help="显示 matplotlib 窗口")
    parser.add_argument(
        "--output-dir", type=Path, default=Path(__file__).resolve().parent
    )
    args = parser.parse_args()
    if not args.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.patches import Circle

    # 3000 个训练样本；early_stopping 从其中留出 15% 作为验证集。
    # 1000 个独立测试样本只用于训练完成后的评估。
    x_train, y_train = make_dataset(3000, seed=10)
    x_test, y_test = make_dataset(1000, seed=20)
    model = MLPClassifier(
        hidden_layer_sizes=(64, 32),
        activation="relu",
        solver="adam",
        learning_rate_init=0.001,
        batch_size=128,
        max_iter=200,
        early_stopping=True,
        validation_fraction=0.15,
        n_iter_no_change=15,
        random_state=42,
    )
    with threadpool_limits(limits=2):
        model.fit(x_train, y_train)
        scores = model.predict_proba(x_test)[:, 1]
    predicted = (scores >= SCORE_THRESHOLD).astype(np.int64)
    metrics: dict[str, object] = {
        "note": "Synthetic, same-generator test only; not real-world safety evidence.",
        "threshold": SCORE_THRESHOLD,
        "train_count_including_validation": 3000,
        "test_count": 1000,
        "epochs": int(model.n_iter_),
        "accuracy": float(accuracy_score(y_test, predicted)),
        "precision": float(precision_score(y_test, predicted, zero_division=0)),
        "recall": float(recall_score(y_test, predicted, zero_division=0)),
        "confusion_matrix_rows_true_columns_predicted": confusion_matrix(
            y_test, predicted
        ).tolist(),
    }

    deep_u = u_path(1.2, 2.8)
    cases = [
        ("A  |  Clear opening", deep_u, [], False),
        ("B  |  Same path, obstacle added", deep_u, [(0.6, 0.28, 0.16)], False),
        ("C  |  Same path, required visit", deep_u, [], True),
        ("D  |  Shallow bend", u_path(1.2, 0.35), [], False),
    ]
    fig, axes = plt.subplots(1, 4, figsize=(13.8, 5.2), sharex=True, sharey=True)
    fig.patch.set_facecolor("#f5f7fb")
    results = []
    for ax, (title, path, obstacles, must_visit) in zip(axes, cases):
        result = decide(model, path, obstacles, must_visit)
        results.append({"case": title, **result})
        color = "#15835c" if result["accepted"] else "#c04b43"
        ax.plot(path[:, 0], path[:, 1], color="#3478c0", linewidth=3)
        ax.plot(path[[0, -1], 0], path[[0, -1], 1], "--", color=color, linewidth=2.5)
        ax.scatter(path[[0, -1], 0], path[[0, -1], 1], s=35, color="#23334d", zorder=5)
        ax.text(-0.1, 0.13, "A", ha="right", fontsize=11)
        ax.text(1.3, 0.13, "B", fontsize=11)
        for cx, cy, radius in obstacles:
            ax.add_patch(
                Circle((cx, cy), radius + 0.2 + 0.05, color="#edb45d", alpha=0.23)
            )
            ax.add_patch(Circle((cx, cy), radius, color="#4d5663"))
        if must_visit:
            ax.scatter([0.6], [-2.8], marker="*", s=170, color="#d28c12", zorder=6)
        ax.set_title(title, fontsize=11, pad=13, loc="left")
        ax.text(
            0.04,
            0.045,
            f"U score: {result['score']:.3f}\n{result['reason']}",
            transform=ax.transAxes,
            fontsize=10,
            color=color,
            bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.95, "pad": 5},
        )
        ax.set_aspect("equal")
        ax.set_xlim(-0.55, 1.75)
        ax.set_ylim(-3.65, 0.95)
        ax.grid(alpha=0.12)
        ax.set_xlabel("x (m)")
        ax.spines[["top", "right"]].set_visible(False)
    axes[0].set_ylabel("y (m)")
    fig.suptitle(
        "A learned shape score proposes; geometry and task rules decide",
        fontsize=16,
        y=0.985,
    )
    legend = [
        Line2D([0], [0], color="#3478c0", lw=3, label="Original candidate path"),
        Line2D([0], [0], color="#15835c", lw=2, ls="--", label="Accepted chord"),
        Line2D([0], [0], color="#c04b43", lw=2, ls="--", label="Rejected chord"),
        Circle(
            (0, 0),
            1,
            color="#edb45d",
            alpha=0.3,
            label="Obstacle + robot radius + margin",
        ),
    ]
    fig.legend(
        handles=legend,
        loc="lower center",
        ncol=4,
        frameon=False,
        fontsize=10,
        bbox_to_anchor=(0.5, 0.04),
    )
    fig.text(
        0.5,
        0.015,
        "Synthetic training demo | Circular robot, radius 0.20 m | Candidate endpoints are supplied",
        ha="center",
        fontsize=9,
        color="#526277",
    )
    fig.tight_layout(rect=(0, 0.11, 1, 0.95))
    args.output_dir.mkdir(parents=True, exist_ok=True)
    image_path = args.output_dir / "u_shape_nn_result.png"
    fig.savefig(image_path, dpi=160, facecolor=fig.get_facecolor())
    metrics["cases"] = results
    metrics_path = args.output_dir / "u_shape_nn_metrics.json"
    metrics_path.write_text(
        json.dumps(metrics, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(metrics, ensure_ascii=False, indent=2))
    print(f"Saved: {image_path}")
    if args.show:
        plt.show()
    plt.close(fig)


if __name__ == "__main__":
    main()
