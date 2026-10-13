from __future__ import annotations

from dataclasses import dataclass

import matplotlib

matplotlib.use("Qt5Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap
from numpy.typing import NDArray

palette = [
    "#bfdbfe",
    "#bbf7d0",
    "#fde68a",
    "#ddd6fe",
    "#fed7aa",
    "#a5f3fc",
    "#fbcfe8",
    "#d9e3b0",
]

IntArray = NDArray[np.int64]
BoolArray = NDArray[np.bool_]

cmap = ListedColormap(["#fbcfe8", "#f1f5f9", "#334155"] + palette)


@dataclass
class MapCase:
    name: str
    purpose: str
    rows: list[str]

    def expected(self) -> NDArray[np.int64]:
        """仅将写好的字母答案转换为整数数组，不执行分区算法。"""
        result = np.full((len(self.rows), len(self.rows[0])), -1, dtype=np.int64)
        ids: dict[str, int] = {}
        for y, row in enumerate(self.rows):
            for x, letter in enumerate(row):
                if letter == ".":
                    continue
                if letter not in ids:
                    ids[letter] = len(ids)
                result[y, x] = ids[letter]
        return result


def make_map(
    height: int = 10,
    width: int = 10,
    obstacles: list[tuple[int, int, int, int]] | None = None,
) -> BoolArray:
    """obstacles 中每项为 (y_start, y_stop, x_start, x_stop)，终点不包含。"""
    free = np.ones((height, width), dtype=bool)

    if obstacles is not None:
        for y_start, y_stop, x_start, x_stop in obstacles:
            free[y_start:y_stop, x_start:x_stop] = False

    return free


obs = [(5, 15, 4, 6), (8, 13, 6, 14), (2, 13, 14, 17)]

free = make_map(20, 20, obs)


# False（0）为深色障碍物，True（1）为浅蓝色自由空间。
free_cmap = ListedColormap(["#334155", "#bfdbfe"])
fig, ax = plt.subplots()
ax.imshow(free, cmap=free_cmap, vmin=0, vmax=1, origin="lower", interpolation="nearest")

height, width = free.shape

ax.set_xticks(np.arange(width))
ax.set_yticks(np.arange(height))

ax.set_xticks(np.arange(width + 1) - 0.5, minor=True)
ax.set_yticks(np.arange(height + 1) - 0.5, minor=True)
ax.grid(which="minor", color="white", linewidth=1)
ax.tick_params(which="minor", length=0)

ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_title("bow")
ax.set_aspect("equal")

plt.show()
