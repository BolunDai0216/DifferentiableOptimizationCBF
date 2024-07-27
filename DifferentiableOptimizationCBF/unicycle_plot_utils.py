import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle, Wedge
from matplotlib.ticker import MultipleLocator
from matplotlib.transforms import Affine2D


def plot_unicycle(history):
    data = np.concatenate(history).reshape(-1, 3)
    fig, ax = plt.subplots(1, 1, figsize=(8, 8))

    polygon1 = Rectangle(
        (-1, -1),
        2,
        2,
        facecolor="cornflowerblue",
        edgecolor="black",
        linewidth=2,
        zorder=20,
    )
    polygon2 = Rectangle(
        (3, -1),
        2,
        2,
        facecolor="cornflowerblue",
        edgecolor="black",
        linewidth=2,
        zorder=20,
    )

    ax.plot(
        data[:, 0], data[:, 1], color="thistle", linewidth=4, zorder=40, label="CBFQP"
    )
    ax.add_patch(polygon1)
    ax.add_patch(polygon2)

    capsule_boxes = []
    capsule_circles1 = []
    capsule_circles2 = []

    frames = [0, 100, 200, 300, 500]
    alphas = [1.0, 0.9, 0.8, 0.7, 0.6]

    for i in range(5):
        capsule_boxes.append(
            Rectangle(
                (-0.25, -0.3), 0.5, 0.6, facecolor="purple", zorder=20, alpha=alphas[i]
            )
        )
        capsule_circles1.append(
            Wedge(
                (0.25, 0.0),
                0.3,
                -90,
                90,
                facecolor="purple",
                zorder=20,
                alpha=alphas[i],
            )
        )
        capsule_circles2.append(
            Wedge(
                (-0.25, 0.0),
                0.3,
                90,
                270,
                facecolor="purple",
                zorder=20,
                alpha=alphas[i],
            )
        )

        ax.add_patch(capsule_boxes[-1])
        ax.add_patch(capsule_circles1[-1])
        ax.add_patch(capsule_circles2[-1])

        t_capsule = (
            Affine2D()
            .rotate(data[frames[i], 2])
            .translate(data[frames[i], 0], data[frames[i], 1])
        )
        capsule_boxes[-1].set_transform(t_capsule + ax.transData)
        capsule_circles1[-1].set_transform(t_capsule + ax.transData)
        capsule_circles2[-1].set_transform(t_capsule + ax.transData)

    ax.xaxis.set_major_locator(MultipleLocator(1.0))
    ax.xaxis.set_minor_locator(MultipleLocator(0.25))
    ax.yaxis.set_major_locator(MultipleLocator(1.0))
    ax.yaxis.set_minor_locator(MultipleLocator(0.25))

    ax.grid(True, "minor", color="0.85", linewidth=0.50, zorder=-20)
    ax.grid(True, "major", color="0.65", linewidth=0.75, zorder=-10)
    ax.tick_params(which="both", bottom=False, left=False)

    ax.set_aspect("equal")
    ax.legend(frameon=False, ncol=1, fontsize=22)
    ax.set_xlim([-2, 5.5])
    ax.set_ylim([-3.8, 3.2])
    plt.show()
