# /// script
# requires-python = ">=3.12"
# dependencies = ["matplotlib>=3.9"]
# ///
"""Rebuild the explanatory figures used by the FOC textbook."""

from pathlib import Path
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "docs/firmware/assets/foc"
PREVIEW = ROOT / "tools/build/docs-figures"
FONT = Path("C:/Windows/Fonts/msyh.ttc")
if FONT.exists():
    font_manager.fontManager.addfont(str(FONT))
    plt.rcParams["font.family"] = font_manager.FontProperties(fname=str(FONT)).get_name()
plt.rcParams.update({"font.size": 11, "axes.unicode_minus": False,
                     "svg.fonttype": "path", "svg.hashsalt": "foc-textbook",
                     "savefig.facecolor": "white"})
BLUE, RED, GRAY = "#1768a6", "#c44d42", "#687787"


def save(fig, name):
    OUTPUT.mkdir(parents=True, exist_ok=True)
    PREVIEW.mkdir(parents=True, exist_ok=True)
    svg = OUTPUT / f"{name}.svg"
    fig.savefig(svg, bbox_inches="tight", metadata={"Date": None})
    svg.write_text("\n".join(line.rstrip() for line in svg.read_text(encoding="utf-8").splitlines()) + "\n",
                   encoding="utf-8", newline="\n")
    fig.savefig(PREVIEW / f"{name}.png", dpi=130, bbox_inches="tight")
    plt.close(fig)


def arrow(ax, end, color, label, label_offset=(0, 0), width=2):
    ax.annotate("", xy=end, xytext=(0, 0),
                arrowprops={"arrowstyle": "->", "color": color, "lw": width})
    ax.text(end[0] + label_offset[0], end[1] + label_offset[1], label,
            color=color, ha="center", va="center")


def clarke_axes():
    fig, axes = plt.subplots(1, 2, figsize=(11.2, 5.0), constrained_layout=True)
    for ax in axes:
        ax.set(xlim=(-1.65, 1.65), ylim=(-1.40, 1.60), aspect="equal")
        ax.axis("off")
    left, right = axes
    green, gold = "#55915a", "#b17b22"
    left.set_title("① 三相绕组轴固定在定子上", pad=18)
    arrow(left, (1.12, 0), green, "A 正轴 / α", (.32, -.12))
    arrow(left, (-.56, .97), RED, "B 正轴：120°", (-.38, .15))
    arrow(left, (-.56, -.97), gold, "C 正轴：−120°", (-.36, -.16))
    left.plot([0, 0], [0, 1.22], "--", color=GRAY, lw=1)
    left.text(.10, 1.32, "β：竖直轴", color=GRAY)
    angle = np.linspace(0, 2*math.pi/3, 60)
    left.plot(.40*np.cos(angle), .40*np.sin(angle), color=GRAY, lw=1)
    left.text(.13, .45, "120°", color=GRAY)
    left.text(0, -1.30, "αβ 是同一平面上的水平、竖直坐标", ha="center", color=GRAY)
    right.set_title("② 例：把 B 相 +1 A 投影到 αβ", pad=18)
    arrow(right, (1.18, 0), GRAY, r"$\alpha$", (.12, 0), 1)
    arrow(right, (0, 1.22), GRAY, r"$\beta$", (.10, .08), 1)
    tip = (-.5, math.sqrt(3)/2)
    arrow(right, tip, RED, "B 相 +1 A", (-.38, .13), 3)
    right.plot([tip[0], tip[0], 0], [0, tip[1], tip[1]], ":", color=RED, lw=1.5)
    right.plot([0, tip[0]], [0, 0], color=RED, lw=3)
    right.text(-.57, -.18, r"$1\times\cos120^\circ=-0.5$", color=RED, ha="center")
    right.text(.14, .84, r"$1\times\sin120^\circ$" + "\n" + r"$=+0.866$", color=RED)
    right.text(0, -.78, "先算每相的水平、竖直分量", ha="center")
    right.text(0, -1.05, "再分别相加，最后统一乘 2/3", ha="center", color=BLUE)
    fig.suptitle("Clarke 变换：把三相合成作用写成 αβ 两个坐标", fontsize=15)
    save(fig, "clarke-axes")


def clarke_example():
    fig, axes = plt.subplots(1, 2, figsize=(11.2, 5.5), constrained_layout=True)
    green, gold = "#55915a", "#b17b22"
    for ax in axes:
        ax.set(xlim=(-1.65, .95), ylim=(-.95, 1.75), aspect="equal")
        ax.axis("off")
        arrow(ax, (.75, 0), GRAY, r"$\alpha$", (.13, 0), 1)
        arrow(ax, (0, 1.49), GRAY, r"$\beta$", (.13, .08), 1)
    left, right = axes
    left.set_title("① 电流带符号：负电流朝相正轴的反向", pad=20)
    contributions = np.array([[-.5, 0], [-.5, math.sqrt(3)/2], [.25, math.sqrt(3)/4]])
    for deg in (120, -120):
        left.plot([0, .92*math.cos(math.radians(deg))],
                  [0, .92*math.sin(math.radians(deg))], "--", color=GRAY, lw=.8, alpha=.6)
    left.text(-.65, -.85, "C 正轴", color=GRAY, fontsize=10)
    arrow(left, contributions[0], green, r"$i_a=-0.5$", (-.40, -.18), 2.7)
    arrow(left, contributions[1], RED, r"$i_b=+1$", (-.32, .16), 2.7)
    arrow(left, contributions[2], gold, r"$i_c=-0.5$", (.36, .07), 2.7)
    left.text(-.35, -.60, "C 电流为负，所以贡献朝右上", ha="center", color=gold)
    right.set_title("② 首尾相接求和，再乘 2/3", pad=20)
    start = np.zeros(2)
    for delta, color in zip(contributions, (green, RED, gold)):
        end = start + delta
        right.annotate("", xy=end, xytext=start,
                       arrowprops={"arrowstyle": "->", "color": color, "lw": 2.7})
        start = end
    raw = contributions.sum(axis=0)
    result = raw * 2/3
    assert np.allclose(result, [-.5, math.sqrt(3)/2])
    right.plot([0, raw[0]], [0, raw[1]], "--", color=GRAY, lw=1.4)
    right.scatter([raw[0]], [raw[1]], color=GRAY, s=25)
    right.text(raw[0]-.14, raw[1]+.16, r"$S=(-0.75,\ 1.299)$", ha="center", color=GRAY)
    arrow(right, result, BLUE, r"$i_{\alpha\beta}=\frac{2}{3}S$", (.64, .10), 3.2)
    right.plot([result[0], result[0], 0], [0, result[1], result[1]], ":", color=BLUE, lw=1.2)
    right.text(-.40, -.38, r"$i_\alpha=-0.5\ A,\quad i_\beta=+0.866\ A$",
               ha="center", color=BLUE, fontsize=12)
    right.text(-.40, -.69, "蓝色箭头与原合成方向相同，长度缩为 2/3", ha="center", fontsize=10)
    fig.suptitle("贯穿算例：三相电流 (−0.5, +1, −0.5) A 怎样变成 αβ", fontsize=15)
    save(fig, "clarke-example")


def coordinates():
    fig, axes = plt.subplots(1, 2, figsize=(10.4, 4.3), constrained_layout=True)
    theta = math.pi / 6
    for ax in axes:
        ax.set(xlim=(-1.2, 1.5), ylim=(-.5, 1.5), aspect="equal")
        ax.axis("off")
    left, right = axes
    left.set_title("定子视角：固定的 αβ 坐标", pad=16)
    arrow(left, (1.28, 0), GRAY, r"$\alpha$", (.12, 0), 1)
    arrow(left, (0, 1.28), GRAY, r"$\beta$", (0, .13), 1)
    arrow(left, (1.12*math.cos(theta), 1.12*math.sin(theta)), RED,
          r"$d$：转子磁链方向", (.18, .12))
    left.plot([0, -.65], [0, 1.126], "--", color=GRAY, lw=1)
    left.text(-.76, 1.20, r"$q$", color=GRAY)
    arrow(left, (-.5, math.sqrt(3)/2), BLUE, r"$i_s=1\,A$", (-.12, .12), 3)
    left.plot([-.5, -.5, 0], [0, math.sqrt(3)/2, math.sqrt(3)/2], ":", color=BLUE)
    left.text(-.51, -.18, r"$i_\alpha=-0.5$", color=BLUE, ha="center")
    left.text(.10, .92, r"$i_\beta=0.866$", color=BLUE)
    arc = np.linspace(0, theta, 40)
    left.plot(.43*np.cos(arc), .43*np.sin(arc), color=RED)
    left.text(.54, .12, "30°", color=RED)
    right.set_title("转子视角：一起转动的 dq 坐标", pad=16)
    arrow(right, (1.20, 0), RED, r"$d$", (.15, 0))
    arrow(right, (0, 1.28), GRAY, r"$q$", (0, .14), 1)
    arrow(right, (0, 1), BLUE, r"$i_s=1\,A$", (.43, 0), 3)
    right.text(.20, .58, r"$i_d=0$" + "\n" + r"$i_q=1\,A$", color=BLUE, fontsize=14)
    fig.suptitle("电流没有改变，改变的是观察它的坐标", fontsize=15)
    save(fig, "coordinates")


def waveforms():
    phase = np.linspace(0, 2*np.pi, 721)
    alpha, beta = -np.sin(phase), np.cos(phase)
    ia = alpha
    ib = -.5*alpha + np.sqrt(3)/2*beta
    ic = -.5*alpha - np.sqrt(3)/2*beta
    id_ = alpha*np.cos(phase) + beta*np.sin(phase)
    iq = -alpha*np.sin(phase) + beta*np.cos(phase)
    fig, axes = plt.subplots(2, 1, figsize=(9.2, 5.8), sharex=True, constrained_layout=True)
    for line, label, color in zip((ia, ib, ic), (r"$i_a$", r"$i_b$", r"$i_c$"), (BLUE, RED, "#55915a")):
        axes[0].plot(np.degrees(phase), line, label=label, color=color, lw=2)
    axes[1].plot(np.degrees(phase), id_, label=r"$i_d$", color=RED, lw=2)
    axes[1].plot(np.degrees(phase), iq, label=r"$i_q$", color=BLUE, lw=2)
    axes[0].set_title("三相电流持续变化，但相对于转子的电流分量可以保持不变")
    for ax in axes:
        ax.set(ylabel="电流 / A", ylim=(-1.25, 1.25))
        ax.grid(alpha=.2)
        ax.legend(loc="upper right", ncol=3)
    axes[1].set(xlabel="电角度 / °", xticks=range(0, 361, 60), xlim=(0, 360))
    save(fig, "waveforms")


def saliency():
    phi = np.linspace(0, 2*np.pi, 361)
    voltage = np.array([np.cos(phi), np.sin(phi)])
    fig, axes = plt.subplots(1, 3, figsize=(11.6, 4.0), constrained_layout=True)
    for ax, deg in zip(axes, (0, 30, 90)):
        theta = math.radians(deg)
        rotation = np.array([[math.cos(theta), -math.sin(theta)],
                             [math.sin(theta), math.cos(theta)]])
        inverse_l = rotation @ np.diag([1/.0008, 1/.0012]) @ rotation.T
        response = inverse_l @ voltage * .00002 * 1000
        ax.plot(response[0], response[1], color=BLUE, lw=2)
        for column, label, color in ((0, "+α 激励", RED), (1, "+β 激励", "#55915a")):
            point = inverse_l[:, column] * .00002 * 1000
            arrow(ax, point, color, label, (3 if column == 0 else -4, 3), 1.7)
        ax.axhline(0, color=GRAY, lw=.6)
        ax.axvline(0, color=GRAY, lw=.6)
        ax.set(xlim=(-31, 36), ylim=(-31, 35), aspect="equal", xlabel=r"$\Delta i_\alpha$ / mA",
               ylabel=r"$\Delta i_\beta$ / mA", title=f"转子电角度 {deg}°")
        ax.grid(alpha=.15)
    fig.suptitle("相同的高频电压，电流响应随转子方向改变（理想算例）", fontsize=14)
    save(fig, "saliency")


if __name__ == "__main__":
    clarke_axes()
    clarke_example()
    coordinates()
    waveforms()
    saliency()
    print(f"Wrote SVG figures to {OUTPUT}")
