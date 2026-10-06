#!/usr/bin/env python3
"""Plot the blockage belief model: hold for t1, linear decay over t2, and the two check outcomes.

    python3 plot_belief_model.py              # writes belief_model.png and belief_model.pdf here
    python3 plot_belief_model.py --show
    python3 plot_belief_model.py --transparent   # transparent background instead of white

Belief: +1 = blocked, 0 = unknown, -1 = free. Timeline (arbitrary units):
  - free until the object is first observed (t_obs), then held at 1 for t1 and decayed to 0 over t2
  - at a later check (t_check), either the object is re-observed (belief back to 1, held, decaying,
    until it is later observed gone -> -1) or it is observed gone right away (-> -1)
"""

import argparse
import os

import matplotlib.pyplot as plt

# Timeline
T_START, T_OBS, T1, T2 = 0.0, 1.0, 2.0, 3.0
T_CHECK = 8.0  # opportunistic check
T_GONE_LATER = 11.4  # re-observed object later observed gone
T_END = 13.0

# Colours: categorical slots 1-3 of the reference palette (fixed order), text in ink tokens
SURFACE = "#ffffff"
INK = "#0b0b0b"
INK_2 = "#52514e"
C_DECAY, C_SEEN, C_GONE = "#2a78d6", "#eb6834", "#1baf7a"


def belief(age, t1=T1, t2=T2):
    """Belief of an object last observed `age` ago (hold t1, then linear decay to 0 over t2)."""
    if age <= t1:
        return 1.0
    return max(0.0, 1.0 - (age - t1) / t2)


def first_episode():
    xs = [T_START, T_OBS, T_OBS, T_OBS + T1, T_OBS + T1 + T2, T_CHECK]
    ys = [-1.0, -1.0, 1.0, 1.0, 0.0, 0.0]
    return xs, ys


def reobserved_branch():
    b_gone = belief(T_GONE_LATER - T_CHECK)
    xs = [T_CHECK, T_CHECK, T_CHECK + T1, T_GONE_LATER, T_GONE_LATER, T_END]
    ys = [0.0, 1.0, 1.0, b_gone, -1.0, -1.0]
    return xs, ys


def gone_branch():
    return [T_CHECK, T_CHECK, T_END], [0.0, -1.0, -1.0]


def span_arrow(ax, x0, x1, y, label):
    ax.annotate("", xy=(x0, y), xytext=(x1, y),
                arrowprops=dict(arrowstyle="<->", color=INK_2, lw=1.0, shrinkA=0, shrinkB=0))
    ax.text((x0 + x1) / 2, y - 0.07, label, ha="center", va="top", color=INK, fontsize=16)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--show", action="store_true", help="open a window instead of only saving")
    parser.add_argument("--transparent", action="store_true", help="transparent background instead of white")
    parser.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "belief_model"))
    args = parser.parse_args()

    fig, ax = plt.subplots(figsize=(9, 2.8), facecolor=SURFACE)
    ax.set_facecolor(SURFACE)

    # Gone-right-away branch first (dashed, below), re-observed branch drawn on top
    ax.plot(*gone_branch(), color=C_GONE, lw=2, ls=(0, (5, 3)), label="Check: observed gone", zorder=3)
    ax.plot(*first_episode(), color=C_DECAY, lw=2, label="First observation, then decay", zorder=4,
            solid_joinstyle="miter")
    ax.plot(*reobserved_branch(), color=C_SEEN, lw=2, label="Check: re-observed (later observed gone)",
            zorder=5, solid_joinstyle="miter")

    # Hold-end guides
    for x in (T_OBS + T1, T_CHECK + T1):
        ax.plot([x, x], [0, 1], color=INK_2, lw=0.8, ls=":", zorder=2)

    # Durations
    span_arrow(ax, T_OBS, T_OBS + T1, -0.14, r"$t_1$")
    span_arrow(ax, T_OBS + T1, T_OBS + T1 + T2, -0.14, r"$t_2$")
    span_arrow(ax, T_CHECK, T_CHECK + T1, -0.14, r"$t_1$")

    # Event labels (ink, not series colour)
    ax.text(T_OBS + 0.08, 1.06, "object observed", color=INK_2, fontsize=13, va="bottom")
    ax.text(T_CHECK - 0.1, 0.06, "check", color=INK_2, fontsize=13, ha="right", va="bottom")
    ax.text(T_CHECK + 0.08, 1.06, "re-observed", color=INK_2, fontsize=13, va="bottom")
    ax.text(T_CHECK + 0.15, -0.94, "observed gone", color=INK_2, fontsize=13, va="bottom")
    ax.text(T_GONE_LATER + 0.1, -0.5, "observed\ngone", color=INK_2, fontsize=13, va="center")

    # Axes: belief scale with meaning, time without units
    ax.set_xlim(T_START, T_END + 0.6)
    ax.set_ylim(-1.3, 1.3)
    ax.set_yticks([-1, 0, 1])
    ax.set_yticklabels(["−1  free", "0  unknown", "1  blocked"], color=INK_2)
    ax.set_xticks([])
    ax.set_ylabel("belief", color=INK_2, fontsize=14)
    # Time axis runs along belief = 0, with an arrowhead and label at its end
    ax.spines["bottom"].set_position(("data", 0))
    ax.plot(1, 0, ">", color=INK_2, markersize=6, transform=ax.get_yaxis_transform(), clip_on=False)
    ax.text(T_END + 0.6, 0.06, "time", color=INK_2, fontsize=14, ha="right", va="bottom")
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(INK_2)
    ax.tick_params(colors=INK_2, length=3, labelsize=13)

    fig.tight_layout()
    for ext in ("png", "pdf"):
        fig.savefig("%s.%s" % (args.out, ext), dpi=300, bbox_inches="tight",
                    transparent=args.transparent, facecolor="none" if args.transparent else SURFACE)
    print("wrote %s.png and %s.pdf" % (args.out, args.out))
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
