"""Matplotlib styles shared with the original Park visualizer."""

COLOR_MAP = {
    "red": "#FF0000",
    "green": "#00FF00",
    "blue": "#0000FF",
    "yellow": "#FFFF00",
    "cyan": "#00FFFF",
    "magenta": "#FF00FF",
    "orange": "#FFA500",
    "gray": "#808080",
    "black": "#000000",
}

RENDER = {
    "obs_polygon": {
        "facecolor": COLOR_MAP["gray"],
        "edgecolor": "black",
        "linewidth": 1.5,
        "alpha": 0.5,
    },
    "init_state": {
        "facecolor": "none",
        "edgecolor": COLOR_MAP["green"],
        "linewidth": 2,
    },
    "goal_state": {"facecolor": "none", "edgecolor": COLOR_MAP["red"], "linewidth": 2},
    "occ_scatter": {"color": COLOR_MAP["black"], "marker": "s", "s": 4},
    "init_path": {
        "color": COLOR_MAP["magenta"],
        "linewidth": 2,
        "linestyle": "-",
        "label": "opt path",
    },
    "init_path_scatter": {
        "color": COLOR_MAP["magenta"],
        "s": 20,
        "marker": "o",
        "zorder": 5,
    },
    "init_path_rect": {
        "facecolor": "none",
        "edgecolor": COLOR_MAP["magenta"],
        "linewidth": 1.5,
    },
    "pre_opt_path": {
        "color": COLOR_MAP["cyan"],
        "linewidth": 2,
        "linestyle": "--",
        "label": "pre-opt path",
    },
    "pre_opt_path_scatter": {
        "color": COLOR_MAP["cyan"],
        "s": 14,
        "marker": "o",
        "zorder": 4,
    },
    "pre_opt_path_rect": {
        "facecolor": "none",
        "edgecolor": COLOR_MAP["cyan"],
        "linewidth": 1,
        "linestyle": "--",
    },
    # speed
    "init_speed": {
        "color": COLOR_MAP["red"],
        "linewidth": 2,
        "label": "speed opt (m/s)",
    },
    "pre_opt_speed": {
        "color": COLOR_MAP["orange"],
        "linewidth": 2,
        "linestyle": "--",
        "label": "speed pre-opt (m/s)",
    },
    # acceleration
    "init_acc": {
        "color": COLOR_MAP["green"],
        "linewidth": 2,
        "label": "acc opt (m/s²)",
    },
    "pre_opt_acc": {
        "color": COLOR_MAP["cyan"],
        "linewidth": 2,
        "linestyle": "--",
        "label": "acc pre-opt (m/s²)",
    },
    # steering
    "init_steer": {
        "color": COLOR_MAP["blue"],
        "linewidth": 2,
        "label": "steer opt (rad)",
    },
    "pre_opt_steer": {
        "color": COLOR_MAP["magenta"],
        "linewidth": 2,
        "linestyle": "--",
        "label": "steer pre-opt (rad)",
    },
}

FIGURE_SIZE_MAIN = (14, 8)
FIGURE_SIZE_SMALL = (8, 4)
FIGURE_SIZE_ESDF = (12, 8)
SAVE_DPI = 150
