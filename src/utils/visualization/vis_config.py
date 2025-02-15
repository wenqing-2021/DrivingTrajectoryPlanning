from bokeh.palettes import Turbo256

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

FIG_VIS = {
    "title": "AutomatedPark",
    "main_figure": {
        "width": 1280,
        "height": 720,
        "match_aspect": True,
    },
    "esdf_figure": {
        "width": 1280,
        "height": 720,
        "match_aspect": True,
    },
    "row_margin_width": 50,
}

RENDER_VIS = {
    "obs_polygon": {
        "color": COLOR_MAP["gray"],
        "line_width": 2,
    },
    "init_state": {
        "color": COLOR_MAP["green"],
        "line_width": 2,
        "fill_alpha": 0.0,
    },
    "goal_state": {
        "color": COLOR_MAP["red"],
        "line_width": 2,
        "fill_alpha": 0.0,
    },
    "occ_square": {
        "color": COLOR_MAP["black"],
        "marker": "square",
        "size": 20,
    },
    "esdf_contour": {
        "contour": {
            "fill_color": Turbo256,
            "line_color": "black",
        },
        "max_levels": 25,
    },
    "init_path": {
        "color": COLOR_MAP["magenta"],
        "line_width": 2,
    },
    "init_path_rect":{
        "fill_alpha": 0.0,
        "line_width": 2,
        "color": COLOR_MAP["magenta"],
    },
    "init_path_scatter":{
        "color": COLOR_MAP["magenta"],
        "size": 9,
    }
}
