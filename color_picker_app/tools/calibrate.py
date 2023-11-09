from typing import List, Dict, Any, Tuple

from matplotlib import pyplot as plt
from tools.start_run import start_run_with_log_scraping
from pathlib import Path
import numpy as np
from tools.color_utils import convert_volumes_to_payload
from tools.plate_color_analysis import get_colors_from_file
import cv2
import base64


def get_image(loop_protocol, plate_volumes, experiment, curr_wells_used, steps_run):
    print(plate_volumes)
    print(curr_wells_used)
    payload, curr_wells_used = convert_volumes_to_payload(
        plate_volumes, curr_wells_used
    )
    print(payload)
    payload["use_existing_resources"] = False
    steps_run, run_info = start_run_with_log_scraping(loop_protocol, payload, steps_run, experiment)
    img_path = run_info["Take Picture"]["action_msg"]
    return Path(img_path), curr_wells_used


def image_analysis(img_path, curr_wells_used):
    plate_colors_ratios = get_colors_from_file(img_path)[1]
    plate_colors_ratios = {a: b[::-1] for a, b in plate_colors_ratios.items()}
    current_plate = []
    wells_used = []
    for well in curr_wells_used:
        color = plate_colors_ratios[well]
        wells_used.append(well)
        current_plate.append(color)
    return current_plate


def calibrate(
    target_color: List[int],
    curr_wells_used: List[str],
    loop_protocol: str,
    exp_folder: Path,
    plate_max_volume: float,
    steps_run: List[Dict[str, Any]],
    pop_size: int,
    experiment: Any,
) -> Tuple[List[List[int]], List[int], List[str], List[Dict[str, Any]]]:
    """Performs a calibration run of the color picker system
    @Inputs:
        target_color: RGB Color the solver is attempting to match
        curr_wells_used: The wells that have so far been filled with liquid by the
        loop_protocol: Workcell protocol used to mix colors
        exp_folder: Folder where the experiment data is saved
        steps_run: The steps run in the workflow so far
    @Outputs:
        colors: colors that are combined linearly to visualize experimental colors
        target_color: RGB Color captured from camera image of actual plate based on the ratio calculated from the input RGB values
        steps_run:"""

    plate_volumes = (
        np.array(
            [
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
                # np.array(target_color) / sum(target_color)
            ]
        )
        * plate_max_volume
    )
    img_path, curr_wells_used = get_image(
        loop_protocol, plate_volumes, experiment, curr_wells_used, steps_run
    )
    print("starting")
    curr_wells_used = ["A1", "A2", "A3", "A4"]
    current_plate = image_analysis(img_path, curr_wells_used)
    # target_color = current_plate[3]
    # target_color = target_color.tolist()
    colors = current_plate[0:4]
    t = np.asarray(colors)
    colors = t.tolist()
    try:
        raise(Exception("test"))
        analytical_sol = np.linalg.lstsq(np.transpose(t), np.array(target_color))
        print("analytical_sol" + str(analytical_sol))
    except Exception as e:
        print("approximating")
        color_inverse = np.linalg.inv([[255, 0, 0], [0, 255, 0], [0, 0, 255]])
        analytical_sol = color_inverse @ np.array(target_color)
        analytical_sol = np.append(analytical_sol, 0.1)
    analytical_sol = analytical_sol / np.sum(analytical_sol) * plate_max_volume
    analytical_sol = analytical_sol.round(3)
    print("analytical solution: " + str(analytical_sol.tolist()))
    color_image = cv2.resize(
        np.asarray([colors]).astype(np.uint8),
        [pop_size * 50, 50],
        interpolation=cv2.INTER_NEAREST,
    )
    img_path, curr_wells_used = get_image(
        loop_protocol,
        [analytical_sol.tolist()],
        experiment,
        curr_wells_used,
        steps_run,
    )
    curr_wells_used = ["A1", "A2", "A3", "A4", "A5"]
    test_plate = image_analysis(img_path, curr_wells_used)
    analytical_sol = test_plate[4]
    print(analytical_sol)
    plt.imsave(exp_folder / "results" / "mixed_colors.png", color_image / 255)
    return (
        colors,
        target_color,
        curr_wells_used,
        steps_run,
        analytical_sol,
        color_inverse,
    )


if __name__ == "__main__":
    calibrate([120, 120, 120], [], "", "", 250, [], 4, [])
