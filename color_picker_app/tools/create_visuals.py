from typing import List, Any
import matplotlib.pyplot as plt
import cv2
import numpy as np
from pathlib import Path


def create_visuals(
    target_plate: List[List[int]],
    current_plate: List[List[int]],
    exp_folder: Path,
    current_iter: int,
    target_color: List[int],
    cur_best_color: List[int],
    pop_size: int,
    diffs: List[List[float]],
    solver_out_dim: int,
    solver: Any,
) -> None:
    # Creates the plots for the color picker system
    # @Inputs:
    # target_plate: Expected colors output by solver
    # current_plate: Colors measured from camera
    # exp_folder: Folder where the experiment data is saved
    # current_iter: Current iteration of the experiment
    # target_color: Color the solver is attempting to match
    # cur_best_color: The color mixed so far that best matches the target color
    # pop_size: The number of wells used per experiment run
    # @Outputs:
    # None

    solver.plot_diffs(diffs, exp_folder)
    plt.ion()
    f, axarr = plt.subplots(2, 2)
    # set figure size to 10x10
    f.set_figheight(10)
    f.set_figwidth(10)
    graph_vis = np.asarray(target_plate)
    graph_vis = graph_vis.reshape(*solver_out_dim)
    plate_vis = np.asarray(current_plate)
    plate_vis = plate_vis.reshape(*solver_out_dim)
    exp_p = np.asarray([graph_vis]).astype(np.uint8)
    exp_p = cv2.resize(
        np.asarray([graph_vis]).astype(np.uint8),
        [pop_size * 50, 50],
        interpolation=cv2.INTER_NEAREST,
    )
    axarr[0][0].imshow(exp_p)
    axarr[0][0].set_title("Experiment plate")
    axarr[1][0].imshow([plate_vis])
    axarr[1][0].set_title("Real plate")
    axarr[0][1].imshow([[target_color]])
    axarr[0][1].set_title("Target Color")
    axarr[1][1].imshow([[cur_best_color]])
    axarr[1][1].set_title("Experiment best color")
    f.suptitle("PAUSING HERE TO MOVE THE PLATE")
    f.canvas.draw()
    f.canvas.flush_events()
    plt.pause(0.001)
    plt.savefig(exp_folder / "results" / "run_summary.png", dpi=300)
    exp_p = np.asarray([graph_vis]).astype(np.uint8)
    exp_p = cv2.resize(
        np.asarray([graph_vis]).astype(np.uint8),
        [pop_size * 50, 50],
        interpolation=cv2.INTER_NEAREST,
    )
    plt.imsave(
        exp_folder / "results" / str("run_" + str(current_iter) + "_expected.png"),
        exp_p,
    )
    np.asarray([plate_vis]).astype(np.uint8)
    real_p = cv2.resize(
        np.asarray([plate_vis]).astype(np.uint8),
        [pop_size * 50, 50],
        interpolation=cv2.INTER_NEAREST,
    )
    plt.imsave(
        exp_folder / "results" / ("run_" + str(current_iter) + "_measured.png"), real_p
    )
    plt.imsave(
        exp_folder / "results" / "target_color.png", np.asarray([[target_color]]) / 255
    ),
    plt.imsave(
        exp_folder / "results" / "best_color.png", np.asarray([[cur_best_color]]) / 255
    )


def create_target_plate(plate_volumes: List[List[float]], colors: List[List[int]]):
    norms = []
    for i in plate_volumes:
        norms.append(i / sum(i))
    target_plate = np.array(norms) @ np.array(colors)
    target_plate = target_plate.tolist()
    return target_plate
