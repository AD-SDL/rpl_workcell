#!/usr/bin/env python3

import logging
from pathlib import Path
from argparse import ArgumentParser
from typing import List, Dict, Any, Tuple
from itertools import product
from uuid import UUID

import numpy as np

from evolutionary_solver import EvolutionaryColors
from rpl_wei.wei_workcell_base import WEI
from rpl_wei.data_classes import Module, Step

try:
    import rclpy
    from wei_executor.weiExecutorNode import weiExecNode

    # Can this be an init to an executor callback?
    rclpy.init()
    wei_execution_node = weiExecNode()
except ImportError:
    pass


def wei_service_callback(step: Step, **kwargs):

    module: Module = kwargs["step_module"]

    msg = {
        "node": module.config["ros_node"],
        "action_handle": step.command,
        "action_vars": step.args,
    }
    print("\n Callback message:")
    print(msg)
    print()

    wei_execution_node.send_wei_command(
        msg["node"], msg["action_handle"], msg["action_vars"]
    )


def testing_callback(step: Step, **kwargs):
    print(step)
    print()


def silent_callback(step: Step, **kwargs):
    pass


def parse_args():
    parser = ArgumentParser()
    parser.add_argument(
        "-wf", "--workflow", help="Path to workflow file", type=Path, required=True
    )
    parser.add_argument(
        "--pop_size",
        default=96,
        type=int,
        help="Population size (num wells to fill per iter)",
    )
    return parser.parse_args()


def convert_volumes_to_payload(
    volumes: List[List[float]], max_vol: float = 30.0
) -> Dict[str, Any]:
    well_rows = ["A", "B", "C", "D", "E", "F", "G", "H"]
    well_cols = [str(elem) for elem in range(1, 13)]
    well_names = ["".join(elem) for elem in product(well_rows, well_cols)]
    assert len(volumes) <= len(well_names)
    r_vol, g_vol, b_vol = [], [], []
    water_volumes = []
    dest_wells = []
    for color, well in zip(volumes, well_names):
        r, g, b = color
        r_vol.append(r)
        g_vol.append(g)
        b_vol.append(b)
        water_volumes.append(max_vol - (sum(color)))
        dest_wells.append(well)

    return {
        "red_volumes": r_vol,
        "green_volumes": g_vol,
        "blue_volumes": b_vol,
        "water_volumes": water_volumes,
        "destination_wells": dest_wells,
    }


def run(
    wei_client: WEI,
    protocol_id: UUID,
    solver: EvolutionaryColors,
    exp_budget: int = 96 * 3,
    solver_out_dim: Tuple[int] = (96, 3),
) -> None:
    """
    Steps
    1. random init
    2. run flow with init
    2. grade population
    do
        1. update population
        2. run workflow with this payload
        3. grade population
    while num_exps < threshhold and solution not found
    """
    show_visuals = True
    num_exps = 0
    current_plate = None
    while num_exps + solver.pop_size <= exp_budget:
        plate_volumes = solver.run_iteration(
            current_plate, out_dim=(solver.pop_size, 3), return_volumes=True
        )
        payload = convert_volumes_to_payload(plate_volumes)
        wei_client.run_workflow(
            workflow_id=protocol_id,
            payload=payload,
            callbacks=[wei_service_callback],
        )
        # need to return run_id from wei_client
        """run_id = wei_client.run_workflow(workflow_id=protocol_id, payload=payload)
        # Need info from camera something like this
        result = wei_client.get_workflow_results(run_id)
        plate_colors_ratios = solver.read_camera(result)"""
        # going to convert back to ratios for now
        plate_color_ratios = [
            (np.asarray(elem) / 30).tolist() for elem in plate_volumes
        ]
        current_plate = plate_color_ratios
        num_exps += solver.pop_size

        if show_visuals:
            import matplotlib.pyplot as plt

            f, axarr = plt.subplots(1, 2)
            graph_vis = np.asarray(current_plate)
            graph_vis = graph_vis.reshape(*solver_out_dim)
            target_color = solver.target.get_value_tuple()
            axarr[0].imshow([graph_vis])
            axarr[0].set_title("Experiment plate")
            axarr[1].imshow([[target_color]])
            axarr[1].set_title("Target Color")
            f.suptitle("PAUSING HERE TO MOVE THE PLATE")
            plt.show()

    if show_visuals:
        f, axarr = plt.subplots(1, 2)
        best_color = solver.current_best_color.color
        target_color = solver.target.get_value_tuple()
        axarr[0].imshow([[best_color]])
        axarr[0].set_title("Experiment best color")
        axarr[1].imshow([[target_color]])
        axarr[1].set_title("Target Color")

        plt.show()


def main(args):
    wf_file_path = args.workflow.resolve()

    wei_client = WEI(
        wf_file_path,
        workcell_log_level=logging.DEBUG,
        workflow_log_level=logging.DEBUG,
    )
    protocol_id = list(wei_client.get_workflows().keys())[0]

    target_ratio = [101, 173, 95]
    mixing_colors = [[255, 0, 0], [0, 255, 0], [0, 0, 255]]
    solver = EvolutionaryColors(
        target=target_ratio,
        mixing_colors=mixing_colors,
        pop_size=args.pop_size,
    )

    run(
        wei_client,
        protocol_id,
        solver,
        solver_out_dim=(args.pop_size, 3),
        exp_budget=24,
    )


if __name__ == "__main__":
    args = parse_args()
    main(args)
