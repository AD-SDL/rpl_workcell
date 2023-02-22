#!/usr/bin/env python3

import logging
from pathlib import Path
from argparse import ArgumentParser
from typing import List, Dict, Any, Tuple
from itertools import product
from typing import Optional
from threading import Thread

import copy
import numpy as np

from rpl_wei import WEI
from plate_color_analysis import get_colors_from_file
from evolutionary_solver import EvolutionaryColorSolver

curr_wells_used = []

def new_plate():
    well_rows = ["A", "B", "C", "D", "E", "F", "G", "H"]
    well_cols = [str(elem) for elem in range(1, 13)]
    well_names = ["".join(elem) for elem in product(well_rows, well_cols)]
    return well_names

def convert_volumes_to_payload(volumes: List[List[float]]) -> Dict[str, Any]:
 
    well_names = new_plate()
 
    if len(volumes) <= len(well_names) and len(volumes) <= 96 - len(curr_wells_used):
        curr_wells_available = copy.copy(well_names)
        for i in curr_wells_used:
            curr_wells_available.remove(i)

    r_vol, g_vol, b_vol = [], [], []
    dest_wells = []
    for color, well in zip(volumes, curr_wells_available):
    # for color, well in zip(volumes, well_names):
        r, g, b = color
        r_vol.append(r)
        g_vol.append(g)
        b_vol.append(b)
        dest_wells.append(well)
        curr_wells_used.append(well)
    return {
        "red_volumes": r_vol,
        "green_volumes": g_vol,
        "blue_volumes": b_vol,
        "destination_wells": dest_wells,
    }

def wei_run_flow(wf_file_path, payload):
    wei_client = WEI(wf_file_path)
    run_info = wei_client.run_workflow(payload=payload)
    return run_info

class ThreadWithReturnValue(Thread):
    
    def __init__(self, group=None, target=None, name=None,
                 args=(), kwargs={}, Verbose=None):
        Thread.__init__(self, group, target, name, args, kwargs)
        self._return = None

    def run(self):
        if self._target is not None:
            self._return = self._target(*self._args,
                                                **self._kwargs)
    def join(self, *args):
        Thread.join(self, *args)
        return self._return

def run(
    target_color: List[float],
    wei_client: Optional["WEI"] = None,
    solver: EvolutionaryColorSolver = EvolutionaryColorSolver,
    exp_budget: int = 96 * 3,
    pop_size: int = 96,
    init_protocol = None,
    loop_protocol = None,
    final_protocol = None,
    solver_out_dim: Tuple[int, int] = (96, 3),
    plate_max_volume: float = 275.0,
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
    plate_n=0
    plate_total = int(exp_budget/96)
    current_iter = 0 
    cur_best_color = None
    cur_best_diff = float("inf")
    runs_list = []
    ot2_iter = 0
    new_plate=True
    payload={}
    

    while num_exps + pop_size <= exp_budget:
        report={
            "plate_N": plate_n,
            "target_color":'',
            "wells":'',
            "tried_values":'',
            "exp_volumes":'',
            "results":'',
            "differences":'',
            "best_on_plate":'',
            "pos_on_plate":'',
            "best_so_far":'',
            "experiments_so_far":runs_list,
        }
        print(
            "Starting experiment, can run at least one iteration:",
            num_exps + pop_size <= exp_budget,
        )

        #grab new plate if experiment starting or current plate is full
        if new_plate or current_iter==0:
            print('Grabbing New Plate')
            iter_thread=ThreadWithReturnValue(target=wei_run_flow,kwargs={'wf_file_path':init_protocol,'payload':payload})
            iter_thread.run()
            curr_wells_used = []
            new_plate = False

        #resets OT2 resources (or not)
        if ot2_iter == 0: 
            payload['use_existing_resources'] = False # This assumes the whole plate was reset and all tips are new
        else: 
            payload['use_existing_resources'] = True 

        # Calculate volumes and current wells for creating the OT2 protocol
        ## FUNCX
        plate_volumes = solver.run_iteration( 
            target_color,
            current_plate,
            out_dim=(pop_size, 3),
            pop_size=pop_size,
            return_volumes=True,
            return_max_volume=plate_max_volume,
        )
        target_plate = [
                (np.asarray(elem) / 275).tolist() for elem in plate_volumes
            ]
        payload = convert_volumes_to_payload(plate_volumes)
        
        iter_thread=ThreadWithReturnValue(target=wei_run_flow,kwargs={'wf_file_path': loop_protocol, 'payload':payload})
        iter_thread.run()
        run_info = iter_thread._return
        runs_list.append(run_info)
        ot2_iter += 1

        used_wells = current_iter*pop_size 
        if used_wells + pop_size > 96: #if we have used all wells or not enough for next iter (thrash plate, start from scratch)
            print('Thrasing Used Plate')
            iter_thread=ThreadWithReturnValue(target=wei_run_flow,kwargs={'wf_file_path':final_protocol,'payload':payload})
            iter_thread.run()
            new_plate = True

        # analize image
        # output should be list [pop_size, 3]
        img_path = run_info["run_dir"] / "results" / "final_image.jpg"
        plate_colors_ratios = get_colors_from_file(img_path)[1] ##FUNCX
        # Swap BGR to RGB
        plate_colors_ratios = {a:b[::-1] for a,b in plate_colors_ratios.items()}  
        
        current_plate = []
        wells_used = []
        with open(run_info["run_dir"] / "results" / "plate_all_colors.csv", "w") as f:
            for well in payload["destination_wells"]:
                color = plate_colors_ratios[well]
            # for well, color in list(plate_colors_ratios.items())[:pop_size]:
                f.write("%s, %s,%s,%s" % (well, color[0], color[1], color[2]))
                f.write("\n")
                wells_used.append(well)
                if well in payload['destination_wells']:
                    current_plate.append(color)
        
        
        ## save those and the initial colors, etc
        plate_best_color_ind = solver._find_best_color(current_plate, target_color)
        plate_best_color = current_plate[plate_best_color_ind]
        plate_best_diff = solver._color_diff(plate_best_color, target_color)

        #Find best colors
        if plate_best_diff < cur_best_diff:
            cur_best_diff = plate_best_diff
            cur_best_color = plate_best_color

        ##update numbers (seems redundant)
        plate_n = plate_n + 1 
        current_iter += 1
        num_exps += pop_size

        ##Plot review
        if show_visuals:
            import matplotlib.pyplot as plt
            plt.ion()
            f, axarr = plt.subplots(2, 2)
            # set figure size to 10x10
            f.set_figheight(10)
            f.set_figwidth(10)
            graph_vis = np.asarray(target_plate)
            graph_vis = graph_vis.reshape(*solver_out_dim)
            plate_vis = np.asarray(current_plate)
            plate_vis = plate_vis.reshape(*solver_out_dim)
            target_color = target_color
            axarr[0][0].imshow([graph_vis])
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
            # plt.imsave(run_info["run_dir"] / "results" / "experiment_summary.jpg")
        
        
        report={
            "plate_N": plate_n,
            "target_color": target_color,
            "wells": wells_used,
            "tried_values": target_plate,
            "exp_volumes": plate_volumes,
            "results":current_plate,
            "differences": 4,
            "best_on_plate": plate_best_color,
            "pos_on_plate": plate_best_color_ind,
            "best_so_far": cur_best_color,
            "experiments_so_far":runs_list,
        }
    iter_thread=ThreadWithReturnValue(target=wei_run_flow,kwargs={'wf_file_path':final_protocol,'payload':payload})
    iter_thread.run()
    if show_visuals:
        import matplotlib.pyplot as plt

        f, axarr = plt.subplots(1, 2)
        # set figure size to 10x10
        f.set_figheight(10)
        f.set_figwidth(10)
        axarr[0].imshow([[cur_best_color]])
        axarr[0].set_title("Experiment best color, diff: " + str(cur_best_diff))
        axarr[1].imshow([[target_color]])
        axarr[1].set_title("Target Color")

        plt.show()

        plt.savefig(run_info["run_dir"] / "results" / "final_plot.png", dpi=300)

    print("This is our best color so far")
    print(cur_best_color)
    print("Runs on this experiment")
    print(runs_list)

def parse_args():
    parser = ArgumentParser()
    parser.add_argument(
        "--pop_size",
        default=96,
        type=int,
        help="Population size (num wells to fill per iter)",
    )
    parser.add_argument(
        "--exp_budget", default=96 * 3, type=int, help="Experiment budget"
    )
    parser.add_argument(
        "--target","-t", default="[101, 148, 30]", help="Color Target"
    )
    parser.add_argument("--plate_max_volume", default=275.0, type=float)
    return parser.parse_args()


def main(args):
    
    target_ratio = eval(args.target)
    wf_dir = Path('/home/rpl/workspace/rpl_workcell/color_picker/workflows')
    wf_get_plate = wf_dir / 'cp_wf_newplate.yaml'
    wf_trash_plate = wf_dir / 'cp_wf_trashplate.yaml'
    wf_mix_colors = wf_dir / 'cp_wf_mixcolor.yaml'

    run_args = {}
    run_args["target_color"] = target_ratio
    run_args["init_protocol"] = wf_get_plate
    run_args["loop_protocol"] = wf_mix_colors
    run_args["final_protocol"] = wf_trash_plate
    run_args["solver"] = EvolutionaryColorSolver
    run_args["exp_budget"] = args.exp_budget
    run_args["pop_size"] = args.pop_size
    run_args["solver_out_dim"] = (args.pop_size, 3)
    run_args["plate_max_volume"] = args.plate_max_volume

    run(**run_args)


if __name__ == "__main__":
    args = parse_args()
    main(args)
