#!/usr/bin/env python3

from pathlib import Path
from argparse import ArgumentParser
from typing import List
import json
import numpy as np
import os
import shutil

# For extracting colors from each plate
from tools.plate_color_analysis import get_colors_from_file

# Different possible solvers for the color_picker problem
from solvers.bayes_solver import BayesColorSolver
# from funcx import FuncXExecutor

from datetime import datetime
import random

# For publishing to RPL Portal
from tools.publish_v2 import publish_iter

# For creating a payload that the OT2 will accept from the solver output
from tools.color_utils import convert_volumes_to_payload

# For running WEI flows
from tools.start_run import start_run_with_log_scraping

from tools.collect_files import collect_files

# for measuring the three mixed colors for calibration and for
# ensuring the target color is in the right color space
from tools.calibrate import calibrate

# For constructing the plots for each run
from tools.create_visuals import create_visuals, make_target_plate

from wei import ExperimentClient


PLATE_MAX_VOLUME = 275.0
PLATE_WELLS = 96
rpl_workcell_path = Path(__file__).parent.parent.parent
app_dir =  rpl_workcell_path / "applications" / "color_picker_app"
wf_dir = app_dir / "workflows"
init_protocol = wf_dir / "cp_wf_newplate.yaml"
loop_protocol = wf_dir / "cp_wf_mixcolor.yaml"
ot2_protocol = (
    app_dir
    / "protocol_files"
    / "combined_protocol.yaml"
)
final_protocol = wf_dir / "cp_wf_trashplate.yaml"
reset_colors_wf = wf_dir / "cp_wf_reset_colors.yaml"
refill_barty = wf_dir / "cp_wf_replenish.yaml"





def run(
    exp_label: str = "",
    exp_path: str = "",
    target_color: List[float] = [125, 0, 252],
    exp_budget: int = 12,
    pop_size: int = 4,
) -> None:
   exp = ExperimentClient(
        "mj.cels.anl.gov",
        "8000",
        "Color_Picker",
    )

   #init Variables
   plate_count = 0
   total_samples = 0
   need_new_plate = True
   previous_ratios = None
   colors_tested = []
   solver = BayesColorSolver(pop_size)
   exp.start_run(reset_colors_wf.resolve(), blocking=False)
   exp.log_loop_start("Main Loop")
   while total_samples < exp_budget:
        if need_new_plate:
            exp.log_decision("Need New Plate",  (need_new_plate))
            exp.start_run(init_protocol)
            wells_used = []
            plate_count += 1
            need_new_plate = False
    
    
    
        exp.log_local_compute("solver.run_iteration")
        
        if previous_ratios:
            grades = solver._grade_population(colors_tested, target_color)
        else:
            grades = None
        print(previous_ratios)
        print(grades)
        test_ratios = solver.run_iteration(previous_ratios, grades) 
        if previous_ratios == None:
            previous_ratios = test_ratios
        else: 
            previous_ratios += test_ratios      
        payload, wells_used = convert_volumes_to_payload(
            np.multiply(test_ratios, PLATE_MAX_VOLUME), wells_used
        )
        print(payload)
        target_plate = make_target_plate(test_ratios)
        if total_samples == 0:
            payload[
                "use_existing_resources"
            ] = False  # This assumes the whole plate was reset and all tips are new
        else:
            payload["use_existing_resources"] = True
        payload["config_path"] = str(ot2_protocol.resolve())
        

        run_info= exp.start_run(loop_protocol.resolve(), payload)

        if(len(wells_used) + pop_size) > PLATE_WELLS:
            exp.start_run(final_protocol)
            need_new_plate = True

        img_path = Path(exp.get_wf_result_file(run_info["run_id"], run_info["hist"]["Take Picture"]["action_msg"], Path.home() / "workspace"/  "results" / "final_img.jpg"))
        print(img_path)
        exp.log_local_compute("get_colors_from_file")

        measured_colors = get_colors_from_file(img_path)[1]
        measured_colors = {a: b[::-1] for a, b in measured_colors.items()}
        for well in payload["destination_wells"]:
            color = measured_colors[well]
            colors_tested.append(color)

        total_samples += pop_size
   if need_new_plate == False:
        exp.start_run(final_protocol)

if __name__ == "__main__":
    run()


