#!/usr/bin/env python3

from pathlib import Path
from argparse import ArgumentParser
from typing import List, Dict, Any, Tuple
from typing import Optional
import json
import numpy as np
import os, shutil

#For extracting colors from each plate
from tools.plate_color_analysis import get_colors_from_file

#Different possible solvers for the color_picker problem
from solvers.bayes_solver import BayesColorSolver
from solvers.evolutionary_solver import EvolutionaryColorSolver
from solvers.aggressive_genetic_solver import AggroColorSolver
from funcx import FuncXExecutor

from datetime import datetime

#For publishing to RPL Portal
from tools.publish_v2 import publish_iter

#For creating a payload that the OT2 will accept from the solver output
from tools.color_utils import convert_volumes_to_payload

#For running WEI flows
from tools.run_flow import run_flow

#for measuring the three mixed colors for calibration and for 
# ensuring the target color is in the right color space
from tools.calibrate import calibrate

#For constructing the plots for each run
from tools.create_visuals import create_visuals, create_target_plate

#DEF TODO:what does it mean?
MAX_PLATE_SIZE = 96
    
def run(
    exp_type: str,
    exp_label: str = "",
    exp_path: str = "",
    target_color: List[float] = [],
    solver: BayesColorSolver = BayesColorSolver,
    solver_name: str = "Evolutionary Solver",
    exp_budget: int = MAX_PLATE_SIZE * 3,
    pop_size: int = MAX_PLATE_SIZE,
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

    #workflows used
    wf_dir = Path('/home/rpl/workspace/rpl_workcell/color_picker/workflows')
    init_protocol = wf_dir / 'cp_wf_newplate.yaml'
    loop_protocol = wf_dir / 'cp_wf_mixcolor.yaml'
    final_protocol = wf_dir / 'cp_wf_trashplate.yaml'
        
    #Constants
    solver_out_dim = (pop_size, 3)
    use_funcx = False
    funcx_local_ep = '299edea0-db9a-4693-84ba-babfa655b1be' # local

    exp_path =  Path(exp_path)
    exp_label = Path(exp_label)
    exp_folder = exp_path / exp_label
    if not (os.path.isdir(exp_path)):
        os.makedirs(exp_path)
        print("makingdir")
    if not (os.path.isdir(exp_folder)):
        os.mkdir(exp_folder)
    if not (os.path.isdir(exp_folder/"results")):
        os.mkdir(exp_folder/"results") 
    
    #Resource Tracking:
    plate_n=1 #total number of plates
    current_iter = 0 #total number of itertions
    num_exps = 0 #total number of wells used
    curr_wells_used = [] #list of all wells used
    
    #Information Tracking:
    current_plate = None
    cur_best_color = None
    cur_best_diff = float("inf") #Min difference between color found and target color so far
    runs_list = [] #List of info about each run of the experiment
    start = datetime.now()
    time_to_best = str(start - start)
    diffs = [] #List of all diffs from all runs of the experiment
    new_plate=True
    payload={}  #Payload to be sent to the WEI runs
    
    
    while num_exps + pop_size <= exp_budget:
        new_run = {}
        steps_run = []
       
        print("Starting experiment, can run at least one iteration:",
            num_exps + pop_size <= exp_budget,)
        print('Starting iteration ' + str(current_iter))
        
        #grab new plate if experiment starting or current plate is full
        if new_plate or current_iter==0:
            print('Grabbing New Plate')
            steps_run, _ = run_flow(init_protocol, payload, steps_run)
            curr_wells_used = []
            new_plate = False
            
            if current_iter == 0:
                #Run the calibration protocol that gets the colors being mixed and ensures the target color is within the possible color space
                colors, target_color, curr_wells_used, steps_run = calibrate(target_color, curr_wells_used, loop_protocol, exp_folder, plate_max_volume, steps_run, pop_size)
            else:
            #save the old plate picture and increment to a new plate
               filename = "plate_"+ str(plate_n)+".jpg"
               shutil.copy2(run_info["run_dir"]/ "results"/"plate_only.jpg",  (exp_folder/"results"/filename))
               plate_n = plate_n + 1
            
        # Calculate volumes and current wells for creating the OT2 protocol
        plate_volumes = solver.run_iteration(
            target_color, 
            current_plate,
            pop_size=pop_size,
            out_dim=(pop_size, 3),
            return_volumes=True,
            return_max_volume=plate_max_volume,
        )

        #Only for visualization, Perform a linear combination of the next colors being tried to show what the solver expects to create on this run. 
        target_plate = create_target_plate(plate_volumes, colors)
        #Assign volumes to wells and colors and make a payload compatible with the OT2 protopiler
        payload, curr_wells_used = convert_volumes_to_payload(plate_volumes, curr_wells_used)
        
        #resets OT2 resources (or not)
        if current_iter == 0: 
            payload['use_existing_resources'] = False # This assumes the whole plate was reset and all tips are new
        else: 
            payload['use_existing_resources'] = True 
        
        #Run the flow to mix all of the colors 
        steps_run, run_info = run_flow(loop_protocol, payload, steps_run)
        run_path =  run_info["run_dir"].parts[-1]
        if not (os.path.isdir(exp_folder / run_path)):
            os.mkdir(exp_folder / run_path)
        runs_list.append(run_info)
        used_wells = (len(curr_wells_used))
        if used_wells + pop_size > MAX_PLATE_SIZE: #if we have used all wells or not enough for next iter (thrash plate, start from scratch)
            print('Trashing Used Plate')
            steps_run, _ = run_flow(final_protocol, payload, steps_run)
            new_plate = True
            curr_wells_used = []

        # Analyze image
        # output should be list [pop_size, 3]   
        fname = "final_image.jpg" 
        img_path = run_info["run_dir"]/ "results" / fname
        
        if use_funcx:
            print("funcx started")
            fx = FuncXExecutor(endpoint_id=funcx_local_ep)
            fxresult = fx.submit(get_colors_from_file, img_path)
            fxresult = fx.submit(get_colors_from_file, img_path)
            plate_colors_ratios = fxresult.result()[1]
            print("funcx finished")
        else: 
            plate_colors_ratios = get_colors_from_file(img_path)[1]

        filename = "plate_"+ str(plate_n)+".jpg"
        #Copy the plate image into the experiment folder
        shutil.copy2(run_info["run_dir"]/ "results"/"plate_only.jpg",  (exp_folder/"results"/filename))
        # Swap BGR to RGB
        plate_colors_ratios = {a:b[::-1] for a,b in plate_colors_ratios.items()}  
        #Find the colors to be processed by the solver
        current_plate = []
        wells_used = []
        for well in payload["destination_wells"]:
            color = plate_colors_ratios[well]
            wells_used.append(well)
            current_plate.append(color)
        
        ## save those and the initial colors, etc
        plate_best_color_ind, plate_diffs = solver._find_best_color(current_plate, target_color, cur_best_color)
        plate_best_color = current_plate[plate_best_color_ind]
        plate_best_diff = solver._color_diff(plate_best_color, target_color)
        diffs.append(plate_diffs)
        #Find best colors
        if plate_best_diff < cur_best_diff:
            cur_best_diff = plate_best_diff
            cur_best_color = plate_best_color
            time_to_best = str(datetime.now() - start)

<<<<<<< HEAD
        ##update numbers
=======
        ##update numbers (seems redundant)
   
>>>>>>> main
        current_iter += 1
        num_exps += pop_size
        ##Plot review
        create_visuals(target_plate, current_plate, exp_folder, current_iter, target_color, cur_best_color, pop_size, diffs, solver_out_dim, solver)
        runs = []
        report = {}
        if (exp_folder/"results"/"exp_data.txt").is_file():
            with open(exp_folder/"results"/"exp_data.txt", "r") as f:
                report = json.loads(f.read())
            runs = report["runs"]
        #Create new run log
        new_run = [{
        "run_number": current_iter, 
        "run_label": str(run_path),
        "plate_N": plate_n,
        "tried_values": target_plate,
        "exp_volumes": plate_volumes,
        "wells": list(wells_used),
        "results": list(map(lambda x: x.tolist(), current_plate)),
        "differences": plate_diffs.tolist(),
        "best_on_plate": plate_best_color.tolist(),
        "pos_on_plate": plate_best_color_ind.tolist(),
        "plate_best_diff": plate_best_diff,
        "best_so_far": cur_best_color.tolist()}]
        #prepend new run to all run logs
        runs = new_run + runs
        #update report
        report.update({
            "experiment": str(exp_label),
            "exp_type": "color_picker",
            "solver": solver_name,
            "plate_N": plate_n,
            "target_color": target_color,
            "best_so_far": cur_best_color.tolist(),
            "best_diff": cur_best_diff,
            "time_to_best": time_to_best,
            "colors": colors, 
            "total_time": str(datetime.now() - start),
            "total_iterations": current_iter, 
            "pop_size": pop_size, 
            "exp_budget": exp_budget,
            "wf_steps": steps_run,
            "runs": runs
        })
        
        #Save run report
        with open(exp_folder/"results"/ "exp_data.txt", "w") as f:
            report_js = json.dumps(report, indent=4)
            f.write(report_js)
        #Save overall results
        print("publishing:")
        publish_iter(exp_folder/"results", exp_folder)
        
    #Trash plate after experiment
    shutil.copy2(run_info["run_dir"]/ "results"/"plate_only.jpg",  (exp_folder/"results"/f"plate_{plate_n}.jpg"))
    steps_run, _ = run_flow(final_protocol, payload, steps_run)
    
    print("This is our best color so far")
    print(cur_best_color)
    print("Runs on this experiment")
    print(runs_list)
   
def parse_args():
    parser = ArgumentParser()
    parser.add_argument(
        "--pop_size",
        default=4,
        type=int,
        help="Population size (num wells to fill per iter)",)
    parser.add_argument(
        "--exp_budget", default=8, type=int, help="Experiment budget")
    parser.add_argument(
        "--target","-t", default=str(np.random.randint(0, 255, 3).tolist()), help="Color Target")
    parser.add_argument(
        "--solver", default="Evo", help="Bay = Bayes, Evo = Evolutionary, Agg = Aggro")
    parser.add_argument("--plate_max_volume", default=275.0, type=float)
    return parser.parse_args()

if __name__ == "__main__":

    #parser
    args = parse_args()

    #target color
    target_ratio = eval(args.target)

    exp_label = "ColorPicker_" + str(target_ratio[0]) +"_" + str(target_ratio[1]) + "_" +  str(target_ratio[2]) + "_" + str(datetime.date(datetime.now()))
    exp_path = '/home/rpl/experiments'
    exp_type = 'color_picker'
    if args.solver:
            if args.solver == "Bay":
                solver = BayesColorSolver
                solver_name = "Bayesian Solver"
            elif args.solver == "Evo":
                solver_name = "Evolutionary Solver"
                solver = EvolutionaryColorSolver
            elif args.solver == "Agg":
                solver = AggroColorSolver
                solver_name = "Aggressive Genetic Solver"
    else:
        solver = EvolutionaryColorSolver
        solver_name = "Evolutionary Solver"
    print(solver)
    print(target_ratio)
    print(exp_label)
    run_args = {"target_color": [120, 120, 120],
                "solver" : solver,
                "solver_name" : solver_name,
                "exp_budget" : args.exp_budget,
                "pop_size": args.pop_size,
                "plate_max_volume": args.plate_max_volume,
                "exp_label": exp_label,
                "exp_path": exp_path,
                "exp_type": exp_type}
    run(**run_args)
