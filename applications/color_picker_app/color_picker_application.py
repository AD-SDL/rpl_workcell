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
from solvers.evolutionary_solver import EvolutionaryColorSolver
from solvers.aggressive_genetic_solver import AggroColorSolver
from solvers.solver import Solver
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
from tools.create_visuals import create_visuals, create_target_plate

from wei import ExperimentClient

MAX_PLATE_SIZE = 96


def run(
    exp_label: str = "",
    exp_path: str = "",
    target_color: List[float] = [],
    solver: BayesColorSolver = BayesColorSolver,
    solver_name: str = "Evolutionary Solver",
    exp_budget: int = MAX_PLATE_SIZE * 3,
    pop_size: int = MAX_PLATE_SIZE,
    plate_max_volume: float = 275.0,
    rpl_workcell_path: Path = Path(__file__).parent.parent.parent,
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
    while num_exps < threshold and solution not found
    """

    # workflows used
    app_dir = rpl_workcell_path / "applications" / "color_picker_app"
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

    # Constants
    solver_out_dim = (pop_size, 3)
    use_globus_compute = False
    compute_local_ep = "299edea0-db9a-4693-84ba-babfa655b1be"  # local

    exp_path: Path = Path(exp_path)
    exp_label = Path(exp_label + str(random.randint(0, 1000)))
    exp_folder = exp_path / exp_label
    exp_path.mkdir(parents=True, exist_ok=True)
    Path("./publish").mkdir(parents=True, exist_ok=True)
    exp_folder.mkdir(parents=True, exist_ok=True)
    (exp_folder / "results").mkdir(parents=True, exist_ok=True)
    exp = ExperimentClient(
        "mj.cels.anl.gov",
        "8000",
        "Color_Picker",
    )
    output_dir = (
                str(Path.cwd())
                + "/experiment_results/"
                + str("Color_Picker")
                + "_id_"
                + exp.experiment_id
    )
    Path(output_dir).mkdir(parents=True, exist_ok=True)
            
    # Resource Tracking:
    plate_n = 1  # total number of plates
    current_iter = 0  # total number of iterations
    num_exps = 0  # total number of wells used
    curr_wells_used = []  # list of all wells used

    # Information Tracking:
    prev_colors = None
    cur_best_color = None
    previous_ratios = None
    prev_diffs = None
    plate_diffs = None
    plate_colors = None
    cur_best_diff = float(
        "inf"
    )  # Min difference between color found and target color so far
    runs_list = []  # List of info about each run of the experiment
    start = datetime.now()
    time_to_best = str(start - start)
    diffs = []  # List of all diffs from all runs of the experiment
    new_plate = True
    payload = {}  # Payload to be sent to the WEI runs
    colors_used = [0, 0, 0]

    # Reset Colors
    
    exp.start_run(reset_colors_wf.resolve(), blocking=False)
    exp.events.log_loop_start("Main Loop")
    while num_exps + pop_size <= exp_budget:
        new_run = {}
        steps_run = []

        print(
            "Starting experiment, can run at least one iteration:",
            num_exps + pop_size <= exp_budget,
        )
        print("Starting iteration " + str(current_iter))

        # grab new plate if experiment starting or current plate is full
        exp.events.log_decision("Need New Plate", (new_plate or current_iter == 0))
        if new_plate or current_iter == 0:
            print('Grabbing New Plate')
            steps_run, run_info = start_run_with_log_scraping(
                    init_protocol, payload, steps_run, exp
            )
            curr_wells_used = []
            new_plate = False
            exp.events.log_decision("Need Calibration", (current_iter == 0))
            if current_iter == 0:
                # Run the calibration protocol that gets the colors being mixed and ensures the target color is within the possible color space
                (
                    colors,
                    target_color,
                    curr_wells_used,
                    steps_run,
                    analytical_sol,
                    color_inverse,
                ) = calibrate(
                    target_color,
                    curr_wells_used,
                    loop_protocol,
                    exp_folder,
                    plate_max_volume,
                    steps_run,
                    pop_size,
                    exp,
                    ot2_protocol,
                )
                analytical_score = solver._grade_population(
                    [analytical_sol], target_color
                )[0]
                pass
            else:
                # save the old plate picture and increment to a new plate
                filename = "plate_" + str(plate_n) + ".jpg"
                shutil.copy2(
                    Path(str(run_info["run_dir"]).replace("/home/app", str(Path.home()))) / "results" / "plate_only.jpg",
                    (exp_folder / "results" / filename),
                )
                plate_n = plate_n + 1

        # Calculate volumes and current wells for creating the OT2 protocol

        exp.events.log_local_compute("solver.run_iteration")
        if plate_colors:
            prev_diffs = solver._grade_population(prev_colors, target_color)
        previous_ratios = solver.run_iteration(previous_ratios, prev_diffs)
        # Only for visualization, Perform a linear combination of the next colors being tried to show what the solver expects to create on this run.
        print(previous_ratios)
        print(colors)
        target_plate = create_target_plate(previous_ratios, colors)
        print(target_plate)
        # Assign volumes to wells and colors and make a payload compatible with the OT2 protopiler
        payload, curr_wells_used = convert_volumes_to_payload(
            np.multiply(previous_ratios, plate_max_volume), curr_wells_used
        )

        # resets OT2 resources (or not)
        if current_iter == 0:
            payload[
                "use_existing_resources"
            ] = False  # This assumes the whole plate was reset and all tips are new
        else:
            payload["use_existing_resources"] = True

        payload["config_path"] = str(ot2_protocol.resolve())

        # Run the flow to mix all of the colors
        run_info= exp.start_run(
        loop_protocol.resolve(), payload=payload, simulate=False, blocking=True
        )
        print(run_info)
        # run_path = run_info["run_dir"].parts[-1]
        # if not (os.path.isdir(exp_folder / run_path)):
        #     os.mkdir(exp_folder / run_path)
        # runs_list.append(run_info)
        used_wells = len(curr_wells_used)
        if (
            used_wells + pop_size > MAX_PLATE_SIZE
        ):  # if we have used all wells or not enough for next iter (thrash plate, start from scratch)
            print("Trashing Used Plate")
            steps_run, _ = start_run_with_log_scraping(
                final_protocol, payload, steps_run, exp
            )
            new_plate = True
            curr_wells_used = []

        # Checking whether to refill ink.
        curr_colors_used = [
            sum(payload["color_A_volumes"]),
            sum(payload["color_B_volumes"]),
            sum(payload["color_C_volumes"]),
            sum(payload["color_D_volumes"]),
        ]
        comb_list = [colors_used, curr_colors_used]
        colors_used = [sum(vols) for vols in zip(*comb_list)]
        print("Total vol of colors used so far:", colors_used)

        for i in colors_used:
            if i > 5000:
                print(i, ": Has used 5 mL of ink, Barty refill command")
                i = 0
                print("Updated colors_used:", colors_used)

        for i, color_vol in enumerate(colors_used):
            exp.events.log_decision(
                "Need Ink", (color_vol >= 5000)
            )  # 5 mL, change to whatever threshold.
            if color_vol >= 5000:
                if i == 0:
                    payload["refill_motor"] = ["motor_1"]
                elif i == 1:
                    payload["refill_motor"] = ["motor_2"]
                elif i == 2:
                    payload["refill_motor"] = ["motor_3"]
                elif i == 3:
                    payload["refill_motor"] = ["motor_4"]
                steps_run, _ = start_run_with_log_scraping(
                    refill_barty, payload, steps_run, exp
                )
                colors_used[i] = 0
        curr_colors_used = [
            sum(payload["color_A_volumes"]),
            sum(payload["color_B_volumes"]),
            sum(payload["color_C_volumes"]),
            sum(payload["color_D_volumes"]),
        ]
        comb_list = [colors_used, curr_colors_used]
        colors_used = [sum(vols) for vols in zip(*comb_list)]
        print("Total vol of colors used so far:", colors_used)

        for i in colors_used:
            if i > 5000:
                print(i, ": Has used 5 mL of ink, Barty refill command")
                i = 0
                print("Updated colors_used:", colors_used)

        for i, color_vol in enumerate(colors_used):
            exp.events.log_decision(
                "Need Ink", (color_vol >= 5000)
            )  # 5 mL, change to whatever threshold.
            if color_vol >= 5000:
                if i == 0:
                    payload["refill_motor"] = ["motor_1"]
                elif i == 1:
                    payload["refill_motor"] = ["motor_2"]
                elif i == 2:
                    payload["refill_motor"] = ["motor_3"]
                elif i == 3:
                    payload["refill_motor"] = ["motor_4"]
                steps_run, _ = start_run_with_log_scraping(
                    refill_barty, payload, steps_run, exp
                )
                colors_used[i] = 0

        # Analyze image
        # output should be list [pop_size, 3]
        #img_path =
        #  Path(run_info["Take Picture"]["action_msg"].replace("/home/app", str(Path.home())))
        img_path = Path(exp.get_wf_result_file(run_info["hist"]["Take Picture"]["action_msg"], Path(run_info["run_dir"].replace("/home/app", str(Path.home()))) / "results" / "final_img.jpg", run_info["run_id"]))
        print(img_path)
        # if use_globus_compute:
        #     print("funcx started")
        #     exp.events.log_globus_compute("get_colors_from_file")
        #     fx = FuncXExecutor(endpoint_id=compute_local_ep)
        #     fxresult = fx.submit(get_colors_from_file, img_path)
        #     plate_colors_ratios = fxresult.result()[1]
        #     print("funcx finished")
        # else:
        exp.events.log_local_compute("get_colors_from_file")
        plate_colors = get_colors_from_file(img_path)[1]

        filename = "plate_" + str(plate_n) + ".jpg"
        # Copy the plate image into the experiment folder
        
        # Swap BGR to RGB
        plate_colors = {a: b[::-1] for a, b in plate_colors.items()}
        # Find the colors to be processed by the solver
        prev_colors = []
        wells_used = []
        for well in payload["destination_wells"]:
            color = plate_colors[well]
            wells_used.append(well)
            prev_colors.append(color)

        ## save those and the initial colors, etc
        plate_best_color_ind, plate_diffs = solver._find_best_color(
            prev_colors, target_color, cur_best_color
        )
        plate_best_color = prev_colors[plate_best_color_ind]
        plate_best_diff = solver._color_diff(plate_best_color, target_color)
        diffs.append(plate_diffs)
        # Find best colors
        if plate_best_diff < cur_best_diff:
            cur_best_diff = plate_best_diff
            cur_best_color = plate_best_color
            time_to_best = str(datetime.now() - start)

        ##update numbers
        current_iter += 1
        num_exps += pop_size
        ##Plot review
        runs = []
        report = {}
        if (Path(output_dir) / "exp_data.txt").is_file():
            with open(Path(output_dir) / "exp_data.txt", "r") as f:
                report = json.loads(f.read())
            runs = report["runs"]
            
        
        
        # Create new run log
        print("prev vols")
        print(np.multiply(previous_ratios, plate_max_volume).tolist())
        new_run = [
            {
                "run_number": current_iter,
                "run_label": str(run_info["run_id"]),
                "plate_N": plate_n,
                "tried_values": target_plate,
                "exp_volumes": np.multiply(previous_ratios, plate_max_volume).tolist(),
                "wells": list(wells_used),
                "results": list(map(lambda x: x.tolist(), prev_colors)),
                "differences": plate_diffs.tolist(),
                "best_on_plate": plate_best_color.tolist(),
                "pos_on_plate": plate_best_color_ind.tolist(),
                "plate_best_diff": plate_best_diff,
                "best_so_far": cur_best_color.tolist(),
            }
        ]
        # prepend new run to all run logs
        runs = new_run + runs
        # update report
        report.update(
            {
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
                "runs": runs,
            }
        )

        # Save run report
        with open(output_dir + "/exp_data.txt", "w") as f:
            report_js = json.dumps(report, indent=4)
            f.write(report_js)
        collect_files(exp, img_path, plate_n)

        create_visuals(
            target_plate,
            prev_colors,
            Path("./publish"),
            current_iter,
            target_color,
            cur_best_color,
            pop_size,
            diffs,
            solver_out_dim,
            solver,
        )
        # Save overall results
        print("publishing:")
        print(exp_folder)   
        publish_iter(Path("./publish").resolve(), exp_folder, exp)
        # publish_iter(exp_folder / "results", exp_folder, exp)
        # exp.events.log_loop_check(
        #     "Sufficient Wells in Experiment Budget", num_exps + pop_size <= exp_budget
        # )
        print(Path(str(run_info["run_dir"]).replace("/home/app", str(Path.home()))))
    exp.events.log_loop_end("Main Loop")
    # Trash plate after experiment
    shutil.copy2(
        Path(str(run_info["run_dir"]).replace("/home/app", str(Path.home()))) / "results" / "plate_only.jpg",
        (exp_folder / "results" / f"plate_{plate_n}.jpg"),
    )
    if not new_plate:
        steps_run, _ = start_run_with_log_scraping(
            final_protocol, payload, steps_run, exp
        )
        pass
    exp.events.end_experiment()
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
        help="Population size (num wells to fill per iter)",
    )
    parser.add_argument("--exp_budget", default=8, type=int, help="Experiment budget")
    parser.add_argument(
        "--target",
        "-t",
        default=str(np.random.randint(0, 255, 3).tolist()),
        help="Color Target",
    )
    parser.add_argument(
        "--solver", default="Evo", help="Bay = Bayes, Evo = Evolutionary, Agg = Aggro"
    )
    parser.add_argument("--plate_max_volume", default=275.0, type=float)
    return parser.parse_args()


if __name__ == "__main__":
    # test_run()

    # parser
    args = parse_args()

    # target color
    target_ratio = eval(args.target)

    exp_label = (
        "ColorPicker_"
        + str(target_ratio[0])
        + "_"
        + str(target_ratio[1])
        + "_"
        + str(target_ratio[2])
        + "_"
        + str(datetime.date(datetime.now()))
    )
    exp_path = "./experiment_results"
    if args.solver:
        if args.solver == "Bay":
            solver = BayesColorSolver(args.pop_size)
            solver_name = "Bayesian Solver"
        elif args.solver == "Evo":
            solver_name = "Evolutionary Solver"
            solver = EvolutionaryColorSolver(args.pop_size)
        elif args.solver == "Agg":
            solver = AggroColorSolver(args.pop_size)
            solver_name = "Aggressive Genetic Solver"
    else:
        solver = Solver()
        solver_name = "Solver"
    print(solver)
    print(target_ratio)
    print(exp_label)
    run_args = {
        "target_color": [120, 120, 120],
        "solver": solver,
        "solver_name": solver_name,
        "exp_budget": args.exp_budget,
        "pop_size": args.pop_size,
        "plate_max_volume": args.plate_max_volume,
        "exp_label": exp_label,
        "exp_path": exp_path,
    }
    run(**run_args)
