from typing import List, Dict, Any, Tuple
from tools.run_flow import run_flow
from pathlib import Path
import numpy as np
from tools.color_utils import convert_volumes_to_payload
from tools.plate_color_analysis import get_colors_from_file
import cv2
from matplotlib import pyplot as plt

def calibrate(target_color: List[int], 
              curr_wells_used: List[str], 
              loop_protocol: str, 
              exp_folder: Path, 
              plate_max_volume: float,
              steps_run: List[Dict[str, Any]],
              pop_size: int,
              experiment: Any,
              )-> Tuple[List[List[int]], List[int], List[str], List[Dict[str, Any]]]:
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
    steps_run: """
    plate_volumes = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1], np.array(target_color)/sum(target_color)])*plate_max_volume
    payload, curr_wells_used = convert_volumes_to_payload(plate_volumes, curr_wells_used)
    payload['use_existing_resources'] = False 
    print(payload)
    steps_run, run_info = run_flow(loop_protocol, payload, steps_run, experiment) 
    fname = "final_image.jpg" #image"+str(ot2_iter) +".jpg"
    img_path = run_info["run_dir"]/ "results" / fname
    plate_colors_ratios = get_colors_from_file(img_path)[1]
    plate_colors_ratios = {a:b[::-1] for a,b in plate_colors_ratios.items()}  
    current_plate = []
    wells_used = []
    for well in payload["destination_wells"]:
        color = plate_colors_ratios[well]
        wells_used.append(well)
        current_plate.append(color)
    target_color = current_plate[3]
    target_color = target_color.tolist()
    colors = current_plate[0:3]
    t = np.asarray(colors)
    colors = t.tolist()
    color_image = cv2.resize( np.asarray([colors]).astype(np.uint8), [pop_size*50, 50], interpolation = cv2.INTER_NEAREST)
    plt.imsave(exp_folder/"results"/"mixed_colors.png", color_image/255)
    return colors, target_color, curr_wells_used, steps_run