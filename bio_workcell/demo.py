#!/usr/bin/env python3

import logging
from pathlib import Path
from argparse import ArgumentParser
from rpl_wei.wei_workcell_base import WEI
from .tools.publishv2 import publish_iter
from pathlib import Path

def main():
    wf_path = Path('/home/rpl/workspace/rpl_workcell/bio_workcell/workflows/demo.yaml')

    wei_client = WEI(wf_config = wf_path.resolve(), workcell_log_level=logging.ERROR, workflow_log_level=logging.ERROR)

    payload={
        'temp': 37.0, 
        'humidity': 95.0,
        'shaker_speed': 30,
        "stacker": 1, 
        "slot": 1,
        }

    run_info = wei_client.run_workflow(payload=payload)
    print(run_info)

    exp_path =  Path(exp_path)
    exp_label = Path(exp_label)
    exp_folder = exp_path / exp_label
    
    # if not (os.path.isdir(exp_path)):
    #     os.makedirs(exp_path)
    #     print("makingdir")
    # if not (os.path.isdir(exp_folder)):
    #     os.mkdir(exp_folder)
    # if not (os.path.isdir(exp_folder/"results")):
    #     os.mkdir(exp_folder/"results") 

    exp_folder = run_info['run_folder']
    print("publishing:")
    publish_iter(exp_folder/"results", exp_folder)

if __name__ == "__main__":
    main()
