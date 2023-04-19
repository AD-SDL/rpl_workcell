#!/usr/bin/env python3

import logging
from pathlib import Path
from argparse import ArgumentParser
from rpl_wei.wei_workcell_base import WEI
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

    # exp_path =  Path(exp_path)
    # exp_label = Path(exp_label)
    # exp_folder = exp_path / exp_label
    
    # # if not (os.path.isdir(exp_path)):
    # #     os.makedirs(exp_path)
    # #     print("makingdir")
    # # if not (os.path.isdir(exp_folder)):
    # #     os.mkdir(exp_folder)
    # # if not (os.path.isdir(exp_folder/"results")):
    # #     os.mkdir(exp_folder/"results") 

    # exp_folder = run_info['run_folder']
    # print("publishing:")
    # publish_iter(exp_folder/"results", exp_folder)

if __name__ == "__main__":
    main()
