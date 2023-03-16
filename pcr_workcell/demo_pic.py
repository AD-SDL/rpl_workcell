#!/usr/bin/env python3

import logging
from pathlib import Path
from argparse import ArgumentParser
from publish import publish_iter
from rpl_wei.wei_workcell_base import WEI
from rpl_wei.data_classes import Module, Step
import shutil, os
import json
from datetime import datetime
import yaml
from threading import Thread

def get_log_info(run_path, steps_run):
        lineiter=0
        print("starting")
        while not(os.path.isfile(run_path / "runLogger.log")):
           
            pass
        with open(run_path/ "runLogger.log") as log:
            lines = log.read().splitlines()
            print(lines)
            log.close()
            for i, step in enumerate(steps_run):
                starttime = []
                while starttime == [] and lineiter < len(lines):
                    line = lines[lineiter]
                    columns = [col.strip() for col in line.split(':') if col]
                    if columns[-2] == 'Started running step with name' and columns[-1] == step["name"]:
                        starttime = datetime.strptime(line[0:23], '%Y-%m-%d %H:%M:%S,%f')
                    lineiter += 1
                endtime = []
                while endtime == [] and lineiter < len(lines):
                    line = lines[lineiter]
                    columns = [col.strip() for col in line.split(':') if col]
                    if columns[-2] == 'Finished running step with name' and columns[-1] == step["name"]:
                        endtime = datetime.strptime(line[0:23], '%Y-%m-%d %H:%M:%S,%f')
                    lineiter += 1
                steps_run[i]["start_time"] = str(starttime)
                steps_run[i]["end_time"] = str(endtime)
                steps_run[i]["duration"] = str(endtime-starttime)
        return steps_run, lineiter
def get_wf_info(ptcl):
    steps_run = []
    with open(ptcl, 'r') as stream:
            wf = yaml.safe_load(stream)
            for test in wf["flowdef"]:
                steps_run.append(test)
    return steps_run
def wei_run_flow(wf_file_path, payload):
    wei_client = WEI(wf_file_path)
    run_info = wei_client.run_workflow(payload=payload)
    print(run_info)
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

def main():
    wf_path = Path('/home/rpl/workspace/rpl_workcell/pcr_workcell/workflows/demo_pic.yaml')

    wei_client = WEI(wf_config = wf_path.resolve(), workcell_log_level= logging.ERROR, workflow_log_level=logging.ERROR)

    # wei_client = WEI(
    #     wf_path.resolve(),
    #     workcell_log_level=logging.DEBUG,
    #     workflow_log_level=logging.DEBUG,
    # )

    # wf_id = list(wei_client.get_workflows().keys())[0]

    payload={}
    run_info = wei_client.run_workflow(payload=payload)
    iter_thread=ThreadWithReturnValue(target=wei_run_flow,kwargs={'wf_file_path':wf_path,'payload':payload})
    iter_thread.run()
    run_info = iter_thread._return
    os.mkdir("/home/rpl/experiments/PCRTest" / Path(run_info["run_dir"].name))
    steps_run = get_wf_info(wf_path)
    steps_run,b = get_log_info(run_info["run_dir"], steps_run)
    with open("/home/rpl/experiments/PCRTest" / Path(run_info["run_dir"].name) /"wf_steps.txt", "w") as f:
           report_js = json.dumps(steps_run, indent=4)
           f.write(report_js) 
    shutil.copy2(run_info["run_dir"]/"results"/"final_image.jpg",  "/home/rpl/experiments/PCRTest" / Path(run_info["run_dir"].name) / "final_image.jpg")
    publish_iter( "/home/rpl/experiments/PCRTest" / Path(run_info["run_dir"].name), "/home/rpl/experiments/PCRTest")
if __name__ == "__main__":
    main()
 