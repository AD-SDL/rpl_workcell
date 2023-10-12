# from wei import WEI
from tools.threadReturn import ThreadWithReturnValue
from tools.log_info import get_log_info
from wei.exp_app import Experiment
from pathlib import Path
import time



def start_run_with_log_scraping(protocol, payload, steps_run, experiment: Experiment):
    """Runs a WEI flow and
    updates the steps that have been run in the experiment
    @Inputs:
        protocol: Filename for which WEI flow to run
        payload: Payload to run that flow with
        steps_run: The steps that have been run so far on this iteration of the experiment
    @Outputs:
        steps run: The list of steps run in this iteration of the experiment including the new ones added
        by the workflow run by this function
        run_info: The WEI output from running the flow"""
   
    result  = experiment.start_run(protocol.resolve(), payload=payload, simulate=False, blocking = True)
    run_info = result["hist"]
    run_info["run_dir"] = Path(run_info["run_dir"])
    run_dir = Path(run_info["run_dir"])
    t_steps_run = get_log_info(run_dir, protocol)
    steps_run.append(t_steps_run)
    return steps_run, run_info
