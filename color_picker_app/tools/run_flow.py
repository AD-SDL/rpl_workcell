
#from rpl_wei import WEI
from tools.threadReturn import ThreadWithReturnValue
from tools.log_info import get_log_info
from rpl_wei.exp_app import Experiment
from pathlib import Path
import time
def wei_run_flow(wf_file_path, payload):
    wei_client = WEI(wf_file_path)
    run_info = wei_client.run_workflow(payload=payload)
    return run_info

def run_flow(protocol, payload, steps_run, experiment):
    '''Runs a WEI flow and 
    updates the steps that have been run in the experiment
    @Inputs: 
        protocol: Filename for which WEI flow to run
        payload: Payload to run that flow with
        steps_run: The steps that have been run so far on this iteration of the experiment
    @Outputs:
        steps run: The list of steps run in this iteration of the experiment including the new ones added
        by the workflow run by this function
        run_info: The WEI output from running the flow'''
    #iter_thread=ThreadWithReturnValue(target=wei_run_flow,kwargs={'wf_file_path':protocol,'payload':payload})
    #iter_thread.run()
    response = experiment.run_job(protocol.resolve(), payload=payload, simulate=False)
    job_status = experiment.query_job(response["job_id"])
    print(job_status)
    while(job_status["status"] != "finished" and job_status["status"] != "failure"):
        job_status = experiment.query_job(response["job_id"])
        time.sleep(3)
    #print(experiment.get_job_log(response["job_id"]))
    run_info = job_status["result"]
    run_info["run_dir"] = Path(run_info["run_dir"])
    print(run_info)
    run_dir = Path(run_info["run_dir"])
    t_steps_run = get_log_info(run_dir, protocol)
    steps_run.append(t_steps_run)
    return steps_run, run_info