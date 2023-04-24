
from rpl_wei import WEI
from threadReturn import ThreadWithReturnValue
from log_info import get_log_info

def wei_run_flow(wf_file_path, payload):
    wei_client = WEI(wf_file_path)
    run_info = wei_client.run_workflow(payload=payload)
    return run_info

def run_flow(protocol, payload, steps_run):
                iter_thread=ThreadWithReturnValue(target=wei_run_flow,kwargs={'wf_file_path':protocol,'payload':payload})
                iter_thread.run()
                run_info = iter_thread._return
                run_dir = run_info["run_dir"]
                t_steps_run = get_log_info(run_dir, protocol)
                steps_run.append(t_steps_run)
                return steps_run, run_info