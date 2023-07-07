#!/usr/bin/env python3

from pathlib import Path
from rpl_wei import Experiment
from time import sleep

def main():

    #Connect to the experiment
    exp = Experiment('127.0.0.1', '8000', 'Full PCR Test')
    
    exp.register_exp()
    #run the workflow
    wf_path = Path('/home/rpl/workspace/rpl_workcell/pcr_workcell/workflows/demo.yaml')
    flow_info = exp.run_job(wf_path.resolve())
    print(flow_info)
    
    while True:
        flow_state = exp.query_job(flow_info['job_id'])
        print(flow_state)
        sleep(1)

if __name__ == "__main__":
    main()
