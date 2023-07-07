#!/usr/bin/env python3

from pathlib import Path
from rpl_wei import Experiment

def main():
    wf_path = Path('/home/rpl/workspace/rpl_workcell/pcr_workcell/workflows/test_rest.yaml')
    exp = Experiment('127.0.0.1', '8000', 'Full PCR Test')
    flow_info = exp.run_job(wf_path.resolve())

    print(flow_info)
    flow_state = exp.query_job(flow_info['job_id'])
    print(flow_state)

if __name__ == "__main__":
    main()
