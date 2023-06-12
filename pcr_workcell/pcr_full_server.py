#!/usr/bin/env python3

from pathlib import Path
# from tools.publish import publish_iter
from rpl_wei import Experiment

def main():
    wf_path = Path('/home/rpl/workspace/rpl_workcell/pcr_workcell/workflows/qtest.yaml')
    v = Experiment('127.0.0.1', '8000')
    t = v.run_job(wf_path.resolve())

    print(t)
    t = v.query_job(t['job_id'])
    print(t)
    payload={}
    #run_info = wei_client.run_workflow(payload=payload)
    # publish_iter(run_info["run_dir"], run_info["run_dir"])
if __name__ == "__main__":
    main()
