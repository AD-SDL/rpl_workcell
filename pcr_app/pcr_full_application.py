#!/usr/bin/env python3

from pathlib import Path
from wei import Experiment
from time import sleep


def main():
    # Connect to the experiment
    exp = Experiment("127.0.0.1", "8000", "Full PCR Test")

    # run the workflow
    wf_path = Path(__file__).parent / "workflows/pcr_demo_wf.yaml"
    flow_info = exp.start_run(wf_path.resolve())
    print(flow_info)

    #while True:
    #    flow_state = exp.query_job(flow_info["job_id"])
    #    print(flow_state)
    #    if flow_state["status"] == "failed" or flow_state["status"] == "success":
    #        break
    #    sleep(1)


if __name__ == "__main__":
    main()
