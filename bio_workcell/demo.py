#!/usr/bin/env python3

import logging
from pathlib import Path
from rpl_wei.wei_workcell_base import WEI
from tools.c2_flow import c2_flow

def main():
    wf_path = Path('/home/rpl/workspace/rpl_workcell/pcr_workcell/workflows/hidex_wf.yaml')
    print(wf_path.resolve())
    wei_client = WEI(wf_config = wf_path.resolve())#, workcell_log_level= logging.ERROR, workflow_log_level=logging.ERROR)
    payload={}
    run_info = wei_client.run_workflow(payload=payload)
    test = run_info["hist"]["run_assay"]["step_response"]
    test = test.replace('\\', '/')
    test = test.replace("C:/", "/C/")
    flow_title = Path(test) #Path(run_info["hist"]["run_assay"]["step_response"])
    fname = flow_title.name
    flow_title = flow_title.parents[0]
    c2_flow("hidex_test", "now", "hi", flow_title, fname)

if __name__ == "__main__":
    main()
