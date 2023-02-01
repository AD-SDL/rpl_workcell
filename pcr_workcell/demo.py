#!/usr/bin/env python3

import logging
from pathlib import Path
from argparse import ArgumentParser

from rpl_wei.wei_workcell_base import WEI
from rpl_wei.data_classes import Module, Step

def main():
    wf_path = Path('/home/rpl/workspace/rpl_workcell/pcr_workcell/workflows/demo.yaml')

    wei_client = WEI(
        wf_path.resolve(),
        workcell_log_level=logging.DEBUG,
        workflow_log_level=logging.DEBUG,
    )

    wf_id = list(wei_client.get_workflows().keys())[0]

    payload={}
    wei_client.run_workflow(wf_id, payload=payload)

if __name__ == "__main__":
    main()
