#!/usr/bin/env python3

import logging
from pathlib import Path
from argparse import ArgumentParser
from ..tools.publish import publish_iter
from wei.wei_workcell_base import WEI


def main():
    wf_path = Path("/home/rpl/workspace/rpl_workcell/pcr_workcell/workflows/demo.yaml")

    wei_client = WEI(
        wf_config=wf_path.resolve(),
        workcell_log_level=logging.ERROR,
        workflow_log_level=logging.ERROR,
    )

    payload = {}
    run_info = wei_client.run_workflow(payload=payload)
    publish_iter(run_info["run_dir"], run_info["run_dir"])


if __name__ == "__main__":
    main()
