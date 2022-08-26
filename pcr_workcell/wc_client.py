from pathlib import Path
from argparse import ArgumentParser

from rpl_wei.wei_client_base import WEI
import logging


def main(args):
    wei = WEI(
        args.workcell,
        args.workflow,
        workcell_log_level=logging.DEBUG,
        workflow_log_level=logging.DEBUG,
    )

    # get the workflow id (currently defaulting to first one available)
    wf_id = list(wei.get_workflows().keys())[0]

    wei.run_workflow(wf_id)


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-wc", "--workcell", help="Path to workcell file", type=Path)
    parser.add_argument("-wf", "--workflow", help="Path to workflow file", type=Path, required=True)
    parser.add_argument("-v", "--verbose", help="Extended printing options", action="store_true")

    args = parser.parse_args()
    main(args)
