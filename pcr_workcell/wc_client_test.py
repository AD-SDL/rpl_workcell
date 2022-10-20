#!/usr/bin/env python3

from pathlib import Path
from argparse import ArgumentParser
from pprint import pprint as print

from rpl_wei.wei_client_base import WEI
from rpl_wei.data_classes import Module, Step
import logging

def wei_service_callback(step: Step , **kwargs):

    module: Module = kwargs['step_module'] # This is a bit weird. I wonder how to pass it all in one single variable

    msg = {"node": module.config['ros_node'], "action_handle": step.command, "action_vars": step.args}
    print("\nWEI ROS2 Client callback:")
    print(msg)
    print("\n")


def print_callback(step: Step , **kwargs):
    module: Module = kwargs['step_module'] # This is a bit weird. I wonder how to pass it all in one single variable
    print("Print callback:")
    print("Step Info:")
    print(step)
    print("Module Info:")
    print(module)

    msg = {"node": module.config['ros_node'], "action_handle": step.command, "action_vars": step.args}
    print("\nWEI ROS2 Client callback:")
    print(msg)
    print("\n")


def parse_args():
    parser = ArgumentParser()
    parser.add_argument("-wf", "--workflow", help="Path to workflow file", type=Path, required=True)

    return parser.parse_args()

def main(args):
    wf_file_path = args.workflow.resolve()
    
    wei_client = WEI(
        wf_file_path,
        workcell_log_level=logging.DEBUG,
        workflow_log_level=logging.DEBUG,
    )

    # get the workflow id (currently defaulting to first one available)
    wf_id = list(wei_client.get_workflows().keys())[0]
    wei_client.run_workflow(wf_id, [print_callback])


if __name__ == "__main__":
    args = parse_args()
    main(args)
