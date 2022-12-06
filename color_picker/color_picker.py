#!/usr/bin/env python3

import logging
from pathlib import Path
from argparse import ArgumentParser

from rpl_wei.wei_workcell_base import WEI
from rpl_wei.data_classes import Module, Step
from evolutionary_solver import EvolutionaryColors

import rclpy
from wei_executor.weiExecutorNode import weiExecNode


# Can this be an init to an executor callback?
rclpy.init()
wei_execution_node = weiExecNode()


def wei_service_callback(step: Step, **kwargs):

    module: Module = kwargs["step_module"]

    msg = {
        "node": module.config["ros_node"],
        "action_handle": step.command,
        "action_vars": step.args,
    }
    print("\n Callback message:")
    print(msg)
    print()

    wei_execution_node.send_wei_command(
        msg["node"], msg["action_handle"], msg["action_vars"]
    )


def parse_args():
    parser = ArgumentParser()
    parser.add_argument(
        "-wf", "--workflow", help="Path to workflow file", type=Path, required=True
    )
    return parser.parse_args()


def run(wei_client: WEI, solver: EvolutionaryColors) -> None:

    ...


def main(args):
    wf_file_path = args.workflow.resolve()

    wei_client = WEI(
        wf_file_path,
        workcell_log_level=logging.DEBUG,
        workflow_log_level=logging.DEBUG,
    )

    target_ratio = [237, 36, 36]
    mixing_colors = [[255, 0, 0], [0, 255, 0], [0, 0, 255]]
    solver = EvolutionaryColors(target=target_ratio, mixing_colors=mixing_colors)

    run(wei_client, solver)


if __name__ == "__main__":
    args = parse_args()
    main(args)
