import logging
from pathlib import Path
from argparse import ArgumentParser

from rpl_wei.wei_client_base import WEI
from rpl_wei.data_classes import Module, Step

import rclpy
from wei_executor.weiExecutorNode import weiExecNode

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


def print_callback(step, **kwargs):
    print("\nFrom callback")
    print(step)


def main(args):
    wei = WEI(
        args.workcell,
        args.workflow,
        workcell_log_level=logging.DEBUG,
        workflow_log_level=logging.DEBUG,
    )

    # get the workflow id (currently defaulting to first one available)
    wf_id = list(wei.get_workflows().keys())[0]

    wei.run_workflow(wf_id, [wei_service_callback])


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-wc", "--workcell", help="Path to workcell file", type=Path)
    parser.add_argument(
        "-wf", "--workflow", help="Path to workflow file", type=Path, required=True
    )
    parser.add_argument(
        "-v", "--verbose", help="Extended printing options", action="store_true"
    )

    args = parser.parse_args()
    main(args)
