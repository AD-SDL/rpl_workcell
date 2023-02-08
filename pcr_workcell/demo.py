#!/usr/bin/env python3

import logging
from pathlib import Path
from argparse import ArgumentParser

from rpl_wei.wei_workcell_base import WEI
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

def main():
    wf_path = Path('/home/rpl/rpl_ws/src/rpl_workcell/pcr_workcell/workflows/demo.yaml')

    wei_client = WEI(
        wf_path.resolve(),
        workcell_log_level=logging.DEBUG,
        workflow_log_level=logging.DEBUG,
    )

    wf_id = list(wei_client.get_workflows().keys())[0]

    payload={}
    wei_client.run_workflow(wf_id, payload=payload, callbacks=[wei_service_callback])

if __name__ == "__main__":
    main()
