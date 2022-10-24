#!/usr/bin/env python3

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
    ) #Make into globus native client, so it gets globusID of whoever is running it.



    # get the workflow id (currently defaulting to first one available)
    wf_id = list(wei_client.get_workflows().keys())[0]

    run_class = wei_client.run_workflow(wf_id, [wei_service_callback])
    #, publish=True)
    #run_class.id
    #run_class.wc_file
    #run_class.wf_name
    # ...
    #run_class.run_folder
    #search_index_uuid = get_from_wc_file
    #run_class.publish(self, search_index_uuid)
    #this is basically pointing at the folder and running globus-pilot with the correct search-index
    

if __name__ == "__main__":
    args = parse_args()
    main(args)
