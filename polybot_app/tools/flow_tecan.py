from gladier import GladierBaseClient, generate_flow_definition
import os
import json
import globus_sdk

@generate_flow_definition()
class polybot_Flow(GladierBaseClient):
    gladier_tools = [
       'gladier_tools.globus.transfer.Transfer',
    ]
#108a843c-2c97-11ee-b441-ebe908329287
def tecan_flow( local_path, fname):
        remote_folder =  "/C/Users/cnmuser/Desktop/Polybot/tecan_code/uv_vis_data" 
        globus_compute_local = 'd293531f-bec9-4f76-95d1-737ed32adc53' 
        flow_input = {
            'input': { 
                'transfer_source_endpoint_id': 'a1f517e4-362d-11ee-b54c-e72de9e39f95', # Tecan endpoint UUI
                'transfer_source_path': os.path.join(remote_folder, fname), # Tecan file location
                'transfer_destination_endpoint_id': '6b517ee4-317d-11ee-87aa-4dfadf03ac7e', #Batman Transfer endpoint
                'transfer_destination_path': os.path.join(local_path, fname),
                'transfer_recursive': False,
                'compute_endpoint': globus_compute_local, #Batman compute endpoint
                'proc_folder': local_path,
                'file_name': fname,
                }
            }

        # Create the Client
        publishFlow = polybot_Flow()
        label = 'PolybotTestFlow'
        # Run the flow
        flow = publishFlow.run_flow(flow_input=flow_input,label=label)
        # Track progress
        action_id = flow['action_id']
        publishFlow.progress(action_id)
          
        
if __name__ == "__main__":
  local_path = "/home/rpl/workspace/polybot_workcell/polybot_app/demo_files/from_tecan"
  fname = "ECP_demo_batch_1.asc" #  "Output_1.csv" # 

  tecan_flow(local_path = local_path, fname = fname)
