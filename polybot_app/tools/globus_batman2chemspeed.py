from gladier import GladierBaseClient, generate_flow_definition
import os

@generate_flow_definition()
class polybot_Flow(GladierBaseClient):
    globus_group = 'dda56f31-53d1-11ed-bd8b-0db7472df7d6'
    gladier_tools = [
       'gladier_tools.globus.transfer.Transfer'
    ]

def batman2chemspeed_flow( local_path, fname):
        remote_folder = "/C/Users/Operator/Desktop/Yukun/Workflow/Electrochromic Synthesis/Closeloop/" 
        globus_compute_local = 'd293531f-bec9-4f76-95d1-737ed32adc53'
        flow_input = {
            'input': {
                'transfer_source_endpoint_id': '6b517ee4-317d-11ee-87aa-4dfadf03ac7e', #Batman Transfer endpoint 
                'transfer_source_path': os.path.join(local_path, fname), # Tecan file location
                'transfer_destination_endpoint_id': '7ab34adc-2c98-11ee-b3eb-ad2493b708f4', #chemspeed endpoint 
                'transfer_destination_path': os.path.join(remote_folder, fname), 
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
        #print(flow_input['input']['proc_folder'])#.get('proc_folder'))
        flow = publishFlow.run_flow(flow_input=flow_input,label=label)
        # Track progress
        action_id = flow['action_id']
        publishFlow.progress(action_id)