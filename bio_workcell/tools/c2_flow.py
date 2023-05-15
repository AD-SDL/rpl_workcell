from gladier import GladierBaseClient, generate_flow_definition, GladierBaseTool

from c2_read_hidex import C2_read_hidex
from c2_check_contam import C2_check_contam
from c2_blank_adjust import C2_blank_adjust
from c2_gen_graphs import C2_gen_graphs
from gather_data import GatherMetaData
from pathlib import Path
@generate_flow_definition(modifiers={'publishv2_gather_metadata' : {'payload': '$.GatherMetadata.details.result[0]'}})
class C2Flow(GladierBaseClient):
    globus_group = 'dda56f31-53d1-11ed-bd8b-0db7472df7d6'
    gladier_tools = [
    #    'gladier_tools.globus.Transfer',
    #     C2_read_hidex,
    #     C2_check_contam,
    #     C2_blank_adjust,
    #     C2_gen_graphs,
        GatherMetaData,
       'gladier_tools.publish.Publishv2'
    ]

def c2_flow(exp_name, plate_n,time, local_path, fname):
        remote_folder = '/home/rpl/test/'
        local_gcp = 'e69053b2-f02f-11ed-ba44-09d6a6f08166'
        local_funcx = 'b246dc22-4cc6-406f-bd44-3748b775f3bb'
        flow_input = {
            'input': {
                'transfer_source_endpoint_id':'2f3968d6-d8a4-11ed-971f-e54704575ba0', #hudson ep
                'transfer_source_path': local_path + fname,
                'transfer_destination_endpoint_id': local_gcp, #biopotts ep
                'transfer_destination_path': remote_folder  + fname,
                'transfer_recursive': False,
                'funcx_endpoint_compute': local_funcx, #biopotts funcx
                'funcx_endpoint_non_compute': local_funcx, #biopotts funcx
                'exp_name':exp_name,
                'plate_n':plate_n,
                'proc_folder': remote_folder,
                'file_name': fname,
                'csv_file': fname.split('.')[0] +".csv",
                'csv_file_corr': fname +"_corr.csv",
                'time':time,
                'publishv2': {
                    'dataset': remote_folder,
                    'index': '4e2884b0-e585-4913-8a33-4be155ebb06c',
                    'project': 'bio',
                    'source_collection': local_gcp,
                    'source_collection_basepath': '/',
                    'destination_collection': 'bb8d048a-2cad-4029-a9c7-671ec5d1f84d',
                    'metadata': {},
                    'ingest_enabled': True,
                    'transfer_enabled':True,
                    'destination':str("/portal/bio"),
                    'visible_to' : ['public']
                   }
                }
            }

        # Create the Client
        publishFlow = C2Flow()
        label = 'BioTestFlow'
        # Run the flow
        flow = publishFlow.run_flow(flow_input=flow_input,label=label)
        # Track progress
        action_id = flow['action_id']
        publishFlow.progress(action_id)
     
        
if __name__ == "__main__":
  local_path = "/C/labautomation/data_wei/proc/"
  fname = "Campaign2_wei_(10)_20230501_143510.xlsx"

  c2_flow("test_exp", 1, "time", local_path, fname)
