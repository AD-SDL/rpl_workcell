from gladier import GladierBaseClient, generate_flow_definition, GladierBaseTool

from .c2_read_hidex import C2_read_hidex
from .c2_check_contam import C2_check_contam
from .c2_blank_adjust import C2_blank_adjust
from .c2_gen_graphs import C2_gen_graphs
from .gather_data import GatherMetaData

@generate_flow_definition(modifiers={'publish_gather_metadata' : {'payload': '$.GatherMetadata.details.result[0]'}})
class C2Flow(GladierBaseClient):
    globus_group = 'dda56f31-53d1-11ed-bd8b-0db7472df7d6'
    gladier_tools = [
        'gladier_tools.transfer.Transfer',
        C2_read_hidex,
        C2_check_contam,
        C2_blank_adjust,
        C2_gen_graphs,
        GatherMetaData,
        'gladier_tools.publish.Publish'
    ]

def c2_flow(exp_name,plate_n,time):
        flow_input = {
            'input': {
                'source_globus_endpoint':'c819ce5c-d3e4-11ed-a9ce-63ca5f6c6821', #hudson ep
                'destination_globus_endpoint':'f9726362-96a7-11ed-b310-55098fa75e99', #ripchip ep
                'funcx_endpoint_compute':'95038e17-339b-4462-9c9f-a8473809af25', #ripchip funcx
                'funcx_endpoint_non_compute':'95038e17-339b-4462-9c9f-a8473809af25', #ripchip funcx
                'exp_name':exp_name,
                'plate_n':plate_n,
                'time':time,
                # 'pilot': {
                #     'dataset': str(folder_path.expanduser()),
                #     'index': '4e2884b0-e585-4913-8a33-4be155ebb06c',
                #     'project': 'bio',
                #     'source_globus_endpoint': '95038e17-339b-4462-9c9f-a8473809af25',
                #     'source_collection_basepath': '/',
                #     'metadata': {},
                #     'destination':str(dest_path)
                #    }
                }
            }

        # Create the Client
        publishFlow = C2Flow()
        label = 'BioTestFlow'
        # Run the flow
        flow = publishFlow.run_flow(flow_input=flow_input,label=label)
        # Track progress
        # action_id = flow['action_id']
        # publishFlow.progress(action_id)
     
        
