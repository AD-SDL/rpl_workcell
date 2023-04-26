from gladier import GladierBaseClient, generate_flow_definition, GladierBaseTool
from tools.gather_data import GatherMetaData
@generate_flow_definition(modifiers={'publish_gather_metadata' : {'payload': '$.GatherMetadata.details.result[0]'}})
class PublishRun(GladierBaseClient):
    globus_group = 'dda56f31-53d1-11ed-bd8b-0db7472df7d6'
    gladier_tools = [
        GatherMetaData,
        'gladier_tools.publish.Publish'
    ]

def publish_iter(folder_path, dest_path):
        flow_input = {
            'input': {
                'make_input': str(folder_path.expanduser()),
                'funcx_endpoint_compute':'95038e17-339b-4462-9c9f-a8473809af25',
                'funcx_endpoint_non_compute':'95038e17-339b-4462-9c9f-a8473809af25',

                'pilot': {
                    'dataset': str(folder_path.expanduser()),
                    'index': '4e2884b0-e585-4913-8a33-4be155ebb06c',
                    'project': 'bio',
                    'source_globus_endpoint': '95038e17-339b-4462-9c9f-a8473809af25',
                    'source_collection_basepath': '/',
                    'metadata': {},
                    'destination':str(dest_path)
                   }
                }
            }

        # Create the Client
        publishFlow = PublishRun()
        label = 'ColorPickerTestPublish'
        # Run the flow
        flow = publishFlow.run_flow(flow_input=flow_input,label=label)
        # Track progress
        # action_id = flow['action_id']
        # publishFlow.progress(action_id)
     
        
