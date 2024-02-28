from gladier import GladierBaseClient, generate_flow_definition
from tools.gather_data_v2 import GatherMetaData
from pathlib import Path


# @generate_flow_definition()
# class PublishRun(GladierBaseClient):
#     globus_group = "dda56f31-53d1-11ed-bd8b-0db7472df7d6"
#     gladier_tools = [GatherMetaData, "gladier_tools.publish.Publishv2"]
@generate_flow_definition(
    modifiers={
        "publishv2_gather_metadata": {"payload": "$.GatherMetadata.details.result[0]"}
    }
)
class PublishRun(GladierBaseClient):
    globus_group = "dda56f31-53d1-11ed-bd8b-0db7472df7d6"
    gladier_tools = [GatherMetaData, "gladier_tools.publish.Publishv2"]


def publish_iter(folder_path, dest_path, exp):
    print(str(folder_path))
    print(str(dest_path))
    flow_input = {
        "input": {
            "data_folder": str(folder_path),
            "compute_endpoint": "f38226d9-2d6b-4da8-b3c9-723a5467a848",
            "publishv2": {
                "dataset": str(folder_path),
                "index": "aefcecc6-e554-4f8c-a25b-147f23091944",
                "project": "reports",
                "compute_endpoint": "f38226d9-2d6b-4da8-b3c9-723a5467a848",
                "source_collection": "095889ca-cc29-11ee-b0ba-7de3e4236180",
                "source_collection_basepath": "/",
                "destination_collection": "bb8d048a-2cad-4029-a9c7-671ec5d1f84d",
                "metadata_file": str(folder_path / "metadata.json"),
                "ingest_enabled": True,
                "transfer_enabled": True,
                "destination": str("/portal/reports/" + str(dest_path)),
                "visible_to": ["public"],
            },
        }
    }

    # Create the Client
    publishFlow = PublishRun()
    label = "ColorPickerRPL"
    # Run the flow
    flow = publishFlow.run_flow(flow_input=flow_input, label=label)
    # exp.events.log_globus_flow(label, flow["action_id"])
    # Track progress
    # action_id = flow['action_id']
    # publishFlow.progress(action_id)


if __name__ == "__main__":
    publish_iter(
        Path(
            "~/experiments/ColorPicker_208_27_181_2023-10-10797/results"
        ),  # ~/.wei/experiments/Color_Picker_id_01HBF02F9MFM49JVH33S7J1A7Z/wei_runs/cp_wf_mixcolor_01HBF057JKEVEZ7RMEG7KJY0TF/results"),
        Path("Color_Picker_id_01HBF02F9MFM49JVH33S7J1A7Z"),
        [],
    )
