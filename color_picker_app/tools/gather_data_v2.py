from gladier import GladierBaseTool, generate_flow_definition


def gather_metadata(**data):
    from pathlib import Path
    import json

    GENERAL_METADATA = {
        "creators": [{"creatorName": "RPL Team"}],
        "publicationYear": "2023",
        "publisher": "Argonne National Lab",
        "resourceType": {"resourceType": "Dataset", "resourceTypeGeneral": "Dataset"},
        "subjects": [{"subject": "SDL"}],
        "exp_type": "color_picker",
    }

    input_path = Path(data["data_folder"]).expanduser()
    with open(input_path / "exp_data.txt") as f:
        datal = json.loads(f.read())

    datal.update(GENERAL_METADATA)
    final_data = data["publishv2"]
    final_data["metadata"] = datal

    with open(data["publishv2"]["metadata_file"], "w") as f:
        json.dump(final_data, f)
    return final_data


@generate_flow_definition
class GatherMetaData(GladierBaseTool):
    compute_functions = [gather_metadata]
    required_input = ["data_folder", "compute_endpoint", "publishv2"]
