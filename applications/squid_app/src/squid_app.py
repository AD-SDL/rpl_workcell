"""
Main application for the Squid Benchmark Experiment
"""

import argparse
from pathlib import Path

# from wei import ExperimentClient
import wei

# Create Inks (N9)
# Supply RPL Workcell (Inks from N9, plates from Storage?)
# Move Plates to OT2
# Run Color Picker:
## TODO
# Do Analysis
# Repeat until failure


def main():
    """
    Entrypoint for the Squid Benchmark Experiment
    """
    parser = argparse.ArgumentParser(
        description="Squid Benchmark Experiment Application",
    )
    parser.add_argument(
        "--rpl_wc_server",
        type=str,
        default="localhost",
        help="Address of Workcell",
    )
    parser.add_argument(
        "--rpl_wc_port",
        type=int,
        default=8000,
        help="Port of Workcell",
    )
    parser.add_argument(
        "--experiment_id", type=str, help="ID of the experiment", default=""
    )
    args = parser.parse_args()
    experiment = wei.ExperimentClient(
        server_addr=args.rpl_wc_server,
        server_port=args.rpl_wc_port,
        experiment_name="Squid Benchmark Experiment",
        # experiment_id=args.experiment_id,
    )

    wf_dir = Path(__file__).parent.parent.absolute() / "workflows"
    protocol_dir = Path("/protocols")

    # experiment.start_run(
    #     workflow_file=wf_dir / "cp_wf_reset_colors.yaml",
    #     payload={},
    #     blocking=True,
    #     simulate=False,
    # )
    # experiment.start_run(
    #     workflow_file=wf_dir / "cp_wf_newplate.yaml",
    #     payload={},
    #     blocking=True,
    #     simulate=False,
    # )
    experiment.start_run(
        workflow_file=wf_dir / "cp_wf_mixcolor.yaml",
        payload={
            "use_existing_resources": False,
            "color_A_volumes": 10.0,
            "color_B_volumes": 10.0,
            "color_C_volumes": 10.0,
            "color_D_volumes": 10.0,
            "destination_wells": ["A1"],
            "config_path": str(protocol_dir / "combined_protocol.yaml"),
        },
        blocking=True,
        simulate=False,
    )


if __name__ == "__main__":
    main()
