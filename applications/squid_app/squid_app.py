"""
Main application for the Squid Benchmark Experiment
"""

import argparse
from pathlib import Path

from wei import ExperimentClient

# Create Inks (N9)
# Supply RPL Workcell (Inks from N9, plates from Storage?)
# Move Plates to OT2
# Run Color Picker:
## TODO
# Do Analysis
# Repeat until failure


def parse_args() -> argparse.Namespace:
    """Parses command line arguments for the Squid Benchmark Experiment"""
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
        "--experiment_file",
        type=Path,
        help="Path to the experiment design file",
        default=Path(__file__).parent.absolute() / "squid_app.design.yaml",
    )
    parser.add_argument(
        "--workflow_dir",
        type=Path,
        help="Path to the workflow directory",
        default=Path(__file__).parent.absolute() / "workflows",
    )
    parser.add_argument(
        "--protocol_dir",
        type=Path,
        help="Path to the protocol directory",
        default=Path(__file__).parent.absolute() / "protocols",
    )
    return parser.parse_args()


def main():
    """
    Entrypoint for the Squid Benchmark Experiment
    """
    args = parse_args()
    experiment = ExperimentClient(
        server_host=args.rpl_wc_server,
        server_port=args.rpl_wc_port,
        experiment_design=args.experiment_file,
        experiment_id=args.experiment_id,
    )

    experiment.start_run(
        workflow_file=args.wf_dir / "reset_colors.wf.yaml",
        payload={},
        blocking=False,
        simulate=False,
    )
    experiment.start_run(
        workflow_file=args.wf_dir / "new_plate.wf.yaml",
        payload={},
        blocking=True,
        simulate=False,
    )
    experiment.start_run(
        workflow_file=args.wf_dir / "mix_colors.wf.yaml",
        payload={
            "use_existing_resources": False,
            "color_A_volumes": 10.0,
            "color_B_volumes": 10.0,
            "color_C_volumes": 10.0,
            "color_D_volumes": 10.0,
            "destination_wells": ["A1"],
            "config_path": str(args.protocol_dir / "combined_protocol.yaml"),
        },
        blocking=True,
        simulate=False,
    )
    experiment.start_run(
        workflow_file=args.wf_dir / "cleanup.wf.yaml",
        payload={},
        blocking=False,
        simulate=False,
    )
    experiment.log_experiment_end()


if __name__ == "__main__":
    main()
