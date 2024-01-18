"""
Main application for the Squid Benchmark Experiment
"""

import argparse

from wei import ExperimentClient

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
        default=8080,
        help="Port of Workcell",
    )
    parser.add_argument(
        "--experiment_id", type=str, help="ID of the experiment", default=""
    )
    args = parser.parse_args()
    experiment = ExperimentClient(
        server_addr=args["rpl_wc_server"],
        server_port=args["rpl_wc_port"],
        experiment_name="Squid Benchmark Experiment",
        experiment_id=args["experiment_id"],
    )
    print(experiment.experiment_id)


if __name__ == "__main__":
    main()
