"""
Main application for testing instruments at RPL
"""

from pathlib import Path

from wei import ExperimentClient

def main():

    experiment = ExperimentClient(
        server_host="localhost",
        server_port="8000",
        experiment_name="RPL Module test",
        email_addresses=["dozgulbas@anl.gov"]
    )

    wf_dir = Path(__file__).parent.absolute() / "workflows"

    experiment.start_run(
        workflow_file=wf_dir / "all_instruments.yaml",
        # payload={},
        # blocking=True,
        # simulate=False,
    )


if __name__ == "__main__":
    main()
