from pathlib import Path
from argparse import ArgumentParser

from devtools import debug

from data_classes import Workflow, WorkCell


class WC_Client:
    def __init__(self, wc_config_file):
        self.state = None

        self.workflow = Workflow.from_yaml(wc_config_file)
        self.modules = self.workflow.modules
        self.actions = self.workflow.actions
        self.workcell = WorkCell.from_yaml(self.workflow.workcell)

    def check_modules(self):
        for module in self.modules:
            print(f"Checking module: {module}")

    def check_actions(self):
        for step in self.actions:
            print(f"Checking step: {step}")

    def run_flow(self):

        for step in self.actions:
            print(step)

    def print_flow(self):
        debug(self.workflow)

    def print_workcell(self):
        debug(self.workcell)


def main(args):
    wc = WC_Client(args.workflow)
    if args.verbose:
        wc.print_flow()
        wc.print_workcell()
    wc.check_modules()
    wc.check_actions()


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-wf", "--workflow", help="Path to workflow file", type=Path, required=True)
    parser.add_argument("-v", "--verbose", help="Extended printing options", action="store_true")

    args = parser.parse_args()
    main(args)
