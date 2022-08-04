from pathlib import Path
from argparse import ArgumentParser

# TODO: Currently does not work due to relative imports,
# when this becomes a package it will be good
from ..maestro.wc_client_base import WC_Client


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
