
import os



class wc_client():

  def __init__(self, wc_config_file):
    self.wc_config_file = wc_config_file
    self.modules = self.get_modules()
    self.state = None

    self.check_modules()
    self.check_action()

  def check_modules():
    pass

  def check_action():
    pass

  def run_flow(self, flow_def_file):
    with f as Open(flow_def_file):
        flow_def = yaml.safeload(f)

    for step in flow_def:
        print(step)
 