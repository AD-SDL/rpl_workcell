


from rpl_maestro import WC_Client

rpl_workcell_path = ''

client = WC_Client(rpl_workcell_path)

client.check_modules()

client.check_actions()



