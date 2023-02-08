from opentrons import protocol_api


metadata = {
    "protocolName": "Color Mixing all",
    "author": "Kyle khippe@anl.gov",
    "description": "Mixing red colors",
    "apiLevel": "2.12"
}

def run(protocol: protocol_api.ProtocolContext):

    deck = {}
    pipettes = {}

    ################
    # load labware #
    ################
    deck["2"] = protocol.load_labware("corning_96_wellplate_360ul_flat", "2")
    deck["7"] = protocol.load_labware("opentrons_6_tuberack_nest_50ml_conical", "7")
    deck["10"] = protocol.load_labware("opentrons_96_tiprack_300ul", "10")
    deck["11"] = protocol.load_labware("opentrons_96_tiprack_1000ul", "11")
    pipettes["right"] = protocol.load_instrument("p1000_single_gen2", "right", tip_racks=[deck["11"]])

    ####################
    # execute commands #
    ####################

    # Mix Color 1
    pipettes["right"].pick_up_tip(deck["11"].wells()[0])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(98.74258858180228, deck["7"]["A1"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(98.74258858180228, deck["2"]["A1"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[1])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(11.754619636312132, deck["7"]["A1"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(11.754619636312132, deck["2"]["A2"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[2])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(170.7652379922411, deck["7"]["A1"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(170.7652379922411, deck["2"]["A3"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[3])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(76.03668723155191, deck["7"]["A1"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(76.03668723155191, deck["2"]["A4"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()


    # Mix color 2
    pipettes["right"].pick_up_tip(deck["11"].wells()[4])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(48.118119721634365, deck["7"]["A2"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(48.118119721634365, deck["2"]["A1"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[5])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(138.1632947791648, deck["7"]["A2"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(138.1632947791648, deck["2"]["A2"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[6])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(74.47829427247885, deck["7"]["A2"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(74.47829427247885, deck["2"]["A3"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[7])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(57.4165121425626, deck["7"]["A2"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(57.4165121425626, deck["2"]["A4"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()


    # Mix color 3
    pipettes["right"].pick_up_tip(deck["11"].wells()[8])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(128.13929169656336, deck["7"]["A3"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(128.13929169656336, deck["2"]["A1"])
    pipettes["right"].mix(3, 100, deck["2"]["A1"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[9])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(125.08208558452307, deck["7"]["A3"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(125.08208558452307, deck["2"]["A2"])
    pipettes["right"].mix(3, 100, deck["2"]["A2"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[10])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(29.756467735280072, deck["7"]["A3"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(29.756467735280072, deck["2"]["A3"])
    pipettes["right"].mix(3, 100, deck["2"]["A3"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[11])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(141.5468006258855, deck["7"]["A3"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(141.5468006258855, deck["2"]["A4"])
    pipettes["right"].mix(3, 100, deck["2"]["A4"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()
