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
    pipettes["right"].aspirate(122.33342723745231, deck["7"]["A1"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(122.33342723745231, deck["2"]["A1"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[1])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(119.54097958514501, deck["7"]["A1"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(119.54097958514501, deck["2"]["A2"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[2])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(143.71772019165485, deck["7"]["A1"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(143.71772019165485, deck["2"]["A3"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[3])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(2.719984241183729, deck["7"]["A1"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(2.719984241183729, deck["2"]["A4"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()


    # Mix color 2
    pipettes["right"].pick_up_tip(deck["11"].wells()[4])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(24.91248224101308, deck["7"]["A2"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(24.91248224101308, deck["2"]["A1"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[5])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(112.32086564306039, deck["7"]["A2"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(112.32086564306039, deck["2"]["A2"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[6])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(63.245364062600835, deck["7"]["A2"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(63.245364062600835, deck["2"]["A3"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[7])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(112.04692916274314, deck["7"]["A2"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(112.04692916274314, deck["2"]["A4"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()


    # Mix color 3
    pipettes["right"].pick_up_tip(deck["11"].wells()[8])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(127.75409052153461, deck["7"]["A3"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(127.75409052153461, deck["2"]["A1"])
    pipettes["right"].mix(3, 100, deck["2"]["A1"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[9])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(43.13815477179463, deck["7"]["A3"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(43.13815477179463, deck["2"]["A2"])
    pipettes["right"].mix(3, 100, deck["2"]["A2"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[10])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(68.0369157457443, deck["7"]["A3"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(68.0369157457443, deck["2"]["A3"])
    pipettes["right"].mix(3, 100, deck["2"]["A3"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[11])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(160.23308659607312, deck["7"]["A3"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(160.23308659607312, deck["2"]["A4"])
    pipettes["right"].mix(3, 100, deck["2"]["A4"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()
