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
    deck["11"] = protocol.load_labware("opentrons_96_tiprack_20ul", "11")
    pipettes["left"] = protocol.load_instrument("p300_single_gen2", "left", tip_racks=[deck["10"]])
    pipettes["right"] = protocol.load_instrument("p20_single_gen2", "right", tip_racks=[deck["11"]])

    ####################
    # execute commands #
    ####################

    # Mix Color 1
    pipettes["left"].pick_up_tip(deck["10"].wells()[0])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(38.18026206235341, deck["7"]["A1"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(38.18026206235341, deck["2"]["A1"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[1])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(131.31874460530528, deck["7"]["A1"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(131.31874460530528, deck["2"]["A2"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[2])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(37.96058245433187, deck["7"]["A1"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(37.96058245433187, deck["2"]["A3"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[3])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(125.1500813879697, deck["7"]["A1"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(125.1500813879697, deck["2"]["A4"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()


    # Mix color 2
    pipettes["left"].pick_up_tip(deck["10"].wells()[4])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(173.70345975520513, deck["7"]["A2"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(173.70345975520513, deck["2"]["A1"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[5])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(64.17736813700432, deck["7"]["A2"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(64.17736813700432, deck["2"]["A2"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[6])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(70.30742485468508, deck["7"]["A2"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(70.30742485468508, deck["2"]["A3"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[7])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(134.64787834527525, deck["7"]["A2"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(134.64787834527525, deck["2"]["A4"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()


    # Mix color 3
    pipettes["left"].pick_up_tip(deck["10"].wells()[8])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(63.11627818244148, deck["7"]["A3"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(63.11627818244148, deck["2"]["A1"])
    pipettes["left"].mix(3, 100, deck["2"]["A1"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[9])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(79.50388725769037, deck["7"]["A3"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(79.50388725769037, deck["2"]["A2"])
    pipettes["left"].mix(3, 100, deck["2"]["A2"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[10])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(166.73199269098305, deck["7"]["A3"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(166.73199269098305, deck["2"]["A3"])
    pipettes["left"].mix(3, 100, deck["2"]["A3"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[0])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(15.202040266755082, deck["7"]["A3"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(15.202040266755082, deck["2"]["A4"])
    pipettes["right"].mix(3, 100, deck["2"]["A4"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()
