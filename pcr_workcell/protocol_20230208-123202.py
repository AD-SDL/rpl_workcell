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
    pipettes["right"].aspirate(3.0382266787469274, deck["7"]["A1"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(3.0382266787469274, deck["2"]["A1"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[1])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(119.22765800252287, deck["7"]["A1"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(119.22765800252287, deck["2"]["A2"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[2])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(19.62922524122895, deck["7"]["A1"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(19.62922524122895, deck["2"]["A3"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[3])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(67.88129213716188, deck["7"]["A1"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(67.88129213716188, deck["2"]["A4"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()


    # Mix color 2
    pipettes["right"].pick_up_tip(deck["11"].wells()[4])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(106.08265877233293, deck["7"]["A2"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(106.08265877233293, deck["2"]["A1"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[5])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(20.604004615381605, deck["7"]["A2"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(20.604004615381605, deck["2"]["A2"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[6])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(155.97932911217947, deck["7"]["A2"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(155.97932911217947, deck["2"]["A3"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[7])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(41.90416681643373, deck["7"]["A2"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(41.90416681643373, deck["2"]["A4"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()


    # Mix color 3
    pipettes["right"].pick_up_tip(deck["11"].wells()[8])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(165.87911454892014, deck["7"]["A3"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(165.87911454892014, deck["2"]["A1"])
    pipettes["right"].mix(3, 100, deck["2"]["A1"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[9])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(135.16833738209556, deck["7"]["A3"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(135.16833738209556, deck["2"]["A2"])
    pipettes["right"].mix(3, 100, deck["2"]["A2"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[10])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(99.39144564659155, deck["7"]["A3"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(99.39144564659155, deck["2"]["A3"])
    pipettes["right"].mix(3, 100, deck["2"]["A3"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[11])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(165.21454104640438, deck["7"]["A3"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(165.21454104640438, deck["2"]["A4"])
    pipettes["right"].mix(3, 100, deck["2"]["A4"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()
