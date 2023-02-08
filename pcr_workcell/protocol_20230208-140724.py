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
    pipettes["right"].aspirate(102.64717599116221, deck["7"]["A1"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(102.64717599116221, deck["2"]["A1"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[1])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(219.02551156384388, deck["7"]["A1"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(219.02551156384388, deck["2"]["A2"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[2])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(27.516754544398598, deck["7"]["A1"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(27.516754544398598, deck["2"]["A3"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[3])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(93.81039595503655, deck["7"]["A1"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(93.81039595503655, deck["2"]["A4"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()


    # Mix color 2
    pipettes["right"].pick_up_tip(deck["11"].wells()[4])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(131.57637639025097, deck["7"]["A2"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(131.57637639025097, deck["2"]["A1"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[5])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(49.83266306838528, deck["7"]["A2"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(49.83266306838528, deck["2"]["A2"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[6])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(177.49223691169, deck["7"]["A2"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(177.49223691169, deck["2"]["A3"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[7])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(97.34622251328774, deck["7"]["A2"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(97.34622251328774, deck["2"]["A4"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()


    # Mix color 3
    pipettes["right"].pick_up_tip(deck["11"].wells()[8])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(40.77644761858685, deck["7"]["A3"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(40.77644761858685, deck["2"]["A1"])
    pipettes["right"].mix(3, 100, deck["2"]["A1"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[9])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(6.141825367770809, deck["7"]["A3"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(6.141825367770809, deck["2"]["A2"])
    pipettes["right"].mix(3, 100, deck["2"]["A2"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[10])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(69.99100854391146, deck["7"]["A3"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(69.99100854391146, deck["2"]["A3"])
    pipettes["right"].mix(3, 100, deck["2"]["A3"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[11])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(83.84338153167569, deck["7"]["A3"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(83.84338153167569, deck["2"]["A4"])
    pipettes["right"].mix(3, 100, deck["2"]["A4"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()
