from opentrons import protocol_api


metadata = {
    "protocolName": "All color mixing",
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
    deck["7"] = protocol.load_labware("opentrons_6_tuberack_nest_50ml_conical", "7")
    deck["2"] = protocol.load_labware("corning_96_wellplate_360ul_flat", "2")
    deck["10"] = protocol.load_labware("opentrons_96_tiprack_1000ul", "10")
    deck["11"] = protocol.load_labware("opentrons_96_tiprack_20ul", "11")
    pipettes["left"] = protocol.load_instrument("p1000_single_gen2", "left", tip_racks=[deck["10"]])
    pipettes["right"] = protocol.load_instrument("p20_single_gen2", "right", tip_racks=[deck["11"]])

    ####################
    # execute commands #
    ####################

    # Mix Color 1
    pipettes["right"].pick_up_tip(deck["11"].wells()[0])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(2.39608887294191, deck["7"]["A1"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(2.39608887294191, deck["2"]["A1"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[1])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(8.786539608282402, deck["7"]["A1"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(8.786539608282402, deck["2"]["A2"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[2])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(14.07144704664365, deck["7"]["A1"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(14.07144704664365, deck["2"]["A3"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[3])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(1.121001514645238, deck["7"]["A1"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(1.121001514645238, deck["2"]["A4"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[4])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(11.099817774421707, deck["7"]["A1"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(11.099817774421707, deck["2"]["A5"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[5])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(15.824586862633153, deck["7"]["A1"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(15.824586862633153, deck["2"]["A6"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[6])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(11.791569428014888, deck["7"]["A1"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(11.791569428014888, deck["2"]["A7"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[7])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(3.991067268261782, deck["7"]["A1"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(3.991067268261782, deck["2"]["A8"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()


    # Mix color 2
    pipettes["right"].pick_up_tip(deck["11"].wells()[8])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(11.188299338655114, deck["7"]["A2"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(11.188299338655114, deck["2"]["A1"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[9])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(13.789745540783555, deck["7"]["A2"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(13.789745540783555, deck["2"]["A2"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[10])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(7.810423510467851, deck["7"]["A2"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(7.810423510467851, deck["2"]["A3"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[11])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(17.381107451224604, deck["7"]["A2"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(17.381107451224604, deck["2"]["A4"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[12])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(2.2058734853448607, deck["7"]["A2"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(2.2058734853448607, deck["2"]["A5"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[13])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(7.5973922713329705, deck["7"]["A2"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(7.5973922713329705, deck["2"]["A6"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[14])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(17.061747366611577, deck["7"]["A2"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(17.061747366611577, deck["2"]["A7"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[15])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(12.584237170216076, deck["7"]["A2"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(12.584237170216076, deck["2"]["A8"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()


    # Mix color 3
    pipettes["right"].pick_up_tip(deck["11"].wells()[16])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(16.415611788402977, deck["7"]["A3"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(16.415611788402977, deck["2"]["A1"])
    pipettes["right"].mix(3, 15, deck["2"]["A1"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[17])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(7.4237148509340445, deck["7"]["A3"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(7.4237148509340445, deck["2"]["A2"])
    pipettes["right"].mix(3, 15, deck["2"]["A2"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[18])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(8.118129442888502, deck["7"]["A3"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(8.118129442888502, deck["2"]["A3"])
    pipettes["right"].mix(3, 15, deck["2"]["A3"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[19])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(11.497891034130156, deck["7"]["A3"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(11.497891034130156, deck["2"]["A4"])
    pipettes["right"].mix(3, 15, deck["2"]["A4"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[20])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(16.694308740233435, deck["7"]["A3"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(16.694308740233435, deck["2"]["A5"])
    pipettes["right"].mix(3, 15, deck["2"]["A5"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[21])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(6.578020866033876, deck["7"]["A3"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(6.578020866033876, deck["2"]["A6"])
    pipettes["right"].mix(3, 15, deck["2"]["A6"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[22])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(1.1466832053735312, deck["7"]["A3"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(1.1466832053735312, deck["2"]["A7"])
    pipettes["right"].mix(3, 15, deck["2"]["A7"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()

    pipettes["right"].pick_up_tip(deck["11"].wells()[23])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(13.42469556152214, deck["7"]["A3"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(13.42469556152214, deck["2"]["A8"])
    pipettes["right"].mix(3, 15, deck["2"]["A8"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()
