from opentrons import protocol_api


metadata = {
    "protocolName": "PCR Prep Full Plate",
    "author": "Abe astroka@anl.gov",
    "description": "mixing primers and templates with wells read in from file",
    "apiLevel": "2.12"
}

def run(protocol: protocol_api.ProtocolContext):

    deck = {}
    pipettes = {}

    ################
    # load labware #
    ################
    deck["1"] = protocol.load_labware("nest_96_wellplate_2ml_deep", "1")
    deck["1"].set_offset(x=0.0, y=1.7, z=3.9)
    deck["2"] = protocol.load_labware("nest_96_wellplate_2ml_deep", "2")
    deck["2"].set_offset(x=0.0, y=1.0, z=1.4)
    module = protocol.load_module("Temperature Module", "3")
    deck["3"] = module.load_labware("nest_96_wellplate_100ul_pcr_full_skirt")
    deck["3"].set_offset(x=1.0, y=1.6, z=11.0)
    deck["4"] = protocol.load_labware("nest_96_wellplate_2ml_deep", "4")
    deck["4"].set_offset(x=0.0, y=0.0, z=1.0)
    deck["6"] = protocol.load_labware("opentrons_24_tuberack_eppendorf_1.5ml_safelock_snapcap", "6")
    deck["6"].set_offset(x=0.0, y=0.0, z=0.5)
    deck["5"] = protocol.load_labware("opentrons_10_tuberack_nest_4x50ml_6x15ml_conical", "5")
    deck["11"] = protocol.load_labware("opentrons_96_tiprack_20ul", "11")
    deck["11"].set_offset(x=0.2, y=0.4, z=0.0)
    deck["9"] = protocol.load_labware("opentrons_96_tiprack_1000ul", "9")
    deck["9"].set_offset(x=0.5, y=0.9, z=-0.3)
    pipettes["left"] = protocol.load_instrument("p20_single_gen2", "left", tip_racks=[deck["11"]])
    pipettes["right"] = protocol.load_instrument("p1000_single_gen2", "right", tip_racks=[deck["9"]])

    ####################
    # execute commands #
    ####################

    # Cool Block
    module.set_temperature(4)

    # BioWater
    pipettes["right"].pick_up_tip(deck["9"].wells()[0])
    pipettes["right"].well_bottom_clearance.aspirate = 50.0
    pipettes["right"].aspirate(676.0, deck["5"]["A1"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(676.0, deck["5"]["C2"])
    pipettes["right"].blow_out()

    pipettes["right"].well_bottom_clearance.aspirate = 50.0
    pipettes["right"].aspirate(676.0, deck["5"]["A1"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(676.0, deck["5"]["C2"])
    pipettes["right"].blow_out()

    pipettes["right"].well_bottom_clearance.aspirate = 50.0
    pipettes["right"].aspirate(676.0, deck["5"]["A1"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(676.0, deck["5"]["C2"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()


    # 5x Reaction Buffer
    pipettes["right"].pick_up_tip(deck["9"].wells()[1])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(720.0, deck["6"]["A3"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(720.0, deck["5"]["C2"])
    pipettes["right"].mix(3, 500, deck["5"]["C2"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()


    # DNA Polymerase
    pipettes["left"].pick_up_tip(deck["11"].wells()[0])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(12.0, deck["6"]["A1"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(12.0, deck["5"]["C2"])
    pipettes["left"].mix(3, 20, deck["5"]["C2"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(12.0, deck["6"]["A1"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(12.0, deck["5"]["C2"])
    pipettes["left"].mix(3, 20, deck["5"]["C2"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()


    # DNTPs
    pipettes["right"].pick_up_tip(deck["9"].wells()[2])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(108.0, deck["6"]["A5"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(108.0, deck["5"]["C2"])
    pipettes["right"].mix(3, 600, deck["5"]["C2"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()


    # GC Enhancer
    pipettes["right"].pick_up_tip(deck["9"].wells()[3])
    pipettes["right"].well_bottom_clearance.aspirate = 0.1
    pipettes["right"].aspirate(720.0, deck["6"]["B3"])
    pipettes["right"].well_bottom_clearance.dispense = 2.0
    pipettes["right"].dispense(720.0, deck["5"]["C2"])
    pipettes["right"].mix(7, 700, deck["5"]["C2"])
    pipettes["right"].blow_out()
    pipettes["right"].drop_tip()


    # master mix distribute
    pipettes["left"].pick_up_tip(deck["11"].wells()[1])
    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["A1"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["A2"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["A3"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["A4"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["A5"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["A6"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["A7"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["A8"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["A9"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["A10"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["A11"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["A12"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["B1"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["B2"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["B3"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["B4"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["B5"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["B6"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["B7"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["B8"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["B9"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["B10"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["B11"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["B12"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["C1"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["C2"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["C3"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["C4"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["C5"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["C6"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["C7"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["C8"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["C9"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["C10"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["C11"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["C12"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["D1"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["D2"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["D3"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["D4"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["D5"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["D6"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["D7"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["D8"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["D9"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["D10"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["D11"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["D12"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["E1"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["E2"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["E3"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["E4"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["E5"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["E6"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["E7"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["E8"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["E9"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["E10"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["E11"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["E12"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["F1"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["F2"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["F3"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["F4"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["F5"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["F6"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["F7"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["F8"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["F9"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["F10"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["F11"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["F12"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["G1"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["G2"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["G3"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["G4"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["G5"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["G6"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["G7"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["G8"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["G9"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["G10"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["G11"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["G12"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["H1"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["H2"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["H3"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["H4"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["H5"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["H6"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["H7"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["H8"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["H9"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["H10"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["H11"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["H12"])
    pipettes["left"].blow_out()


    # master mix distribute 2
    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["A1"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["A2"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["A3"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["A4"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["A5"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["A6"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["A7"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["A8"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["A9"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["A10"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["A11"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["A12"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["B1"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["B2"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["B3"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["B4"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["B5"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["B6"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["B7"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["B8"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["B9"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["B10"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["B11"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["B12"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["C1"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["C2"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["C3"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["C4"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["C5"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["C6"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["C7"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["C8"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["C9"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["C10"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["C11"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["C12"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["D1"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["D2"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["D3"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["D4"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["D5"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["D6"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["D7"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["D8"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["D9"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["D10"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["D11"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["D12"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["E1"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["E2"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["E3"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["E4"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["E5"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["E6"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["E7"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["E8"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["E9"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["E10"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["E11"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["E12"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["F1"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["F2"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["F3"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["F4"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["F5"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["F6"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["F7"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["F8"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["F9"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["F10"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["F11"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["F12"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["G1"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["G2"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["G3"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["G4"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["G5"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["G6"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["G7"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["G8"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["G9"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["G10"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["G11"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["G12"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["H1"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["H2"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["H3"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["H4"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["H5"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["H6"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["H7"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["H8"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["H9"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["H10"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["H11"])
    pipettes["left"].blow_out()

    pipettes["left"].well_bottom_clearance.aspirate = 1.0
    pipettes["left"].aspirate(15.0, deck["5"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 1.0
    pipettes["left"].dispense(15.0, deck["3"]["H12"])
    pipettes["left"].blow_out()

    pipettes["left"].drop_tip()