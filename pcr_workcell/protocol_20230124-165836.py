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
    deck["8"] = protocol.load_labware("opentrons_96_tiprack_20ul", "8")
    deck["8"].set_offset(x=0.2, y=0.4, z=0.0)
    deck["7"] = protocol.load_labware("opentrons_96_tiprack_20ul", "7")
    deck["7"].set_offset(x=0.2, y=0.4, z=0.0)
    deck["10"] = protocol.load_labware("opentrons_96_tiprack_20ul", "10")
    deck["10"].set_offset(x=0.2, y=0.4, z=0.0)
    deck["6"] = protocol.load_labware("opentrons_24_tuberack_eppendorf_1.5ml_safelock_snapcap", "6")
    deck["6"].set_offset(x=0.0, y=0.0, z=0.5)
    deck["5"] = protocol.load_labware("opentrons_10_tuberack_nest_4x50ml_6x15ml_conical", "5")
    deck["9"] = protocol.load_labware("opentrons_96_tiprack_1000ul", "9")
    deck["9"].set_offset(x=0.5, y=0.9, z=-0.3)
    pipettes["left"] = protocol.load_instrument("p20_single_gen2", "left", tip_racks=[deck["8"], deck["7"], deck["10"]])
    pipettes["right"] = protocol.load_instrument("p1000_single_gen2", "right", tip_racks=[deck["9"]])

    ####################
    # execute commands #
    ####################

    # Cool Block
    module.set_temperature(4)

    # forward primer
    pipettes["left"].pick_up_tip(deck["8"].wells()[0])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["A1"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["A1"])
    pipettes["left"].mix(3, 15, deck["3"]["A1"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[1])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["A2"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["A2"])
    pipettes["left"].mix(3, 15, deck["3"]["A2"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[2])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["A3"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["A3"])
    pipettes["left"].mix(3, 15, deck["3"]["A3"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[3])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["A4"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["A4"])
    pipettes["left"].mix(3, 15, deck["3"]["A4"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[4])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["A5"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["A5"])
    pipettes["left"].mix(3, 15, deck["3"]["A5"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[5])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["A6"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["A6"])
    pipettes["left"].mix(3, 15, deck["3"]["A6"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[6])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["A7"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["A7"])
    pipettes["left"].mix(3, 15, deck["3"]["A7"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[7])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["A8"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["A8"])
    pipettes["left"].mix(3, 15, deck["3"]["A8"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[8])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["A9"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["A9"])
    pipettes["left"].mix(3, 15, deck["3"]["A9"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[9])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["A10"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["A10"])
    pipettes["left"].mix(3, 15, deck["3"]["A10"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[10])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["A11"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["A11"])
    pipettes["left"].mix(3, 15, deck["3"]["A11"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[11])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["A12"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["A12"])
    pipettes["left"].mix(3, 15, deck["3"]["A12"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[12])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["B1"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["B1"])
    pipettes["left"].mix(3, 15, deck["3"]["B1"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[13])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["B2"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["B2"])
    pipettes["left"].mix(3, 15, deck["3"]["B2"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[14])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["B3"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["B3"])
    pipettes["left"].mix(3, 15, deck["3"]["B3"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[15])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["B4"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["B4"])
    pipettes["left"].mix(3, 15, deck["3"]["B4"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[16])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["B5"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["B5"])
    pipettes["left"].mix(3, 15, deck["3"]["B5"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[17])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["B6"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["B6"])
    pipettes["left"].mix(3, 15, deck["3"]["B6"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[18])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["B7"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["B7"])
    pipettes["left"].mix(3, 15, deck["3"]["B7"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[19])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["B8"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["B8"])
    pipettes["left"].mix(3, 15, deck["3"]["B8"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[20])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["B9"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["B9"])
    pipettes["left"].mix(3, 15, deck["3"]["B9"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[21])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["B10"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["B10"])
    pipettes["left"].mix(3, 15, deck["3"]["B10"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[22])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["B11"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["B11"])
    pipettes["left"].mix(3, 15, deck["3"]["B11"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[23])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["B12"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["B12"])
    pipettes["left"].mix(3, 15, deck["3"]["B12"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[24])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["C1"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["C1"])
    pipettes["left"].mix(3, 15, deck["3"]["C1"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[25])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["C2"])
    pipettes["left"].mix(3, 15, deck["3"]["C2"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[26])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["C3"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["C3"])
    pipettes["left"].mix(3, 15, deck["3"]["C3"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[27])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["C4"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["C4"])
    pipettes["left"].mix(3, 15, deck["3"]["C4"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[28])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["C5"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["C5"])
    pipettes["left"].mix(3, 15, deck["3"]["C5"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[29])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["C6"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["C6"])
    pipettes["left"].mix(3, 15, deck["3"]["C6"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[30])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["C7"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["C7"])
    pipettes["left"].mix(3, 15, deck["3"]["C7"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[31])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["C8"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["C8"])
    pipettes["left"].mix(3, 15, deck["3"]["C8"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[32])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["C9"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["C9"])
    pipettes["left"].mix(3, 15, deck["3"]["C9"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[33])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["C10"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["C10"])
    pipettes["left"].mix(3, 15, deck["3"]["C10"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[34])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["C11"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["C11"])
    pipettes["left"].mix(3, 15, deck["3"]["C11"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[35])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["C12"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["C12"])
    pipettes["left"].mix(3, 15, deck["3"]["C12"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[36])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["D1"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["D1"])
    pipettes["left"].mix(3, 15, deck["3"]["D1"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[37])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["D2"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["D2"])
    pipettes["left"].mix(3, 15, deck["3"]["D2"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[38])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["D3"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["D3"])
    pipettes["left"].mix(3, 15, deck["3"]["D3"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[39])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["D4"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["D4"])
    pipettes["left"].mix(3, 15, deck["3"]["D4"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[40])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["D5"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["D5"])
    pipettes["left"].mix(3, 15, deck["3"]["D5"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[41])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["D6"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["D6"])
    pipettes["left"].mix(3, 15, deck["3"]["D6"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[42])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["D7"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["D7"])
    pipettes["left"].mix(3, 15, deck["3"]["D7"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[43])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["D8"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["D8"])
    pipettes["left"].mix(3, 15, deck["3"]["D8"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[44])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["D9"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["D9"])
    pipettes["left"].mix(3, 15, deck["3"]["D9"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[45])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["D10"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["D10"])
    pipettes["left"].mix(3, 15, deck["3"]["D10"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[46])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["D11"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["D11"])
    pipettes["left"].mix(3, 15, deck["3"]["D11"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[47])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["D12"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["D12"])
    pipettes["left"].mix(3, 15, deck["3"]["D12"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[48])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["E1"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["E1"])
    pipettes["left"].mix(3, 15, deck["3"]["E1"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[49])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["E2"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["E2"])
    pipettes["left"].mix(3, 15, deck["3"]["E2"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[50])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["E3"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["E3"])
    pipettes["left"].mix(3, 15, deck["3"]["E3"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[51])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["E4"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["E4"])
    pipettes["left"].mix(3, 15, deck["3"]["E4"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[52])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["E5"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["E5"])
    pipettes["left"].mix(3, 15, deck["3"]["E5"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[53])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["E6"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["E6"])
    pipettes["left"].mix(3, 15, deck["3"]["E6"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[54])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["E7"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["E7"])
    pipettes["left"].mix(3, 15, deck["3"]["E7"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[55])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["E8"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["E8"])
    pipettes["left"].mix(3, 15, deck["3"]["E8"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[56])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["E9"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["E9"])
    pipettes["left"].mix(3, 15, deck["3"]["E9"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[57])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["E10"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["E10"])
    pipettes["left"].mix(3, 15, deck["3"]["E10"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[58])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["E11"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["E11"])
    pipettes["left"].mix(3, 15, deck["3"]["E11"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[59])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["E12"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["E12"])
    pipettes["left"].mix(3, 15, deck["3"]["E12"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[60])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["F1"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["F1"])
    pipettes["left"].mix(3, 15, deck["3"]["F1"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[61])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["F2"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["F2"])
    pipettes["left"].mix(3, 15, deck["3"]["F2"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[62])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["F3"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["F3"])
    pipettes["left"].mix(3, 15, deck["3"]["F3"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[63])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["F4"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["F4"])
    pipettes["left"].mix(3, 15, deck["3"]["F4"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[64])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["F5"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["F5"])
    pipettes["left"].mix(3, 15, deck["3"]["F5"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[65])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["F6"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["F6"])
    pipettes["left"].mix(3, 15, deck["3"]["F6"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[66])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["F7"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["F7"])
    pipettes["left"].mix(3, 15, deck["3"]["F7"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[67])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["F8"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["F8"])
    pipettes["left"].mix(3, 15, deck["3"]["F8"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[68])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["F9"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["F9"])
    pipettes["left"].mix(3, 15, deck["3"]["F9"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[69])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["F10"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["F10"])
    pipettes["left"].mix(3, 15, deck["3"]["F10"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[70])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["F11"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["F11"])
    pipettes["left"].mix(3, 15, deck["3"]["F11"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[71])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["F12"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["F12"])
    pipettes["left"].mix(3, 15, deck["3"]["F12"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[72])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["G1"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["G1"])
    pipettes["left"].mix(3, 15, deck["3"]["G1"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[73])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["G2"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["G2"])
    pipettes["left"].mix(3, 15, deck["3"]["G2"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[74])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["G3"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["G3"])
    pipettes["left"].mix(3, 15, deck["3"]["G3"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[75])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["G4"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["G4"])
    pipettes["left"].mix(3, 15, deck["3"]["G4"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[76])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["G5"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["G5"])
    pipettes["left"].mix(3, 15, deck["3"]["G5"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[77])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["G6"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["G6"])
    pipettes["left"].mix(3, 15, deck["3"]["G6"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[78])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["G7"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["G7"])
    pipettes["left"].mix(3, 15, deck["3"]["G7"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[79])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["G8"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["G8"])
    pipettes["left"].mix(3, 15, deck["3"]["G8"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[80])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["G9"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["G9"])
    pipettes["left"].mix(3, 15, deck["3"]["G9"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[81])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["G10"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["G10"])
    pipettes["left"].mix(3, 15, deck["3"]["G10"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[82])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["G11"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["G11"])
    pipettes["left"].mix(3, 15, deck["3"]["G11"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[83])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["G12"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["G12"])
    pipettes["left"].mix(3, 15, deck["3"]["G12"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[84])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["H1"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["H1"])
    pipettes["left"].mix(3, 15, deck["3"]["H1"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[85])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["H2"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["H2"])
    pipettes["left"].mix(3, 15, deck["3"]["H2"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[86])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["H3"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["H3"])
    pipettes["left"].mix(3, 15, deck["3"]["H3"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[87])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["H4"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["H4"])
    pipettes["left"].mix(3, 15, deck["3"]["H4"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[88])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["H5"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["H5"])
    pipettes["left"].mix(3, 15, deck["3"]["H5"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[89])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["H6"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["H6"])
    pipettes["left"].mix(3, 15, deck["3"]["H6"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[90])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["H7"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["H7"])
    pipettes["left"].mix(3, 15, deck["3"]["H7"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[91])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["H8"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["H8"])
    pipettes["left"].mix(3, 15, deck["3"]["H8"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[92])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["H9"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["H9"])
    pipettes["left"].mix(3, 15, deck["3"]["H9"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[93])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["H10"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["H10"])
    pipettes["left"].mix(3, 15, deck["3"]["H10"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[94])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["H11"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["H11"])
    pipettes["left"].mix(3, 15, deck["3"]["H11"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["8"].wells()[95])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["1"]["H12"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["H12"])
    pipettes["left"].mix(3, 15, deck["3"]["H12"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()


    # backward primer
    pipettes["left"].pick_up_tip(deck["7"].wells()[0])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["A1"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["A1"])
    pipettes["left"].mix(3, 15, deck["3"]["A1"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[1])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["A2"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["A2"])
    pipettes["left"].mix(3, 15, deck["3"]["A2"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[2])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["A3"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["A3"])
    pipettes["left"].mix(3, 15, deck["3"]["A3"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[3])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["A4"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["A4"])
    pipettes["left"].mix(3, 15, deck["3"]["A4"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[4])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["A5"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["A5"])
    pipettes["left"].mix(3, 15, deck["3"]["A5"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[5])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["A6"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["A6"])
    pipettes["left"].mix(3, 15, deck["3"]["A6"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[6])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["A7"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["A7"])
    pipettes["left"].mix(3, 15, deck["3"]["A7"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[7])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["A8"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["A8"])
    pipettes["left"].mix(3, 15, deck["3"]["A8"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[8])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["A9"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["A9"])
    pipettes["left"].mix(3, 15, deck["3"]["A9"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[9])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["A10"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["A10"])
    pipettes["left"].mix(3, 15, deck["3"]["A10"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[10])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["A11"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["A11"])
    pipettes["left"].mix(3, 15, deck["3"]["A11"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[11])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["A12"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["A12"])
    pipettes["left"].mix(3, 15, deck["3"]["A12"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[12])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["B1"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["B1"])
    pipettes["left"].mix(3, 15, deck["3"]["B1"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[13])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["B2"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["B2"])
    pipettes["left"].mix(3, 15, deck["3"]["B2"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[14])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["B3"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["B3"])
    pipettes["left"].mix(3, 15, deck["3"]["B3"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[15])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["B4"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["B4"])
    pipettes["left"].mix(3, 15, deck["3"]["B4"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[16])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["B5"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["B5"])
    pipettes["left"].mix(3, 15, deck["3"]["B5"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[17])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["B6"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["B6"])
    pipettes["left"].mix(3, 15, deck["3"]["B6"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[18])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["B7"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["B7"])
    pipettes["left"].mix(3, 15, deck["3"]["B7"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[19])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["B8"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["B8"])
    pipettes["left"].mix(3, 15, deck["3"]["B8"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[20])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["B9"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["B9"])
    pipettes["left"].mix(3, 15, deck["3"]["B9"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[21])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["B10"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["B10"])
    pipettes["left"].mix(3, 15, deck["3"]["B10"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[22])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["B11"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["B11"])
    pipettes["left"].mix(3, 15, deck["3"]["B11"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[23])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["B12"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["B12"])
    pipettes["left"].mix(3, 15, deck["3"]["B12"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[24])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["C1"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["C1"])
    pipettes["left"].mix(3, 15, deck["3"]["C1"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[25])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["C2"])
    pipettes["left"].mix(3, 15, deck["3"]["C2"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[26])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["C3"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["C3"])
    pipettes["left"].mix(3, 15, deck["3"]["C3"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[27])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["C4"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["C4"])
    pipettes["left"].mix(3, 15, deck["3"]["C4"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[28])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["C5"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["C5"])
    pipettes["left"].mix(3, 15, deck["3"]["C5"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[29])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["C6"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["C6"])
    pipettes["left"].mix(3, 15, deck["3"]["C6"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[30])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["C7"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["C7"])
    pipettes["left"].mix(3, 15, deck["3"]["C7"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[31])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["C8"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["C8"])
    pipettes["left"].mix(3, 15, deck["3"]["C8"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[32])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["C9"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["C9"])
    pipettes["left"].mix(3, 15, deck["3"]["C9"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[33])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["C10"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["C10"])
    pipettes["left"].mix(3, 15, deck["3"]["C10"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[34])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["C11"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["C11"])
    pipettes["left"].mix(3, 15, deck["3"]["C11"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[35])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["C12"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["C12"])
    pipettes["left"].mix(3, 15, deck["3"]["C12"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[36])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["D1"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["D1"])
    pipettes["left"].mix(3, 15, deck["3"]["D1"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[37])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["D2"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["D2"])
    pipettes["left"].mix(3, 15, deck["3"]["D2"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[38])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["D3"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["D3"])
    pipettes["left"].mix(3, 15, deck["3"]["D3"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[39])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["D4"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["D4"])
    pipettes["left"].mix(3, 15, deck["3"]["D4"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[40])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["D5"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["D5"])
    pipettes["left"].mix(3, 15, deck["3"]["D5"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[41])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["D6"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["D6"])
    pipettes["left"].mix(3, 15, deck["3"]["D6"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[42])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["D7"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["D7"])
    pipettes["left"].mix(3, 15, deck["3"]["D7"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[43])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["D8"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["D8"])
    pipettes["left"].mix(3, 15, deck["3"]["D8"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[44])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["D9"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["D9"])
    pipettes["left"].mix(3, 15, deck["3"]["D9"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[45])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["D10"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["D10"])
    pipettes["left"].mix(3, 15, deck["3"]["D10"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[46])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["D11"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["D11"])
    pipettes["left"].mix(3, 15, deck["3"]["D11"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[47])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["D12"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["D12"])
    pipettes["left"].mix(3, 15, deck["3"]["D12"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[48])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["E1"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["E1"])
    pipettes["left"].mix(3, 15, deck["3"]["E1"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[49])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["E2"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["E2"])
    pipettes["left"].mix(3, 15, deck["3"]["E2"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[50])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["E3"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["E3"])
    pipettes["left"].mix(3, 15, deck["3"]["E3"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[51])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["E4"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["E4"])
    pipettes["left"].mix(3, 15, deck["3"]["E4"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[52])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["E5"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["E5"])
    pipettes["left"].mix(3, 15, deck["3"]["E5"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[53])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["E6"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["E6"])
    pipettes["left"].mix(3, 15, deck["3"]["E6"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[54])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["E7"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["E7"])
    pipettes["left"].mix(3, 15, deck["3"]["E7"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[55])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["E8"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["E8"])
    pipettes["left"].mix(3, 15, deck["3"]["E8"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[56])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["E9"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["E9"])
    pipettes["left"].mix(3, 15, deck["3"]["E9"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[57])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["E10"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["E10"])
    pipettes["left"].mix(3, 15, deck["3"]["E10"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[58])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["E11"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["E11"])
    pipettes["left"].mix(3, 15, deck["3"]["E11"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[59])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["E12"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["E12"])
    pipettes["left"].mix(3, 15, deck["3"]["E12"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[60])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["F1"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["F1"])
    pipettes["left"].mix(3, 15, deck["3"]["F1"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[61])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["F2"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["F2"])
    pipettes["left"].mix(3, 15, deck["3"]["F2"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[62])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["F3"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["F3"])
    pipettes["left"].mix(3, 15, deck["3"]["F3"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[63])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["F4"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["F4"])
    pipettes["left"].mix(3, 15, deck["3"]["F4"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[64])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["F5"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["F5"])
    pipettes["left"].mix(3, 15, deck["3"]["F5"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[65])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["F6"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["F6"])
    pipettes["left"].mix(3, 15, deck["3"]["F6"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[66])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["F7"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["F7"])
    pipettes["left"].mix(3, 15, deck["3"]["F7"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[67])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["F8"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["F8"])
    pipettes["left"].mix(3, 15, deck["3"]["F8"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[68])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["F9"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["F9"])
    pipettes["left"].mix(3, 15, deck["3"]["F9"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[69])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["F10"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["F10"])
    pipettes["left"].mix(3, 15, deck["3"]["F10"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[70])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["F11"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["F11"])
    pipettes["left"].mix(3, 15, deck["3"]["F11"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[71])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["F12"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["F12"])
    pipettes["left"].mix(3, 15, deck["3"]["F12"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[72])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["G1"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["G1"])
    pipettes["left"].mix(3, 15, deck["3"]["G1"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[73])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["G2"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["G2"])
    pipettes["left"].mix(3, 15, deck["3"]["G2"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[74])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["G3"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["G3"])
    pipettes["left"].mix(3, 15, deck["3"]["G3"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[75])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["G4"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["G4"])
    pipettes["left"].mix(3, 15, deck["3"]["G4"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[76])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["G5"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["G5"])
    pipettes["left"].mix(3, 15, deck["3"]["G5"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[77])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["G6"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["G6"])
    pipettes["left"].mix(3, 15, deck["3"]["G6"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[78])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["G7"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["G7"])
    pipettes["left"].mix(3, 15, deck["3"]["G7"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[79])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["G8"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["G8"])
    pipettes["left"].mix(3, 15, deck["3"]["G8"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[80])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["G9"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["G9"])
    pipettes["left"].mix(3, 15, deck["3"]["G9"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[81])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["G10"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["G10"])
    pipettes["left"].mix(3, 15, deck["3"]["G10"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[82])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["G11"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["G11"])
    pipettes["left"].mix(3, 15, deck["3"]["G11"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[83])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["G12"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["G12"])
    pipettes["left"].mix(3, 15, deck["3"]["G12"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[84])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["H1"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["H1"])
    pipettes["left"].mix(3, 15, deck["3"]["H1"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[85])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["H2"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["H2"])
    pipettes["left"].mix(3, 15, deck["3"]["H2"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[86])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["H3"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["H3"])
    pipettes["left"].mix(3, 15, deck["3"]["H3"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[87])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["H4"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["H4"])
    pipettes["left"].mix(3, 15, deck["3"]["H4"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[88])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["H5"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["H5"])
    pipettes["left"].mix(3, 15, deck["3"]["H5"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[89])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["H6"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["H6"])
    pipettes["left"].mix(3, 15, deck["3"]["H6"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[90])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["H7"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["H7"])
    pipettes["left"].mix(3, 15, deck["3"]["H7"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[91])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["H8"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["H8"])
    pipettes["left"].mix(3, 15, deck["3"]["H8"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[92])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["H9"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["H9"])
    pipettes["left"].mix(3, 15, deck["3"]["H9"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[93])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["H10"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["H10"])
    pipettes["left"].mix(3, 15, deck["3"]["H10"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[94])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["H11"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["H11"])
    pipettes["left"].mix(3, 15, deck["3"]["H11"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["7"].wells()[95])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["2"]["H12"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["H12"])
    pipettes["left"].mix(3, 15, deck["3"]["H12"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()


    # template
    pipettes["left"].pick_up_tip(deck["10"].wells()[0])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["A1"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["A1"])
    pipettes["left"].mix(3, 15, deck["3"]["A1"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[1])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["A2"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["A2"])
    pipettes["left"].mix(3, 15, deck["3"]["A2"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[2])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["A3"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["A3"])
    pipettes["left"].mix(3, 15, deck["3"]["A3"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[3])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["A4"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["A4"])
    pipettes["left"].mix(3, 15, deck["3"]["A4"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[4])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["A5"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["A5"])
    pipettes["left"].mix(3, 15, deck["3"]["A5"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[5])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["A6"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["A6"])
    pipettes["left"].mix(3, 15, deck["3"]["A6"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[6])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["A7"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["A7"])
    pipettes["left"].mix(3, 15, deck["3"]["A7"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[7])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["A8"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["A8"])
    pipettes["left"].mix(3, 15, deck["3"]["A8"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[8])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["A9"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["A9"])
    pipettes["left"].mix(3, 15, deck["3"]["A9"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[9])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["A10"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["A10"])
    pipettes["left"].mix(3, 15, deck["3"]["A10"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[10])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["A11"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["A11"])
    pipettes["left"].mix(3, 15, deck["3"]["A11"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[11])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["A12"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["A12"])
    pipettes["left"].mix(3, 15, deck["3"]["A12"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[12])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["B1"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["B1"])
    pipettes["left"].mix(3, 15, deck["3"]["B1"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[13])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["B2"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["B2"])
    pipettes["left"].mix(3, 15, deck["3"]["B2"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[14])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["B3"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["B3"])
    pipettes["left"].mix(3, 15, deck["3"]["B3"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[15])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["B4"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["B4"])
    pipettes["left"].mix(3, 15, deck["3"]["B4"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[16])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["B5"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["B5"])
    pipettes["left"].mix(3, 15, deck["3"]["B5"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[17])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["B6"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["B6"])
    pipettes["left"].mix(3, 15, deck["3"]["B6"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[18])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["B7"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["B7"])
    pipettes["left"].mix(3, 15, deck["3"]["B7"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[19])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["B8"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["B8"])
    pipettes["left"].mix(3, 15, deck["3"]["B8"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[20])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["B9"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["B9"])
    pipettes["left"].mix(3, 15, deck["3"]["B9"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[21])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["B10"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["B10"])
    pipettes["left"].mix(3, 15, deck["3"]["B10"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[22])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["B11"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["B11"])
    pipettes["left"].mix(3, 15, deck["3"]["B11"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[23])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["B12"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["B12"])
    pipettes["left"].mix(3, 15, deck["3"]["B12"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[24])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["C1"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["C1"])
    pipettes["left"].mix(3, 15, deck["3"]["C1"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[25])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["C2"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["C2"])
    pipettes["left"].mix(3, 15, deck["3"]["C2"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[26])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["C3"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["C3"])
    pipettes["left"].mix(3, 15, deck["3"]["C3"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[27])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["C4"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["C4"])
    pipettes["left"].mix(3, 15, deck["3"]["C4"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[28])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["C5"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["C5"])
    pipettes["left"].mix(3, 15, deck["3"]["C5"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[29])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["C6"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["C6"])
    pipettes["left"].mix(3, 15, deck["3"]["C6"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[30])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["C7"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["C7"])
    pipettes["left"].mix(3, 15, deck["3"]["C7"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[31])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["C8"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["C8"])
    pipettes["left"].mix(3, 15, deck["3"]["C8"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[32])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["C9"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["C9"])
    pipettes["left"].mix(3, 15, deck["3"]["C9"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[33])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["C10"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["C10"])
    pipettes["left"].mix(3, 15, deck["3"]["C10"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[34])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["C11"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["C11"])
    pipettes["left"].mix(3, 15, deck["3"]["C11"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[35])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["C12"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["C12"])
    pipettes["left"].mix(3, 15, deck["3"]["C12"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[36])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["D1"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["D1"])
    pipettes["left"].mix(3, 15, deck["3"]["D1"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[37])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["D2"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["D2"])
    pipettes["left"].mix(3, 15, deck["3"]["D2"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[38])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["D3"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["D3"])
    pipettes["left"].mix(3, 15, deck["3"]["D3"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[39])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["D4"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["D4"])
    pipettes["left"].mix(3, 15, deck["3"]["D4"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[40])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["D5"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["D5"])
    pipettes["left"].mix(3, 15, deck["3"]["D5"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[41])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["D6"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["D6"])
    pipettes["left"].mix(3, 15, deck["3"]["D6"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[42])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["D7"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["D7"])
    pipettes["left"].mix(3, 15, deck["3"]["D7"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[43])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["D8"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["D8"])
    pipettes["left"].mix(3, 15, deck["3"]["D8"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[44])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["D9"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["D9"])
    pipettes["left"].mix(3, 15, deck["3"]["D9"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[45])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["D10"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["D10"])
    pipettes["left"].mix(3, 15, deck["3"]["D10"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[46])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["D11"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["D11"])
    pipettes["left"].mix(3, 15, deck["3"]["D11"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[47])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["D12"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["D12"])
    pipettes["left"].mix(3, 15, deck["3"]["D12"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[48])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["E1"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["E1"])
    pipettes["left"].mix(3, 15, deck["3"]["E1"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[49])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["E2"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["E2"])
    pipettes["left"].mix(3, 15, deck["3"]["E2"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[50])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["E3"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["E3"])
    pipettes["left"].mix(3, 15, deck["3"]["E3"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[51])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["E4"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["E4"])
    pipettes["left"].mix(3, 15, deck["3"]["E4"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[52])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["E5"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["E5"])
    pipettes["left"].mix(3, 15, deck["3"]["E5"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[53])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["E6"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["E6"])
    pipettes["left"].mix(3, 15, deck["3"]["E6"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[54])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["E7"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["E7"])
    pipettes["left"].mix(3, 15, deck["3"]["E7"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[55])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["E8"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["E8"])
    pipettes["left"].mix(3, 15, deck["3"]["E8"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[56])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["E9"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["E9"])
    pipettes["left"].mix(3, 15, deck["3"]["E9"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[57])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["E10"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["E10"])
    pipettes["left"].mix(3, 15, deck["3"]["E10"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[58])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["E11"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["E11"])
    pipettes["left"].mix(3, 15, deck["3"]["E11"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[59])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["E12"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["E12"])
    pipettes["left"].mix(3, 15, deck["3"]["E12"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[60])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["F1"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["F1"])
    pipettes["left"].mix(3, 15, deck["3"]["F1"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[61])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["F2"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["F2"])
    pipettes["left"].mix(3, 15, deck["3"]["F2"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[62])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["F3"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["F3"])
    pipettes["left"].mix(3, 15, deck["3"]["F3"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[63])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["F4"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["F4"])
    pipettes["left"].mix(3, 15, deck["3"]["F4"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[64])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["F5"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["F5"])
    pipettes["left"].mix(3, 15, deck["3"]["F5"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[65])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["F6"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["F6"])
    pipettes["left"].mix(3, 15, deck["3"]["F6"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[66])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["F7"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["F7"])
    pipettes["left"].mix(3, 15, deck["3"]["F7"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[67])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["F8"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["F8"])
    pipettes["left"].mix(3, 15, deck["3"]["F8"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[68])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["F9"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["F9"])
    pipettes["left"].mix(3, 15, deck["3"]["F9"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[69])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["F10"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["F10"])
    pipettes["left"].mix(3, 15, deck["3"]["F10"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[70])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["F11"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["F11"])
    pipettes["left"].mix(3, 15, deck["3"]["F11"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[71])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["F12"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["F12"])
    pipettes["left"].mix(3, 15, deck["3"]["F12"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[72])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["G1"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["G1"])
    pipettes["left"].mix(3, 15, deck["3"]["G1"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[73])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["G2"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["G2"])
    pipettes["left"].mix(3, 15, deck["3"]["G2"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[74])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["G3"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["G3"])
    pipettes["left"].mix(3, 15, deck["3"]["G3"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[75])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["G4"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["G4"])
    pipettes["left"].mix(3, 15, deck["3"]["G4"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[76])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["G5"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["G5"])
    pipettes["left"].mix(3, 15, deck["3"]["G5"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[77])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["G6"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["G6"])
    pipettes["left"].mix(3, 15, deck["3"]["G6"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[78])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["G7"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["G7"])
    pipettes["left"].mix(3, 15, deck["3"]["G7"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[79])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["G8"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["G8"])
    pipettes["left"].mix(3, 15, deck["3"]["G8"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[80])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["G9"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["G9"])
    pipettes["left"].mix(3, 15, deck["3"]["G9"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[81])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["G10"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["G10"])
    pipettes["left"].mix(3, 15, deck["3"]["G10"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[82])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["G11"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["G11"])
    pipettes["left"].mix(3, 15, deck["3"]["G11"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[83])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["G12"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["G12"])
    pipettes["left"].mix(3, 15, deck["3"]["G12"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[84])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["H1"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["H1"])
    pipettes["left"].mix(3, 15, deck["3"]["H1"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[85])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["H2"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["H2"])
    pipettes["left"].mix(3, 15, deck["3"]["H2"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[86])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["H3"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["H3"])
    pipettes["left"].mix(3, 15, deck["3"]["H3"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[87])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["H4"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["H4"])
    pipettes["left"].mix(3, 15, deck["3"]["H4"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[88])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["H5"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["H5"])
    pipettes["left"].mix(3, 15, deck["3"]["H5"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[89])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["H6"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["H6"])
    pipettes["left"].mix(3, 15, deck["3"]["H6"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[90])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["H7"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["H7"])
    pipettes["left"].mix(3, 15, deck["3"]["H7"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[91])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["H8"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["H8"])
    pipettes["left"].mix(3, 15, deck["3"]["H8"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[92])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["H9"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["H9"])
    pipettes["left"].mix(3, 15, deck["3"]["H9"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[93])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["H10"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["H10"])
    pipettes["left"].mix(3, 15, deck["3"]["H10"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[94])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["H11"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["H11"])
    pipettes["left"].mix(3, 15, deck["3"]["H11"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()

    pipettes["left"].pick_up_tip(deck["10"].wells()[95])
    pipettes["left"].well_bottom_clearance.aspirate = 0.1
    pipettes["left"].aspirate(4, deck["4"]["H12"])
    pipettes["left"].well_bottom_clearance.dispense = 2.0
    pipettes["left"].dispense(4, deck["3"]["H12"])
    pipettes["left"].mix(3, 15, deck["3"]["H12"])
    pipettes["left"].blow_out()
    pipettes["left"].drop_tip()
