from gladier import generate_flow_definition, GladierBaseTool


def Tecan_Proc(**data):
    """
    Description: Reading a csv file with the Absorption spectra and converting to the Lab values

    Parameters:
        file_name: the complete path and name of the csv file with the absorption spectra
        the dataframe includes a column named 'wavelength' and columns with the abs spectra named after the
        positions on the plate reader, e.g. 'C3', 'C4' etc.

    Returns:
        df: a pandas dataframe with the Lab color coordicates
    """

    import pandas as pd
    import colour  # requires to install the Corol library: pip install colour

    file_name = data.get("csv_name")

    lab_list = []

    for col in file_name.columns.values[1:]:
        Trans = 10 ** (2 - file_name[col].values)
        data_sample = dict(zip(file_name["wavelength"], Trans / 100))
        sd = colour.SpectralDistribution(data_sample)
        cmfs = colour.MSDS_CMFS["CIE 1931 2 Degree Standard Observer"]
        illuminant = colour.SDS_ILLUMINANTS["D65"]
        XYZ = colour.sd_to_XYZ(sd, cmfs, illuminant)
        Lab = colour.XYZ_to_Lab(XYZ / 100)
        lab_list.append(Lab)
    df = pd.DataFrame(
        file_name["wavelength"],
        pd.DataFrame(lab_list, columns=file_name.columns.values[1:]),
    )
    return df


@generate_flow_definition
class Tecan_Proc(GladierBaseTool):
    compute_functions = [Tecan_Proc]
    required_input = ["compute_endpoint"]
