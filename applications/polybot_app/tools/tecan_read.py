from gladier import generate_flow_definition, GladierBaseTool


def tecan_read(**data):
    """
    Extracts raw data from Mangelan asc file into a csv file

    :param str filename: filename of Mangelan asc file to convert

    output: path of new csv file (str)

    """
    import os
    import csv

    print(data["input"]["proc_folder"])
    delimiter = "\t"
    data = {"wavelength": []}
    column_names = set()
    filename = data.get("proc_folder") + "/" + data.get("fname")

    with open(filename, "r", encoding="utf-16-le") as input_file:
        for line in input_file:
            print(line)
            line = line.strip()

            if line.startswith("*"):
                fields = line[2:].replace("nm", "").split(delimiter)
                data["wavelength"].extend(fields)
            elif line.startswith("C"):
                columns = line.split(delimiter)
                column_name = columns[0]
                column_values = columns[1:]
                data[column_name] = column_values
                column_names.add(column_name)

    if os.path.exists(filename):
        asc_basename = os.path.splitext(os.path.basename(filename))[0]
        csv_filename = asc_basename + ".csv"
        csv_filepath = filename.replace(os.path.basename(filename), csv_filename)

    with open(csv_filepath, "w", newline="") as output_file:
        csv_writer = csv.writer(output_file)
        csv_writer.writerow(["wavelength"] + sorted(column_names))
        num_rows = max(
            len(data["wavelength"]),
            max(len(values) for values in data.values() if isinstance(values, list)),
        )
        for i in range(num_rows):
            row = [
                data[column][i] if i < len(data[column]) else ""
                for column in ["wavelength"] + sorted(column_names)
            ]
            csv_writer.writerow(row)

    return csv_filepath


@generate_flow_definition()
class Tecan_Read(GladierBaseTool):
    compute_functions = [tecan_read]
    required_input = ["compute_endpoint"]
