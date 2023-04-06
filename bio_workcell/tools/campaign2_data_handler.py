import os
import sys
import csv 
import json
import pandas as pd
import numpy as np
from utils.run_qc import run_qc
from utils.zmq_connection import zmq_connect
import matplotlib.pyplot as plt
sys.path.append("../rdbms/")
sys.path.append("../../rdbms/") # this is the one that works
from database_functions import update_plate_data, insert_control_qc, insert_blank_adj


def handle_campaign2_data(address, json_decoded_message):
    """handle_campaign2_data

    Description: handles data processing for campaign2 formatted data messages from hudson01

    Parameters: 
        address: unique message address
        message_body: utf-8 decoded message body 

    """

    # log and save the files 
    data_dir_path, file_name, plate_id, exp_name, plot_dir_path = log_and_save(address, json_decoded_message)

    file_path = os.path.join(data_dir_path, os.path.basename(file_name))

    # parse the hidex data file
    df, timestamp_list, reading_date, reading_time, data_filename = parse_hidex_campaign2(file_path)

    # Add data to db 
    update_plate_data(exp_name, plate_id, timestamp_list, df, reading_date, reading_time, data_filename)

    # check for contaminated controls
    print(f"calling qc on {file_path}")
    qc_result = check_for_contamination(df, timestamp_list)  # TODO reformat to produce QC df

    # print qc result 
    print(f"done running qc on {file_name}")
    print(f"result: {qc_result}")

    # insert qc results into DB  # TESTING # TODO pass QC df to db  
    insert_control_qc(exp_name, plate_id, qc_result) 

    if qc_result == "PASS": 

        # blank adjust the data
        blank_adj_df, blank_adj_list = od_blank_adjusted(df, timestamp_list)  

        #insert blank adjusted data into db # TESTING
        insert_blank_adj(exp_name, plate_id, blank_adj_list)
        #print("INSERTED BLANK ADJ DATA INTO TABLE")
        
        # graph blank adjusted data for each timepoint
        data_basename = data_filename.split(".")[0]
        graph_filepaths_list = generate_graphs(blank_adj_df, data_filename, plot_dir_path)  
        print(f"graphs generated for {data_basename}")

        print(f"Done handling message: {str(address)}")

        # send message to build_dataframe if the data is good 
        context, socket = zmq_connect(port=5556, pattern="REQ")
        basename = os.path.basename(file_path)
        print("got basename {} for filename {}".format(basename, file_path))
        message = {  # TODO: might not need to pass to build df becuase df already generated in previous step
            basename: {
                "path": [file_path],
                "purpose": ["build_dataframe"],
                "type": ["JSON"],
            }
        }
        socket.send_string(json.dumps(message))  
        repl = socket.recv()
        print(f"Got {repl}")
    else:
        print(f"qc failed on {file_name}")


def log_and_save(address, json_decoded_message): 
    """log_and_save

    Description: Record message in message logs and save copy of data to files on lambda6

    Parameters: 
        address: unique message address
        json_decoded_message: decoded contents of the mesage 

    Returns: 
        data_dir_path: path to directory on lambda6 where data file was saved
        file_name: name of the data file
        plate_id: ID of the plate used to generate the data
        exp_name: name of the experiment which generated the data
        plot_dir_path: path to directory created to hold generated graphs


    """

    lambda6_data_path = "/lambda_stor/data/hudson/data/"

    # # * extract message address and body
    # address, message_body = decoded_message.split("***")
    # json_decoded = json.loads(message_body)

    # return_val = "PASS"
    #print(f"Handling message: {str(address)}")

    # * assign path names (on lambda6 or running locally for testing?)
    if os.path.exists(lambda6_data_path):
        # format log and data directory paths
        log_dir_path = os.path.join(lambda6_data_path, "log/")
        data_dir_path = os.path.join(lambda6_data_path, str(address) + "/")

        # * record in message_log.txt
        if not os.path.exists(os.path.dirname(log_dir_path)):
            try:
                os.makedirs(os.path.dirname(log_dir_path))
            except OSError as e:
                print("Failed to create directory-> " + str(log_dir_path))
                raise

        # write to log file if directory exists/was created
        if os.path.exists(os.path.dirname(log_dir_path)):
            with open(
                os.path.join(log_dir_path, "message_log.txt"), "a+"
            ) as message_log:
                message_log.writelines(
                    address + "-R\n"
                )  # address = {timestamp}-{numFiles}-{R(received) or S(sent)}
                for key, value in json_decoded_message.items():
                    message_log.writelines("\t" + str(key) + "\n")  # filenames

        # * write data to files
        # create a folder to store data (same name as in log file)
        if not os.path.exists(os.path.dirname(data_dir_path)):
            try:
                os.makedirs(os.path.dirname(data_dir_path))
            except OSError as e:
                print("Failed to create directory -> " + str(data_dir_path))
                raise

        # write data contents to files within folder
        if os.path.exists(os.path.dirname(data_dir_path)):
            for key, value in json_decoded_message.items():
                file_name = key
                data = value["data"]
                plate_id = value["plate_id"]
                exp_name = value["experiment_name"]
                data_format = value["data_format"]
                with open(
                    os.path.join(data_dir_path, os.path.basename(file_name)), "w+"
                ) as data_file:
                    data_file.writelines(data)

                # write info file inside the same folder
                with open(
                    os.path.join(data_dir_path, "info.txt"), "w+"
                ) as info_file:
                    info_file.write(f"Plate ID: {plate_id}\n")
                    info_file.write(f"Experiment Name: {exp_name}\n")  

                # create a folder to store graphs
                plot_dir_path = os.path.join(data_dir_path, "graphs/")
                try: 
                    os.makedirs(os.path.dirname(plot_dir_path))
                    print(f"created plot directory: {plot_dir_path}")
                except OSError as e: 
                    print(f"FAILED to create directory: {plot_dir_path}")
                    raise

    return data_dir_path, file_name, plate_id, exp_name, plot_dir_path

def parse_hidex_campaign2(file_name):
    """parses the Hidex csv file

    Description: TODO

    Parameters:
        file_name: the complete path and name of the Hidex csv file

    Returns:
        df: a pandas data frame


    """
    df = pd.DataFrame()

    # extract the reading date, time, and data (into dataframe)
    DATA = False
    with open(file_name, newline="") as csvfile:
        print(f"opened {file_name}")
        csv.QUOTE_NONNUMERIC = True
        reader = csv.reader(csvfile)
        i = 0
        for row in reader:
            i += 1
            row = [x.strip() for x in row]
            if i == 3: 
                reading_date, reading_time = row[0].split(" ")
            if len(row) > 0 and row[0] == "Plate #":
                df = pd.DataFrame(columns=row)
                DATA = True
                continue
            if DATA == True:
                df.loc[len(df.index) + 1] = row

    timestamp_list = df.columns[3:].to_list()

    # extract file basename 
    basename = os.path.basename(file_name)

    return df, timestamp_list, reading_date, reading_time, basename 



def check_for_contamination(raw_df, timepoint_list):  # TODO
    """check_for_contaminaton

    Description: checks data from all timepoints for contaminated blanks. 
        (All wells in Row H are blanks)

    Parameters: 
        raw_df = dataframe of raw OD(590) absorbance readings
        timepoint_list = list of timepoints at which the data was collected 
        (both parsed directly from hidex csv data)

    Returns: 
        ret_val: TODO
    """ 
    
    ret_val = "PASS"
    print("TODO: reformat campaign2 qc check")  #TEST

    blanks = raw_df.iloc[84:,3:]
    numpy_blanks = blanks.to_numpy()
    flat_numpy_blanks = numpy_blanks.ravel().tolist()
    
    for blank_raw_OD in flat_numpy_blanks: 
        if float(blank_raw_OD) > 0.07: 
            ret_val == "FAIL"
            print(f"FAIL control sample {blank_raw_OD} has Raw OD value greater than 0.07")
            # TODO: improve transparency about which sample failed at what timepoint

    return ret_val


def calculate_avg(list):
    """calculate_avg

    Description: Average calculations of the "H" values (blanks/control wells)

    Parameters: 
        list: TODO

    Returns: 
        avg_list: TODO
    
    """
    avg_list = []
    pointer1 = 0
    pointer2 = 6

    while pointer1 < 6:
        
        avg_list.insert(pointer1,(float(list[pointer1]) + float(list[pointer2]))/2)
        avg_list.insert(pointer2,(float(list[pointer1]) + float(list[pointer2]))/2)
        pointer1+=1
        pointer2+=1
    
    return avg_list



def od_blank_adjusted(data_frame, time_stamps):
    """od_blank_adjusted

        Description: Recives a data_frame and calculates blank adjusted values for each data point

        Parameters: 
            data_frame: Data itself
            time_stamps: A list that contains the time points
        
        Returns: 
            - blank_adj_data_frame: A new data frame with blank adjusted values
            - adjusted_values_list: A list of blank adjusted values (this will be used to insert the values into the database)
    """
    blank_adj_data_frame = data_frame
    adjusted_values_list = []
    for time_point in time_stamps:
    
        blank_adj_list = calculate_avg(list(data_frame[time_point][84:]))
        index = 0
        
        for data_index_num in range(1,len(data_frame)+1) :
            A = float(data_frame[time_point][data_index_num])
            if index == len(blank_adj_list):
                index = 0
            adjust = round(float(data_frame[time_point][data_index_num]) - blank_adj_list[index], 3)
            if adjust < 0:
                adjust = 0
            
            blank_adj_data_frame[time_point][data_index_num] = adjust
            adjusted_values_list.append(adjust)
            index+=1
    
    return blank_adj_data_frame, adjusted_values_list


# METHOD REMOVED FOR TESTING
def generate_graphs(blank_adj_df, data_filename, plot_directory_path): #TODO: check graph format
    """generate_graphs

    Description: Received a blank adjusted data frame and produces one graph per data timepoint

    Parameters: 
        blank_adj_df: blank adjusted data in data frame format
        data_filename: file name of the original data csv file without extension (not file path)
        plot_directory_path: path to the directory where all created graphs should be stored 

    Returns: 
        plot_paths: a list of file paths to newly saved graphs in order of increasing data timepoint
    
    """
    plot_paths = []

    # define x-axis and x-axis ticks
    x_axis = []
    x_rep1 = [1,2,3,4,5,6]
    x_rep2 = [7,8,9,10,11,12]

    x_ticks = x_rep1.copy()
    for x in x_rep2:
        x_ticks.append(x)

    for i in range(8): 
        x_axis.append(x_rep1)
        x_axis.append(x_rep2)
    
    # define plot colors and figure dimensions
    colors = ["b","g","r","c","m","y",'tab:brown', "k"]

    # Graph data from each timepoint
    for timepoint in blank_adj_df.columns[3:]:
        data_list = blank_adj_df[timepoint].tolist()
        data_list = [float(x) for x in data_list]

        # determine plot title and file path
        plot_title = f"{data_filename}, timepoint = {timepoint} (seconds)"
        plot_basename = f"{data_filename}_{str(timepoint).split('.')[0]}.png"
        
        try:
            plot_file_path = os.path.join(plot_directory_path, plot_basename)
            plot_paths.append(plot_file_path)
        except OSError as e: 
            print(e)

        data_in_sets = []
        data_rep1 = []
        data_rep2 = []

        # separate data into replicate 1 and 2
        while not len(data_list) == 0:  
            data_in_sets.append(data_list[:6])
            data_in_sets.append(data_list[6:12])
            data_rep1.append(data_list[:6])
            data_rep2.append(data_list[6:12])
            data_list = data_list[12:]

        plt.figure(figsize=(10, 5))

        # plot each replicate separately (to maintain color across replicates)
        for i in range(len(data_rep1)): 
            plt.plot(x_rep1, data_rep1[i], colors[i])
        for i in range(len(data_rep2)): 
            plt.plot(x_rep2, data_rep2[i], colors[i])

        plt.legend(["Row A","Row B","Row C","Row D","Row E","Row F","Row G","Row H"], loc='center left', bbox_to_anchor=(1, 0.5), borderaxespad=0.)  #TODO: extract strain names from db
        plt.title(plot_title)
        plt.xticks(x_ticks)
        plt.xlabel("Dilution (1-6) and (7-12) are replicates. 1 is most concentrated, 6 is no treatment")
        plt.ylabel("Blank-adjusted OD(590)")
    
        # save figure to file
        plt.savefig(plot_file_path)
        plt.close() 

    return plot_paths


def main(json_string):
    lambda6_handle_message(json_string)  # TODO 
    

if __name__ == "__main__":
    #execute only if run as a script
    if os.path.isfile(sys.argv[1]):
        with open(sys.argv[1], "r") as file:
            json_string = file.read()
    else:
        json_string = sys.argv[1]

    main(json_string)
    
 