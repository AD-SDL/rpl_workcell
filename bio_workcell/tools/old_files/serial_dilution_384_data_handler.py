# imports
from audioop import avg
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

def handle_sd_384_data(address, json_decoded_message):

    """handle_serial_dilution_384_data

    Description: handles data processing for serial dilution (384 well plate) formatted data messages from hudson01

    Parameters: 
        address: unique message address
        message_body: utf-8 decoded message body 

    """

    # log and save the files 
    data_dir_path, file_name, plate_id, exp_name, plot_dir_path = log_and_save(address, json_decoded_message)

    file_path = os.path.join(data_dir_path, os.path.basename(file_name))

    # parse the hidex data file
    df, timestamp_list, reading_date, reading_time, data_filename = parse_hidex_sd(file_path)

    j = 0
    for i in range(4):
        # check for contaminated controls
        print(f"calling qc on {file_path}")
        qc_result = check_for_contamination(df, timestamp_list, i)  # TODO reformat to produce QC df

        # print qc result 
        print(f"done running qc on {file_name}")
        print(f"result: {qc_result}")

        if qc_result == "PASS": 

            # blank adjust the data
            blank_adj_df, blank_adj_list = od_blank_adjusted(df, timestamp_list, i)  

            # graph blank adjusted data for each timepoint
            data_basename = data_filename.split(".")[0]
            j+=1

            graph_filepaths_list = generate_graphs(blank_adj_df, data_filename, plot_dir_path,i)
            print(f"graphs generated for {data_basename}")

            if j == 3:
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
                j = 0
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


def parse_hidex_sd(file_name):

    """parses the Hidex csv file

    Description: extract the reading date, time, and data (into dataframe)

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

def check_for_contamination(raw_df, timepoint_list,i): 
    """check_for_contaminaton

    Description: checks data from all timepoints for contaminated blanks. 
        (All wells in Row o and P are blanks)

    Parameters: 
        raw_df = dataframe of raw OD(590) absorbance readings
        timepoint_list = list of timepoints at which the data was collected 
        (both parsed directly from hidex csv data)

    Returns: 
        ret_val: TODO

        o : 336 p:360
    """ 
    ret_val = "PASS"
    print("TODO: reformat serial dilution qc check")  #TEST
    if i == 0:
        blanks_o = raw_df.iloc[336:342,3:]
        blanks_p = raw_df.iloc[360:366,3:]
        numpy_blanks_o = blanks_o.to_numpy()
        flat_numpy_blanks_o = numpy_blanks_o.ravel().tolist()
        numpy_blanks_p = blanks_p.to_numpy()
        flat_numpy_blanks_p = numpy_blanks_p.ravel().tolist()
    elif i == 1:
        blanks_o = raw_df.iloc[342:348,3:]
        blanks_p = raw_df.iloc[366:372,3:]
        numpy_blanks_o = blanks_o.to_numpy()
        flat_numpy_blanks_o = numpy_blanks_o.ravel().tolist()
        numpy_blanks_p = blanks_p.to_numpy()
        flat_numpy_blanks_p = numpy_blanks_p.ravel().tolist()
    elif i == 2:
        blanks_o = raw_df.iloc[348:354,3:]
        blanks_p = raw_df.iloc[372:378,3:]
        numpy_blanks_o = blanks_o.to_numpy()
        flat_numpy_blanks_o = numpy_blanks_o.ravel().tolist()
        numpy_blanks_p = blanks_p.to_numpy()
        flat_numpy_blanks_p = numpy_blanks_p.ravel().tolist()
    elif i == 3:
        blanks_o = raw_df.iloc[354:360,3:]
        blanks_p = raw_df.iloc[378:,3:]
        numpy_blanks_o = blanks_o.to_numpy()
        flat_numpy_blanks_o = numpy_blanks_o.ravel().tolist()
        numpy_blanks_p = blanks_p.to_numpy()
        flat_numpy_blanks_p = numpy_blanks_p.ravel().tolist()
    else:
        print("ERROR: Incorrect plate quadrant given to check_for_contamination function")

    flat_numpy_blanks = np.concatenate((flat_numpy_blanks_o,flat_numpy_blanks_p))

    for blank_raw_OD in flat_numpy_blanks: 
        if float(blank_raw_OD) > 0.07: 
            ret_val == "FAIL"
            print(f"FAIL control sample {blank_raw_OD} has Raw OD value greater than 0.07")
            # TODO: improve transparency about which sample failed at what timepoint

    return ret_val

def calculate_avg(list_o, list_p):
    """calculate_avg

    Description: Average calculations of the "O" and "P" values (blanks/control wells)

    Parameters: 
        list: TODO

    Returns: 
        avg_list: TODO
    
    """
    avg_list = []
    for i in range(len(list_o)):
        avg_list.insert(i,(float(list_o[i]) + float(list_p[i]))/2)
    return avg_list

def od_blank_adjusted(data_frame, time_stamps,i):
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
        if i == 0:
            blank_adj_list= calculate_avg(list(data_frame[time_point][336:342]), (list(data_frame[time_point][360:366])))
        elif i == 1:
            blank_adj_list = calculate_avg(list(data_frame[time_point][342:348]), list(data_frame[time_point][366:372]))       
        elif i == 2:
            blank_adj_list = calculate_avg(list(data_frame[time_point][348:354]), list(data_frame[time_point][372:378]))
        elif i == 3:
            blank_adj_list = calculate_avg(list(data_frame[time_point][354:360]), list(data_frame[time_point][378:]))
        else:
            print("ERROR: Incorrect plate quadrant given to od_blank_adjusted function")
        
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

    
def generate_graphs(blank_adj_df, data_filename, plot_directory_path, quadrant): #TODO: check graph format
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
        plot_title = f"{data_filename}, timepoint = {timepoint} (seconds), quadrant = {quadrant+1}"
        plot_basename = f"{data_filename}_{str(timepoint).split('.')[0]}_quadrant:{str(quadrant)}.png"
        
        try:
            plot_file_path = os.path.join(plot_directory_path, plot_basename)
            plot_paths.append(plot_file_path)
        except OSError as e: 
            print(e)

        data_in_sets = []
        data_rep1 = []
        data_rep2 = []
        # print("data list", data_list)
        # print(data_list[:6])


        if quadrant == 0:
        # separate data into replicate 1 and 2
            for p in range(8):
                data_in_sets.append(data_list[:6])
                data_in_sets.append(data_list[24:30])
                data_rep1.append(data_list[:6])
                data_rep2.append(data_list[24:30])
                del data_list[:48]
        
        elif quadrant == 1:
            for p in range(8):
                data_in_sets.append(data_list[6:12])
                data_in_sets.append(data_list[30:36])
                data_rep1.append(data_list[6:12])
                data_rep2.append(data_list[30:36])
                del data_list[:48]
        
        elif quadrant == 2:
            for p in range(8):
                data_in_sets.append(data_list[12:18])
                data_in_sets.append(data_list[36:42])
                data_rep1.append(data_list[12:18])
                data_rep2.append(data_list[36:42])
                del data_list[:48]
        
        elif quadrant == 3:
            for p in range(8):
                data_in_sets.append(data_list[18:24])
                data_in_sets.append(data_list[42:48])
                data_rep1.append(data_list[18:24])
                data_rep2.append(data_list[42:48])
                del data_list[:48]




        plt.figure(figsize=(10, 5))

        # plot each replicate separately (to maintain color across replicates)
        for i in range(len(data_rep1)): 
            plt.plot(x_rep1, data_rep1[i], colors[i])
        for i in range(len(data_rep2)): 
            plt.plot(x_rep2, data_rep2[i], colors[i])

        plt.legend(["Row A and B","Row C and D","Row E and F","Row G and H", "Row I and J", "Row K and L", "Row M and N", "Row O and P"], loc='center left', bbox_to_anchor=(1, 0.5), borderaxespad=0.)  #TODO: extract strain names from db
        plt.title(plot_title)
        plt.xticks(x_ticks)
        plt.xlabel("Dilution columns with highest to lowest concentration from left to right with last column being control")
        plt.ylabel("Blank-adjusted OD(590)")
    
        # save figure to file
        plt.savefig(plot_file_path)
        plt.close() 

    return plot_paths


def main(json_string):
    lambda6_handle_message(json_string)
    

if __name__ == "__main__":
    #execute only if run as a script
    if os.path.isfile(sys.argv[1]):
        with open(sys.argv[1], "r") as file:
            json_string = file.read()
    else:
        json_string = sys.argv[1]

    main(json_string)
