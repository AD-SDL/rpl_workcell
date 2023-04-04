import os
import sys
import json

def handle_dna_assembly_data(address, json_decoded_message): 
    print(f"handling dna assembly data {address}")

    # log the message and save to files 
    data_dir_path, file_name, plate_id, exp_name, plot_dir_path = log_and_save(address, json_decoded_message)
    print(f"done logging and saving {file_name}")

    file_path = os.path.join(data_dir_path, os.path.basename(file_name))

    # parse hidex data csv file 
    parse_hidex_dna_assembly(file_path)
    print("done parsing dna assembly data")

    # add data to DB
        # TODO
        
    
    # check for contaminated controls
        #run_qc(DATA_FRAME)

    # print qc result 
        #TODO

    # insert qc results into DB
        #TODO 
    
    # if qc == pass: 
        # blank adjust whatever data necessay

        # add blank adjusted data to db

        # graph blank adjusted data 

        # add graphs into db 

        # send message to next lambda6 listener

    # else: 
        # print(qc failed message)
    print("DONE handling dna assembly data")



def log_and_save(address, json_decoded_message):  #TODO reformat code, this method same as log and save in campaign2_data_handler.py
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

            
    

def parse_hidex_dna_assembly(data_filename): 
    print("TODO: parse_data_csv")

def run_qc(data_df): 
    print("TODO: run_qc")


def generate_graphs(data_df): 
    print("TODO: generate graphs")



def main(address, message_body):
    handle_dna_assembly_data(address,message_body)

if __name__ == "__main__":
    #execute only if run as a script # FIX THIS
    address = sys.argv[1]
    message_body = sys.argv[2]
    # if os.path.isfile(sys.argv[1]):
    #     with open(sys.argv[1], "r") as file:
    #         json_string = file.read()
    # else:
    #     json_string = sys.argv[1]

    main(address, message_body)