#!/usr/bin/env python3
from pathlib import Path
from rpl_wei import Experiment
import json
import time
from time import sleep
from ml_loop.ml_modules import *
from ml_loop.ml_utils import *
from ml_loop.ml_utils import tecan_proc, tecan_read
from ml_loop.set_transfomer import SmallSetTransformer_v2
from tools.globus_batman2chemspeed import *
from tools.flow_tecan import tecan_flow
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import os
import csv

def iter_run(init_file, iteration):
        
    """Actions to perform to run one full experimental loop:
    The experiment starts when the user decides on the color,i.e., Lab values, of the polymer they want to synthezise 
    and setting the starting monomers they have in the inventory
    """

    # Read the init.json to extract the user provided information and store the metadata
    with open(init_file, 'r') as f:
         init_data = json.load(f)

    # 3. Call the pretrained ML model , ML precidt the L,a,b values of the provided file
    device ='cpu'
    epochs, learning_rate, batch_size, Ndims = 100, 1e-3, 12, 1056 
    dropout_ratio = 0.2  # replace with your desired value
    model=SmallSetTransformer_v2(dropout_ratio, device, epochs, learning_rate, batch_size)

    # load the model which was pretrained on the literature data + inhouse samples
    ckpt = torch.load(os.path.join('/home/rpl/workspace/polybot_workcell/polybot_app/ml_loop/set_transformer_checkpoints', f'set_transformer_dft_ecfp_2.tar')) #
    model.load_state_dict(ckpt['state_dict'])

    # 4. Use the Set transformer model to get all the predictions
    all_possibilities_avail = all_possibilities[all_possibilities.exclude==0]
 
    lab_scaler = joblib.load('/home/rpl/workspace/polybot_workcell/polybot_app/ml_loop/scaler.gz')
    X_valid= all_possibilities_avail.iloc[:,3:-1]
    pred_data_1, pred_data_2,pred_data_3, y_target = np.array(X_valid.iloc[:, :Ndims].values, dtype=np.float16),  np.array(X_valid.iloc[:, Ndims:2*Ndims].values, dtype=np.float16), np.array(X_valid.iloc[:, 2*Ndims:].values, dtype=np.float16), np.zeros(X_valid.shape[0])
    preds, uncertainties = model.test_model(pred_data_1, pred_data_2,pred_data_3, y_target)
    
  
    preds_inv_scaled = lab_scaler.inverse_transform(preds)
    target_Lab = init_data["target_Lab"]
    preds_df = pd.concat([pd.DataFrame(all_possibilities_avail['smiles1'].values, columns=["smiles1"]),
                     pd.DataFrame(all_possibilities_avail['percentage_1'].values, columns=["percentage_1"]),
                     pd.DataFrame(all_possibilities_avail['smiles2'].values, columns=["smiles2"]),
                     pd.DataFrame(all_possibilities_avail['percentage_2'].values, columns=["percentage_2"]),
                     pd.DataFrame(all_possibilities_avail['smiles3'].values, columns=["smiles3"]),
                     pd.DataFrame(all_possibilities_avail['percentage_3'].values, columns=["percentage_3"]),
                     pd.DataFrame(preds_inv_scaled, columns=["L", "a", "b"]) ], axis=1)

    # 5. Select: get the 6 top candidates to run in Chemspeed: create the correct format and send it to chemspeed, file named 
    # according to the experimental iteration
    top_candidates = select_next_exp_ml(preds_df, target_Lab, uncertainties, iteration) # print out the first monomer smiles + ratio as well
    print('top_candidates', top_candidates)

    # # # # update init.json with the top 6 selections and the predicted values    
    init_data[f"iteration_{iteration+1}"]["ml_suggestions"] = top_candidates.iloc[: , :-3].to_dict()
    init_data[f"iteration_{iteration+1}"]["predicted_lab"] = top_candidates.iloc[: , -3:].to_dict()


    for i in list(all_possibilities_avail.iloc[list(top_candidates.index)].index):
        all_possibilities.loc[i, 'exclude'] = 1

    # all_possibilities.to_csv(f"all_possibilities_{iteration+1}_loop.csv", index=None)
    df_pivot = convert_for_chemspeed(top_candidates.iloc[: , :-3])
    print('df_pivot', df_pivot)
    df_pivot.to_csv(f"/home/rpl/workspace/polybot_workcell/polybot_app/demo_files/for_chemspeed/loop_{iteration+1}.csv", index=False)
    
    # # 5. Call the globus data transfer to send the file from the local folder to Chemspeed
    local_path = Path("/home/rpl/workspace/polybot_workcell/polybot_app/demo_files/for_chemspeed")
    fname_chemspeed =  f"loop_{iteration+1}.csv" 

    # # batman2chemspeed_flow(local_path = str(local_path), fname = fname_chemspeed)

    # # 6. Protocol files to run the experiments 
    chem_speed_fname = f"C:/Users/Operator/Desktop/Yukun/Workflow/Electrochromic Synthesis/Closeloop/Closeloop-{iteration+1}_actual.app" # _actual
    tecan_filename= f"C:/Users/Public/Documents/Tecan/Magellan Pro/mth/ECP-350-800-Yukun-{iteration+1}.mth"

    local_path_tecan = Path("C:/Users/cnmuser/Desktop/Polybot/tecan_code/uv_vis_data")
    local_path_from_tecan = "/home/rpl/workspace/polybot_workcell/polybot_app/demo_files/from_tecan"
    fname_tecan = f"ECP_demo_batch_{iteration}.asc" 

    flow_status = polybot_wei_workflow(
        file_name =chem_speed_fname,
        tecan_file_name=tecan_filename, 
        loop_file_path=str(local_path / fname_chemspeed),
        tecan_file_path=str(local_path_tecan / fname_tecan),
        tecan_iteration = iteration
    )
    
    # tecan_flow(local_path_from_tecan , fname_tecan)

    # Read the data file from tecan
    filename=os.path.join(local_path_from_tecan, fname_tecan)
    df = tecan_read(filename)
    print('df', df)
    
    # Check the results of the absorption spectra measurements
    peak_check_result = peak_check(df)
    
    if 1 in peak_check_result or -1 in peak_check_result:
        print('Entering dilutuion loop')
        fname_rerun_chemspeed = f"rerun_{iteration+1}.csv"
        with open(str(local_path / fname_rerun_chemspeed), 'w') as f:
            write=csv.writer(f)
            write.writerow(peak_check_result)

        # Create filename for Chemspeed rerun
        rerun_chem_speed_fname = f"C:/Users/Operator/Desktop/Yukun/Workflow/Electrochromic Synthesis/Closeloop/Closeloop-{iteration+1}_rerun_actual.app"
       
        # Create filename for Tecan rerun
        tecan_filename_rerun= f"C:/Users/Public/Documents/Tecan/Magellan Pro/mth/ECP-350-800-Yukun-{iteration+1}_rerun.mth"
        
        flow_status = polybot_wei_workflow(
        file_name = rerun_chem_speed_fname,
        tecan_file_name=tecan_filename_rerun, 
        loop_file_path=str(local_path / fname_rerun_chemspeed),
        tecan_file_path=str(local_path_tecan / fname_tecan),
        tecan_iteration = iteration + 4 )
    
        # Read the data file from Tecan (assuming local_path_from_tecan and fname_tecan are defined)
        filename = os.path.join(local_path_from_tecan, fname_tecan)
        df = tecan_read(filename) # updates the file with the Tecan measurements to be used further
   
    init_data[f"iteration_{iteration+1}"]["uv_vis_data"] = pd.read_csv(df).to_dict()
    lab_values = tecan_proc(df)
    print('lab_values',lab_values)

    # Apply stopping criteria: measure the distance of the samples to the desired Lab value, if distance < 10 then stop
    if get_lab_distance(target_Lab, lab_values) == True:
        print("Target color achieved")
        # exit()
         
    # Apply a quality check on the data 
    peak_check_result = peak_check(df)
    quality_check_idx = [index for index, value in enumerate(peak_check_result) if value in [1,-1]]    
    top_candidates_updated =   top_candidates.reset_index().drop(quality_check_idx).reset_index()
    print('top_candidates_updated', top_candidates_updated)
    lab_values_updated = [lab_values[i] for i in range(len(lab_values)) if i not in quality_check_idx]
    print('lab_values', lab_values_updated )
    lab_values = lab_values_updated

    # 10. Retrain the ML model and save the new weights to the checkpoints folder  
    X_train, y_train = get_train_data_representation_dft(top_candidates.reset_index()) , lab_values
    train_data_1, train_data_2,train_data_3, y_train = np.array(X_train.iloc[:, :Ndims].values, dtype=np.float16),  np.array(X_train.iloc[:, Ndims:2*Ndims].values, dtype=np.float16), np.array(X_train.iloc[:, 2*Ndims:].values, dtype=np.float16), np.array(y_train, dtype=np.float16)
    y_train = pd.DataFrame(y_train, columns=['L', 'a', 'b'])
    init_data[f"iteration_{iteration+1}"]["measured_lab"] = y_train.to_dict()
    my_scaler = joblib.load('/home/rpl/workspace/polybot_workcell/polybot_app/ml_loop/scaler.gz')
    y_train = my_scaler.transform(y_train.values)\
    
    model.train_model(train_data_1, train_data_2,train_data_3, y_train)    
    torch.save({'state_dict':model.state_dict()}, os.path.join('/home/rpl/workspace/polybot_workcell/polybot_app/ml_loop/set_transformer_checkpoints', f"set_transformer_dft_ecfp_3.tar"))
    
    # At the end of each loop return the updated init.json file
    with open('/home/rpl/workspace/polybot_workcell/polybot_app/demo_files/init.json', 'w', encoding='utf-8') as f:
            json.dump(init_data, f, ensure_ascii=False, indent=4)


def polybot_wei_workflow(file_name = None, tecan_file_name=None, loop_file_path=None, tecan_file_path=None, tecan_iteration=None): # we can add the iteration here and read the yaml based on the iteration we are

    """Calls the WEI2 process:
     1. Opens the Chemspeed
     2. Run the Chemspeed experiment
     3. Once the experiment finish, then call UR5e arm to grab the plate from Chemspeed and move it to Tecan.
     4. Wait for the Tecan measurement to finish and then returns the plate back to Chemspeed
     """
    
    wf_path = Path("/home/rpl/workspace/polybot_workcell/polybot_app/workflows/demo.yaml")
    exp = Experiment("127.0.0.1", "8000", "CNM Experiment")

    if not file_name:
        file_name = "C:/Users/Operator/Desktop/Yukun/Workflow/Electrochromic Synthesis/Closeloop/Closeloop-1.app"

    if not tecan_file_name:
        tecan_file_name = "C:/Users/Public/Documents/Tecan/Magellan Pro/mth/ECP-350-800-Yukun.mth"

    payload={
        "chemspeed_protocol": file_name,
        "loop_file_path":loop_file_path,
        "tecan_protocol": tecan_file_name,
        "tecan_file_path": tecan_file_path,
        "tecan_iteration":tecan_iteration
        }
    
    flow_info = exp.run_job(wf_path.resolve(), simulate=False, payload = payload)
    print('flow_info', flow_info)
    flow_status = exp.query_job(flow_info["job_id"])
    print('flow_status',flow_status)
    while flow_status["status"] != "finished":
        flow_status = exp.query_job(flow_info["job_id"])
        print(flow_status)
        sleep(1)


class MyHandler(FileSystemEventHandler):
    def __init__(self, init_file, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.init_file = init_file
        self.iteration = 0

    def on_created(self, event):
        print(f'New file - {event.src_path} - has been created!')
        self.iteration += 1
        iter_run(self.init_file, self.iteration)


# if __name__ == "__main__":
#     # 1. Read the initial file with the user defined information
#     init_file = '/home/rpl/workspace/polybot_workcell/polybot_app/demo_files/init.json'
#     with open(init_file, 'r') as f:
#         init_data = json.load(f)

#     # 2. Given the initial file with the inventory generate all the possible combinations
#     inventory = init_data["monomers"]
#     num_monomers = init_data["number_of_monomers"]
#     ratio_step = init_data["ratio_step"]

#     all_possibilities = create_predictions_dataset_dft(inventory, num_monomers, ratio_step) #, number if monomers and step 
#     all_possibilities = all_possibilities.drop_duplicates(subset = ['smiles1', 'percentage_1', 'smiles2', 'percentage_2']).reset_index(drop=True)
#     all_possibilities['exclude'] = 0
#     all_possibilities = all_possibilities.reset_index(drop=True)
#     # all_possibilities=pd.read_csv('/home/rpl/workspace/all_possibilities_2_loop.csv') #if one loop is stoped and we want to continue from where we left, we read the all_possibilities dataframe
#     event_handler = MyHandler(init_file)
#     observer = Observer()
#     # The observer is based on the updates of the ML checkpoints
#     observer.schedule(event_handler, path='/home/rpl/workspace/polybot_workcell/polybot_app/ml_loop/set_transformer_checkpoints', recursive=False)
#     observer.start()
#     iter_run(init_file, 0) # Run first iteration of iter_run before starting to observe   
#     try:
#         while True:
#             time.sleep(4)
#     except KeyboardInterrupt:
#         observer.stop()
#         observer.join()


init_file = '/home/rpl/workspace/polybot_workcell/polybot_app/demo_files/init.json'

with open(init_file, 'r') as f:
    init_data = json.load(f)

# 2. Given the initial file with the inventory generate all the possible combinations
inventory = init_data["monomers"]
num_monomers = init_data["number_of_monomers"]
ratio_step = init_data["ratio_step"]

# all_possibilities = create_predictions_dataset_dft(inventory, num_monomers, ratio_step) #, number if monomers and step 
# all_possibilities = all_possibilities.drop_duplicates(subset = ['smiles1', 'percentage_1', 'smiles2', 'percentage_2']).reset_index(drop=True)
# all_possibilities['exclude'] = 0
# all_possibilities = all_possibilities.reset_index(drop=True)
all_possibilities = pd.read_csv('updated_csv1.csv') #('/home/rpl/workspace/all_possibilities_2_loop.csv') 
iter_run(init_file, 0)