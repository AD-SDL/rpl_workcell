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


# tested_candidates = pd.read_csv('/home/rpl/workspace/polybot_workcell/all_results.csv')
# # print(tested_candidates)
# iteration=0

# # 3. Call the pretrained ML model , ML precidt the L,a,b values of the provided file
# device ='cpu'
# epochs, learning_rate, batch_size, Ndims = 100, 1e-3, 12, 1056 
# dropout_ratio = 0.2  # replace with your desired value
# model=SmallSetTransformer_v2(dropout_ratio, device, epochs, learning_rate, batch_size)

# # load the model which was pretrained on the literature data + inhouse samples
# ckpt = torch.load(os.path.join('/home/rpl/workspace/polybot_workcell/polybot_app/ml_loop/set_transformer_checkpoints', f'set_transformer_dft_ecfp_{iteration}.tar'))
# model.load_state_dict(ckpt['state_dict'])

# local_path_from_tecan = "/home/rpl/workspace/polybot_workcell/polybot_app/demo_files/from_tecan"
# fname_tecan = f"ECP_demo_batch_{iteration}.asc" 
# filename=os.path.join(local_path_from_tecan, fname_tecan)

# # df = tecan_read(filename)
# # lab_values = tecan_proc(df)
# # print('lab_values',lab_values)
# lab_values = tested_candidates[['L', 'a', 'b']]


# y_train =  lab_values
# y_train =  np.array(y_train, dtype=np.float16)
# # 10. Retrain the ML model and save the new weights to the checkpoints folder  
# X_train, y_train = get_train_data_representation_dft(tested_candidates) , lab_values
# print(y_train)
# train_data_1, train_data_2,train_data_3, y_train = np.array(X_train.iloc[:, :Ndims].values, dtype=np.float16),  np.array(X_train.iloc[:, Ndims:2*Ndims].values, dtype=np.float16), np.array(X_train.iloc[:, 2*Ndims:].values, dtype=np.float16), np.array(y_train, dtype=np.float16)
# y_train = pd.DataFrame(y_train, columns=['L', 'a', 'b'])

# my_scaler = joblib.load('/home/rpl/workspace/polybot_workcell/polybot_app/ml_loop/scaler.gz')
# y_train = my_scaler.transform(y_train.values)\

# model.train_model(train_data_1, train_data_2,train_data_3, y_train)    
# torch.save({'state_dict':model.state_dict()}, os.path.join('/home/rpl/workspace/polybot_workcell/polybot_app/ml_loop/set_transformer_checkpoints', f"set_transformer_dft_ecfp_2.tar"))
    




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
    ckpt = torch.load(os.path.join('/home/rpl/workspace/polybot_workcell/polybot_app/ml_loop/set_transformer_checkpoints', f'set_transformer_dft_ecfp_{iteration}.tar')) #
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




init_file = '/home/rpl/workspace/polybot_workcell/polybot_app/demo_files/init.json'

with open(init_file, 'r') as f:
    init_data = json.load(f)

# 2. Given the initial file with the inventory generate all the possible combinations
inventory = init_data["monomers"]
num_monomers = init_data["number_of_monomers"]
ratio_step = init_data["ratio_step"]

all_possibilities = create_predictions_dataset_dft(inventory, num_monomers, ratio_step) #, number if monomers and step 
all_possibilities = all_possibilities.drop_duplicates(subset = ['smiles1', 'percentage_1', 'smiles2', 'percentage_2']).reset_index(drop=True)
all_possibilities['exclude'] = 0
all_possibilities = all_possibilities.reset_index(drop=True)
all_possibilities=pd.read_csv('updated_csv1.csv') 
iter_run(init_file, 2)


# all_possibilities = pd.read_csv('/home/rpl/workspace/polybot_workcell/all_possibilities.csv')
# tested_candidates = pd.read_csv('/home/rpl/workspace/polybot_workcell/all_results.csv')
# columns_to_compare =['smiles1', 'percentage_1', 'smiles2', 'percentage_2', 'smiles3', 'percentage_3']

# merged = all_possibilities.merge(tested_candidates[columns_to_compare], on=columns_to_compare, how='left', indicator=True)
# # print(all_possibilities.columns.values)
# merged['exclude'] = merged['_merge'].apply(lambda x: 1 if x == 'both' else 0)
# merged = merged.drop('_merge', axis=1) # 4. Save the Updated CSV 
# merged.to_csv('updated_csv1.csv', index=False)
# # print(all_possibilities[all_possibilities['exclude']==1])
# print(merged[merged['exclude']==1])