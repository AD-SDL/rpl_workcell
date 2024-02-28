#!/usr/bin/env python3
from pathlib import Path
import json
from ml_loop.ml_modules import *
from ml_loop.ml_utils import *
from ml_loop.ml_utils import tecan_proc, tecan_read
from ml_loop.set_transfomer import SmallSetTransformer_v2
from tools.globus_batman2chemspeed import *
import os


def iter_run(init_file, iteration):
    """Actions to perform to run one full experimental loop:
    The experiment starts when the user decides on the color,i.e., Lab values, of the polymer they want to synthezise
    and setting the starting monomers they have in the inventory
    """

    # Read the init.json to extract the user provided information and store the metadata
    with open(init_file, "r") as f:
        init_data = json.load(f)

    # 3. Call the pretrained ML model , ML precidt the L,a,b values of the provided file
    device = "cpu"
    epochs, learning_rate, batch_size, Ndims = 100, 1e-3, 12, 1056
    dropout_ratio = 0.2  # replace with your desired value
    model = SmallSetTransformer_v2(
        dropout_ratio, device, epochs, learning_rate, batch_size
    )

    # load the model which was pretrained on the literature data + inhouse samples
    ckpt = torch.load(
        os.path.join(
            "/home/rpl/workspace/polybot_workcell/polybot_app/ml_loop/set_transformer_checkpoints",
            "set_transformer_dft_ecfp_2.tar",
        )
    )
    model.load_state_dict(ckpt["state_dict"])

    # 4. Use the Set transformer model to get all the predictions
    all_possibilities_avail = all_possibilities[all_possibilities.exclude == 0]

    lab_scaler = joblib.load(
        "/home/rpl/workspace/polybot_workcell/polybot_app/ml_loop/scaler.gz"
    )
    X_valid = all_possibilities_avail.iloc[:, 3:-1]
    pred_data_1, pred_data_2, pred_data_3, y_target = (
        np.array(X_valid.iloc[:, :Ndims].values, dtype=np.float16),
        np.array(X_valid.iloc[:, Ndims : 2 * Ndims].values, dtype=np.float16),
        np.array(X_valid.iloc[:, 2 * Ndims :].values, dtype=np.float16),
        np.zeros(X_valid.shape[0]),
    )
    preds, uncertainties = model.test_model(
        pred_data_1, pred_data_2, pred_data_3, y_target
    )

    preds = lab_scaler.inverse_transform(preds)
    target_Lab = init_data["target_Lab"]
    preds = pd.concat(
        [
            pd.DataFrame(
                all_possibilities_avail["smiles1"].values, columns=["smiles1"]
            ),
            pd.DataFrame(
                all_possibilities_avail["percentage_1"].values, columns=["percentage_1"]
            ),
            pd.DataFrame(
                all_possibilities_avail["smiles2"].values, columns=["smiles2"]
            ),
            pd.DataFrame(
                all_possibilities_avail["percentage_2"].values, columns=["percentage_2"]
            ),
            pd.DataFrame(
                all_possibilities_avail["smiles3"].values, columns=["smiles3"]
            ),
            pd.DataFrame(
                all_possibilities_avail["percentage_3"].values, columns=["percentage_3"]
            ),
            pd.DataFrame(preds, columns=["L", "a", "b"]),
        ],
        axis=1,
    )

    # 5. Select: get the 6 top candidates to run in Chemspeed: create the correct format and send it to chemspeed, file named
    # according to the experimental iteration
    top_candidates = select_next_exp_ml(
        preds, target_Lab, uncertainties, iteration
    )  # print out the first monomer smiles + ratio as well
    print(top_candidates.iloc[:, 2:-1])
    # # # # update init.json with the top 6 selections and the predicted values
    init_data[f"iteration_{iteration+1}"]["ml_suggestions"] = top_candidates.iloc[
        :, :-3
    ].to_dict()
    init_data[f"iteration_{iteration+1}"]["predicted_lab"] = top_candidates.iloc[
        :, -3:
    ].to_dict()

    df_pivot = convert_for_chemspeed(top_candidates.iloc[:, :-3])
    print("df_pivot", df_pivot)

    local_path_tecan = Path("C:/Users/cnmuser/Desktop/Polybot/tecan_code/uv_vis_data")
    local_path_from_tecan = (
        "/home/rpl/workspace/polybot_workcell/polybot_app/demo_files/from_tecan"
    )
    fname_tecan = "ECP_demo_batch_4.asc"
    filename = os.path.join(local_path_from_tecan, fname_tecan)

    df = tecan_read(filename)
    lab_values = tecan_proc(df)
    print("lab_values", lab_values)
    # # Apply stopping criteria: measure the distance of the samples to the desired Lab value, if distance < 10 then stop
    # if get_lab_distance(target_Lab, lab_values) == True:
    #     print("Target color achieved")
    #     # exit()

    # init_data[f"iteration_{iteration+1}"]["uv_vis_data"] = pd.read_csv(df).to_dict()
    # # Check the results of the absorption spectra measurements
    # peak_check_result = peak_check(df)
    # y_train =  lab_values
    # y_train =  np.array(y_train, dtype=np.float16)
    # # 10. Retrain the ML model and save the new weights to the checkpoints folder
    # # X_train, y_train = get_train_data_representation_dft(top_candidates.reset_index()) , lab_values
    # # train_data_1, train_data_2,train_data_3, y_train = np.array(X_train.iloc[:, :Ndims].values, dtype=np.float16),  np.array(X_train.iloc[:, Ndims:2*Ndims].values, dtype=np.float16), np.array(X_train.iloc[:, 2*Ndims:].values, dtype=np.float16), np.array(y_train, dtype=np.float16)
    # y_train = pd.DataFrame(y_train, columns=['L', 'a', 'b'])
    # init_data[f"iteration_{iteration+1}"]["measured_lab"] = y_train.to_dict()

    #     # At the end of each loop return the updated init.json file
    # with open('/home/rpl/workspace/polybot_workcell/polybot_app/demo_files/init.json', 'w', encoding='utf-8') as f:
    #         json.dump(init_data, f, ensure_ascii=False, indent=4)


init_file = "/home/rpl/workspace/polybot_workcell/polybot_app/demo_files/init.json"

with open(init_file, "r") as f:
    init_data = json.load(f)

# 2. Given the initial file with the inventory generate all the possible combinations
inventory = init_data["monomers"]
num_monomers = init_data["number_of_monomers"]
ratio_step = init_data["ratio_step"]

# all_possibilities = create_predictions_dataset_dft(inventory, num_monomers, ratio_step) #, number if monomers and step
# all_possibilities = all_possibilities.drop_duplicates(subset = ['smiles1', 'percentage_1', 'smiles2', 'percentage_2']).reset_index(drop=True)
# all_possibilities['exclude'] = 0
# all_possibilities = all_possibilities.reset_index(drop=True)
all_possibilities = pd.read_csv("/home/rpl/workspace/polybot_workcell/updated_csv1.csv")
iter_run(init_file, 0)


def get_lab_distance(target_Lab, measured_lab_values):
    """Color comparison based on the delta-E value: https://python-colormath.readthedocs.io/en/latest/delta_e.html"""
    target_Lab_color = LabColor(*target_Lab)
    distances = np.array(
        [
            delta_e_cie2000(target_Lab_color, LabColor(*val))
            for val in measured_lab_values
        ]
    )
    print("distances", distances)
    if np.any(distances < 5):
        return True


# target_Lab= np.array([70, 36, 70])
# measured_lab_values= [[72, 38, 55]]
# measured_lab_values= [[79.5, 36, 55.43]]
# measured_lab_values= [[79.8, 33.03, 62.78]]
# print(get_lab_distance(target_Lab, measured_lab_values))
