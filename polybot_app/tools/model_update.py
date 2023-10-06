#from gladier import GladierBaseTool, generate_flow_definition
import pandas as pd
import numpy as np
import os
from rdkit import Chem
from rdkit.Chem import AllChem
from sklearn.ensemble import RandomForestRegressor
import joblib
from scipy.spatial import distance

# Initial inventory in Chemspeed
inventory = ["CCCC", "CCCCCC", "CCC", "CCCCCCC", "CCCCO"]

# helper functions to get the molecular fingerprint representation format
def smile_to_bits(smile):
  mol = Chem.MolFromSmiles(smile)
  return AllChem.GetMorganFingerprintAsBitVect(mol, 2, nBits=1024, useChirality=True)  

def get_vectors(smiles):
  bits = []
  for smile in smiles:
    try:
      bits.append(np.asarray(smile_to_bits(smile)))
    except:
      bits.append(np.zeros(1024))
  return bits

def bits_to_df(smiles, prefix):
  df = pd.DataFrame(get_vectors(smiles))
  columns = [f'{prefix}_{i}' for i in df.columns]
  df.columns = columns
  return df

def create_predictions_dataset(inventory):
    """Given the inventory as a list, create a dataset of all possible combinations
       and ratios of the first molecule of the list with the remaining ones.
       This dataset is the input to the trained predictive model.

    Parameters:
        inventory: SMILES list of molecules available in Chemspeed

    Returns:
        df: a pandas dataframe with all the possible pairs of the inventory
    """
    percentage_1 = np.array([50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100]) / 100
    percentage_2 = 1 - percentage_1
    percentage_3 = np.zeros(len(percentage_1))
    smiles1 = inventory[0]  # the first molecule of the inventory is going to be the standardized one

    data_all = []
    for smiles2 in inventory:
        validation = pd.DataFrame({
            'smiles1': np.repeat(smiles1, len(percentage_1)),
            'smiles2': np.repeat(smiles2, len(percentage_1)),
            'smiles3': np.repeat(0, len(percentage_1)),
            'percentage_1': percentage_1,
            'percentage_2': percentage_2,
            'percentage_3': percentage_3
        })

        validation_1 = bits_to_df(validation['smiles1'], 'bit_1')
        validation_2 = bits_to_df(validation['smiles2'], 'bit_2')
        validation_3 = bits_to_df(validation['smiles3'], 'bit_3')

        validation_dataset = pd.concat([
            validation['smiles2'],
            validation_1,
            validation[['percentage_1']],
            validation_2,
            validation[['percentage_2']],
            validation_3,
            validation[['percentage_3']]
        ], axis=1)

        data_all.append(validation_dataset)

    dataset = pd.concat(data_all, axis=0)
    print(validation_1.values[0][:300])
    return dataset

def get_train_data_representation(dataframe):   
   """Given the dataframe with the experimental results, i.e., smiles + ratios + extracted Lab values
      return the finger print representation"""
   
   df1 = bits_to_df(dataframe.smiles_A, 'bit')
   df2 = bits_to_df(dataframe.smiles_B, 'bit')
   df3 = bits_to_df(dataframe.smiles_C, 'bit')
   dataset = pd.concat([df1,pd.DataFrame(dataframe[['Percentage of A %']].values, columns=['Percentage of A %']),df2,
                        pd.DataFrame(dataframe[['Percentage of B %']].values, columns=['Percentage of B %']),
                        df3, pd.DataFrame(dataframe[['Percentage of C %']].values, columns=['Percentage of C %']),  
                        pd.DataFrame(dataframe[['L']].values, columns=['L']),
                        pd.DataFrame(dataframe[['a']].values, columns=['a']),
                        pd.DataFrame(dataframe[['b']].values, columns=['b'])], axis=1)
   return dataset
   
def train_model(model, X_train, y_train):    
    model.fit(X_train, y_train)
    joblib.dump(model, 'polybot_workcell/tools/saved_checkpoints/random_forest_model.pkl')    
    return model

def predict(model, dataset):
    """Predicts the Lab values for a given dataset"""
    preds = model.predict(dataset.iloc[:, 1:])
    dataset['L'] = preds[:, 0]
    dataset['a'] = preds[:, 1]
    dataset['b'] = preds[:, 2]
    return dataset

def select_next_exp(dataframe, selected_color):
    coords = list(zip(dataframe['a'].values, dataframe['b'].values))
    dist = distance.cdist(selected_color, coords, 'euclidean')
    dataframe['euclidean_distance'] = dist.ravel()
    df = dataframe.sort_values(by=['euclidean_distance']).head(6) # select the 6 points closer to the selected color value
    return df[['smiles2', 'percentage_2']]

def Update_Model():
    import pandas as pd
    import csv
    import os
    import json
    
    """
    Description: Predictive ML model reads the filename with the previous Lab measurements 
    and the experimental inputs

    Parameters:
        inventory: list of smiles with the molecules available in Chemspeed
        tecan_filename: the complete path and name of the csv file with the Lab values
        experimental_filename: the complete path and name of the csv file with the experimental conditions
        e.g., smilesA, smilesB, smilesC, %A, %B, %C
        ml_model_checkpoints: pretrained ML model checkpoints

    Returns:
        df: a pandas dataframe with the next suggested experiments

    """
    #file_name = data.get('csv_name')
    #df = pd.DataFrame()
    experimental_df = pd.read_csv('demo_data/ECP_train_data.csv') # df with combined the tecan results and the initial experiment parameters (i.e., smiles + ratios)
    experimental_df_repr = get_train_data_representation(experimental_df)
    model = joblib.load('polybot_workcell/tools/saved_checkpoints/random_forest_model.pkl')
    X_train, y_train = experimental_df_repr.iloc[:, :-3], experimental_df_repr.iloc[ :, -3:]
    train_model(model, X_train.values, y_train.values) # trains the model on the collected data
    dataset = create_predictions_dataset(inventory) # load the dataset of possible combinations
    df = predict(model, dataset.iloc[:, :]) # gives the Lab values to all the possible combinations dataset
    selected_color = [(30, 35)]
    df = select_next_exp(df, selected_color)
    print(df)
    # check the index on the inventory and update the file that goes to chemspeed
    return df # this file should go to chemspeed

#create_predictions_dataset(inventory)
Update_Model()
#@generate_flow_definition
#class Model_Update(GladierBaseTool):
#    funcx_functions = [Update_Model]
#    required_input = [
#        'funcx_endpoint_compute'
#    ]
