# importing necessary libraries
import pandas as pd
import numpy as np
import torch
import tqdm
import torch.nn as nn
from ml_loop.ml_modules import SAB, PMA
from torch.autograd import Variable
from torch.utils.data import DataLoader
import torch.nn.functional as F
device = 'cpu'

# Helper functions
def combine(data1, data2):
    return np.array(list(zip(data1, data2)))

def enable_dropout(m):
  for each_module in m.modules():
    if each_module.__class__.__name__.startswith('Dropout'):
      each_module.train()

def weighted_mse_loss(input,target):
    weights = Variable(torch.Tensor([0.5,1,1]))#.cuda()  
    pct_var = (input-target)**2
    out = pct_var * weights.expand_as(target)
    loss = out.mean() 
    return loss

###########################################################################################
class MonomerPair(torch.utils.data.Dataset):
  def __init__(self, dataset1, dataset2, dataset3, y):
    self.x = np.array(list(zip(dataset1, dataset2, dataset3)))
    self.x = np.concatenate([dataset1, dataset2, dataset3], axis=-1)
    self.y = np.reshape(y, (y.shape[0], -1))
    
  def __getitem__(self, index):
    return self.x[index], self.y[index]
  
  def __len__(self):
    return len(self.x)
  

class SmallSetTransformer_v2(nn.Module):
    def __init__(self, dropout_ratio, device, epochs, learning_rate, batch_size):
        super().__init__()
        self.dropout_ratio = dropout_ratio
        self.device = device
        self.epochs = epochs
        self.learning_rate = learning_rate
        self.batch_size = batch_size
        self.enc = nn.Sequential(
            SAB(dim_in=1056, dim_out=800, num_heads=5),
            nn.Dropout(p=self.dropout_ratio),
            nn.LayerNorm(800),
            SAB(dim_in=800, dim_out=500, num_heads=5),
            nn.LayerNorm(500),
            nn.Dropout(p=self.dropout_ratio),
            SAB(dim_in=500, dim_out=200, num_heads=5),
            nn.Dropout(p=self.dropout_ratio),
            nn.LayerNorm(200)
        )
        self.dec = nn.Sequential(PMA(dim=200, num_heads=4, num_seeds=1),
                                SAB(dim_in=200, dim_out=100, num_heads=5),
                                 nn.LayerNorm(100),
                                 nn.Dropout(p=self.dropout_ratio),
                                 nn.LeakyReLU(),
                                 nn.Linear(in_features=100, out_features=3))

    def forward(self, x):
        x = x.reshape((x.shape[0], 3, -1))
        x = self.enc(x)
        x = self.dec(x)
        return x.squeeze(-1).squeeze(1)

    def predict(self, data):
        #self = self.to(self.device)
        torch.manual_seed(0)
        x = torch.FloatTensor(data)
        x = x.float().to(self.device)
        y = self(x)
        return y.detach().squeeze(1).numpy()

    def train_model(self, data1, data2, data3, y_new):
        #self = self.to(self.device)
        torch.manual_seed(0)
        optimizer = torch.optim.Adam(self.parameters(), lr=self.learning_rate)
        criterion = weighted_mse_loss
        losses = []
        for _ in tqdm.tqdm(range(self.epochs)):
            for x, y in DataLoader(MonomerPair(data1, data2, data3, y_new), batch_size=self.batch_size):
                x, y = x.float(), y.float() #.to(self.device)
                loss = criterion(self(x), y)
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
                losses.append(loss.item())
        return losses

    def test_model(self, data1, data2, data3, target):
        y_list=[]
        y_list_std=[]
        torch.manual_seed(0)
        enable_dropout(self)

        for i in range(10):
            self = self.to(self.device)
            for x, y in DataLoader(MonomerPair(data1, data2, data3, target), batch_size=len(data1)):
                x, y = x.float(), y.float() #.to(self.device)
                y= self(x)
                y_list.append(y.detach().numpy())
                y_list_std.append(y[:, :].detach().numpy())
        return np.mean(y_list, axis=0), np.std(np.std(y_list_std, axis=0), axis=1)