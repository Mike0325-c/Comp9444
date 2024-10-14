"""
   kuzu.py
   COMP9444, CSE, UNSW
"""

from __future__ import print_function
import torch
import torch.nn as nn
import torch.nn.functional as F

class NetLin(nn.Module):
    # linear function followed by log_softmax
    def __init__(self):
        super(NetLin, self).__init__()
        # INSERT CODE HERE
        self.fc = nn.Linear(28 * 28, 10)

    def forward(self, x):
        x = x.flatten(start_dim=1)
        return F.log_softmax(self.fc(x), dim=1)

class NetFull(nn.Module):
    # two fully connected tanh layers followed by log softmax
    def __init__(self):
        super(NetFull, self).__init__()
        # INSERT CODE HERE
        self.hidden_layer = nn.Linear(28 * 28, 240)
        self.output_layer = nn.Linear(240, 10)

    def forward(self, x):
        x = x.flatten(start_dim=1)
        activated_output = torch.tanh(self.hidden_layer(x))
        return F.log_softmax(self.output_layer(activated_output), dim=1)

class NetConv(nn.Module):
    # two convolutional layers and one fully connected layer,
    # all using relu, followed by log_softmax
    def __init__(self):
        super(NetConv, self).__init__()
        # INSERT CODE HERE
        self.conv1 = nn.Conv2d(in_channels=1, out_channels=32, kernel_size=5, stride=1, padding=2)  
        self.conv2 = nn.Conv2d(in_channels=32, out_channels=64, kernel_size=5, stride=1, padding=2)
        self.fc1 = nn.Linear(64 * 7 * 7, 128)
        # Dropout layer to reduce overfitting
        self.dropout = nn.Dropout(p=0.5)  
        self.output_layer = nn.Linear(128, 10)
        # Max pooling layer
        self.pool = nn.MaxPool2d(kernel_size=2, stride=2, padding=0)

    def forward(self, x):
    
        x = self.pool(F.relu(self.conv1(x)))
        x = self.pool(F.relu(self.conv2(x)))
        x = x.view(-1, 64 * 7 * 7)    
        x = F.relu(self.fc1(x))
        x = self.dropout(x)
        x = F.log_softmax(self.output_layer(x), dim=1)
        return x
