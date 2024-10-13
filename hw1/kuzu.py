"""
   kuzu.py
   COMP9444, CSE, UNSW
"""

from __future__ import print_function
import torch
import torch.nn as nn
import torch.nn.functional as F

class NetLin(nn.Module):
    def __init__(self):
        super().__init__()
        self.fc = nn.Linear(28 * 28, 10)

    def forward(self, x):
        x = x.flatten(start_dim=1)
        return F.log_softmax(self.fc(x), dim=1)

class NetFull(nn.Module):
    # two fully connected tanh layers followed by log softmax
    def __init__(self):
        super().__init__()
        self.input_layer = nn.Linear(28 * 28, 220)
        self.output_layer = nn.Linear(220, 10)

    def forward(self, x):
        flattened_images = x.flatten(start_dim=1)
        activated_output = torch.tanh(self.input_layer(flattened_images))
        return F.log_softmax(self.output_layer(activated_output), dim=1) # CHANGE CODE HERE

class NetConv(nn.Module):
    # two convolutional layers and one fully connected layer,
    # all using relu, followed by log_softmax
    def __init__(self):
        super(NetConv, self).__init__()
        # Two convolutional layers with Batch Normalization
        self.conv_layer1 = nn.Conv2d(1, 32, kernel_size=3)  
        self.bn1 = nn.BatchNorm2d(32)  
        self.conv_layer2 = nn.Conv2d(32, 64, kernel_size=3)
        self.bn2 = nn.BatchNorm2d(64) 

        # Fully connected layers
        self.fc_layer1 = nn.Linear(64 * 12 * 12, 128)     
        self.fc_layer2 = nn.Linear(128, 10)

        # Dropout layer
        self.dropout = nn.Dropout(p=0.5)  # 50% dropout

    def forward(self, x):
        # First convolution layer with Batch Normalization and ReLU
        feature_maps1 = F.relu(self.bn1(self.conv_layer1(x)))
        # Second convolution layer with Batch Normalization and ReLU
        feature_maps2 = F.relu(self.bn2(self.conv_layer2(feature_maps1)))
        # Max pooling
        pooled_output = F.max_pool2d(feature_maps2, 2)

        # Flatten the output for the fully connected layer
        flattened_output = pooled_output.view(-1, 64 * 12 * 12)

        # Fully connected layer with ReLU
        activated_fc1 = F.relu(self.fc_layer1(flattened_output))
        # Apply Dropout
        activated_fc1 = self.dropout(activated_fc1)

        # Output layer
        output = self.fc_layer2(activated_fc1)

        return F.log_softmax(output, dim=1)

