"""
   seq_train.py
   COMP9444, CSE, UNSW
"""

import argparse
import sys
import torch
import torch.nn.functional as F
import torch.optim as optim
import numpy as np
from seq_models import SRN_model, LSTM_model
from anb2n import lang_anb2n


parser = argparse.ArgumentParser()
# language options
parser.add_argument('--lang', type=str, default='anb2n', help='anb2n or anb2nc3n')
parser.add_argument('--length', type=int, default=0, help='max number of As')
# network options
parser.add_argument('--model', type=str, default='srn', help='srn or lstm')
parser.add_argument('--hid', type=int, default=0, help='number of hidden units')
# optimizer options
parser.add_argument('--optim', type=str, default='sgd', help='sgd or adam')
parser.add_argument('--lr', type=float, default=0.005, help='learning rate')
parser.add_argument('--mom', type=float, default=0, help='momentum (srn)')
parser.add_argument('--init', type=float, default=0.001, help='initial weight size (srn)')
# training options
parser.add_argument('--epoch', type=int, default=0, help='number of training epochs (\'000s)')
parser.add_argument('--out_path', type=str, default='net', help='outputs path')
args = parser.parse_args()

if args.lang == 'anb2n':
    num_class = 2
    hid_default = 2
    epoch_default = 100
elif args.lang == 'anb2nc3n':
    num_class = 3
    if args.model == 'lstm':
        hid_default = 3
        epoch_default = 100
    else:           # srn
        hid_default = 4
        epoch_default = 200

if args.length == 0:
    args.length = 4

lang = lang_anb2n(num_class,args.length)

if args.hid == 0:
    args.hid = hid_default

if args.epoch == 0:
    args.epoch = epoch_default
    
if args.model == 'srn':
    net = SRN_model(num_class,args.hid,num_class)
    for m in list(net.parameters()):
        m.data.normal_(0,args.init)
elif args.model == 'lstm':
    net = LSTM_model(num_class,args.hid,num_class)

if args.optim == 'adam':
    optimizer = optim.Adam(net.parameters(), lr=args.lr,
                           weight_decay=0.0001)
else:
    optimizer = optim.SGD(net.parameters(), lr=args.lr,
                      momentum=args.mom, weight_decay=0.0001)

loss_function = F.nll_loss

np.set_printoptions(suppress=True,precision=2,sign=' ')

for epoch in range((args.epoch*1000)+1):
    net.zero_grad()

    input, seq, target, state = lang.get_sequence()
    label  = seq[1:]

    net.init_hidden()
    hidden, output = net(input)
    log_prob = F.log_softmax(output, dim=2)
    prob_out = torch.exp(log_prob)
    loss = F.nll_loss(log_prob.squeeze(), label.squeeze())
    loss.backward()
    optimizer.step()

    if epoch % 1000 == 0:

        # Check accuracy during training
        with torch.no_grad():
            net.eval()

            input, seq, target, state = lang.get_sequence()
            label = seq[1:]
            
            net.init_hidden()
            hidden, output = net(input)
            log_prob = F.log_softmax(output, dim=2)
            prob_out = torch.exp(log_prob)
            
            lang.print_outputs(epoch, seq, state, hidden, target, output)
            sys.stdout.flush()

            net.train()

        if epoch % 10000 == 0:
            path = args.out_path+'/'
            torch.save(net.state_dict(),path+'%s_%s%d_%d.pth'
                       %(args.lang,args.model,args.hid,epoch/1000))

