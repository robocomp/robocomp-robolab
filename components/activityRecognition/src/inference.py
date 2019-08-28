import torch
import torch.nn.functional as F
import torch.nn as nn
from model import HCN
import numpy as np
import torch.optim as optim
from utils import utils
import os

np.set_printoptions(suppress=True)

_json_file = os.path.join(os.path.dirname(__file__), 'data/params.json')
_chkp_path = os.path.join(os.path.dirname(__file__), 'data/best.pth.tar')

class Predictor:

	def __init__(self):
		params = utils.Params(_json_file)
		self.model = HCN.HCN(**params.model_args)
		self.optimizer = optim.Adam(filter(lambda p: p.requires_grad, self.model.parameters()), lr=params.lr, betas=(0.9, 0.999), eps=1e-8,
				 weight_decay=params.weight_decay)
		out_channel = params.model_args['out_channel']
		window_size = params.model_args['window_size']
		self.model.fc7 = nn.Sequential(
			nn.Linear((out_channel * 4)*(window_size//16)*(window_size//16), 256), 
			nn.ReLU(),
			nn.Dropout2d(p=0.5))
		self.model.fc8 = nn.Linear(256, 12)

		checkpoint = utils.load_checkpoint(_chkp_path, self.model, self.optimizer)

		self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
		self.model = self.model.to(self.device)
		self.model.eval()

	def estimate(self, sample):
		data = self.process_input(sample)
		inp = np.zeros((1, 3, 32, 15, 2))
		inp[0, :, :, :, 0] = data
		inp = torch.from_numpy(inp).to(self.device, dtype=torch.float)
		output = self.model(inp)
		# output = output.cpu().data.numpy()
		output = output.data
		maxk = 5

		probs = F.softmax(output, dim=1)

		probs, cl = probs.topk(maxk, 1, True, True)
		classes = cl.t()
		probs = probs.t()
		classes = classes.cpu().numpy()[:,0]
		probs = probs.cpu().numpy()[:,0]
		return classes, probs


	def process_input(self, sample):
		origin = sample[:, :, 2]
		data = sample - origin[:, :, None]
		return data


if __name__ == '__main__':
	
	boop = np.random.randint(low=-600, high=900, size=(3, 32, 15))
	# sample = np.load(_test_sample_path)
	# print(sample.shape)
	predictor = Predictor()
	predictor.estimate(boop)
