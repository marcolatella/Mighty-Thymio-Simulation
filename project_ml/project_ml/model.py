import torch
import torch.nn as nn
import pickle
from nnsimple.predictors import Predictor

MODELPATH = "/home/usi/dev_ws/src/project_ml/model/epoch=5-step=2670.ckpt"


def load_pkl_ds(path):
    with open(path, 'rb') as file:
        data = pickle.load(file)
    return data


def get_model():
    model = Predictor().load_from_checkpoint(MODELPATH)
    return model