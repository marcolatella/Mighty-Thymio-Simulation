import torch
import torch.nn as nn
import pickle

from .cnn import CNN


MODELPATH = "/home/usi/dev_ws/src/project_ml/model/test_21-epoch=16-val_loss=0.0022.ckpt"


def get_model():
    checkpoint = torch.load(MODELPATH, map_location=torch.device('cpu'))
    model_params = checkpoint['hyper_parameters']['model_params']
    model = CNN(**model_params)
    model_weights = checkpoint['state_dict']

    for key in list(model_weights):
        model_weights[key.replace("model.", "")] = model_weights.pop(key)

    model.load_state_dict(model_weights)
    return model

