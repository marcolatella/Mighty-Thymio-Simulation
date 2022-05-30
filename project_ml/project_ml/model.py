import torch
import torch.nn as nn
import pickle

from .cnn import CNN


MODELPATH = "/home/usi/dev_ws/src/project_ml/model/epoch=5-step=2670.ckpt"


def get_model():
    checkpoint = torch.load(MODELPATH)
    model_params = checkpoint['hyper_parameters']['model_params']
    model = CNN(**model_params)
    model_weights = checkpoint['state_dict']

    for key in list(model_weights):
        model_weights[key.replace("model.", "")] = model_weights.pop(key)

    model.load_state_dict(model_weights)
    return model

