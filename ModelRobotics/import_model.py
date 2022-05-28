import torch
import torch.nn as nn
import pickle
from nnsimple.predictors import Predictor

MODELPATH = "/Users/marcolatella/PycharmProjects/roboticsCNN/checkpoints/epoch=5-step=2670.ckpt"
TESTSET = '/Users/marcolatella/PycharmProjects/roboticsCNN/PersonalData/testset.pkl'


def load_pkl_ds(path):
    with open(path, 'rb') as file:
        data = pickle.load(file)
    return data


def main():
    testset = load_pkl_ds(TESTSET)

    test_loader = torch.utils.data.DataLoader(testset,
                                              batch_size=32,
                                              shuffle=False,
                                              num_workers=2
                                              )

    model = Predictor().load_from_checkpoint(MODELPATH)
    model.eval()

    with torch.no_grad():
        correct = 0
        total = 0
        for i, (inputs, labels) in enumerate(test_loader):
            out = model(inputs)
            soft = nn.Softmax(dim=1)
            out = soft(out)
            _, predicted = out.max(dim=1)
            total += labels.size(0)
            correct += (predicted == labels).sum().item()
        test_acc = 100 * correct / total
        print(f'Test accuracy: {test_acc} %')
        print(f'Test error rate: {100 - 100 * correct / total: .2f} %')


if __name__ == '__main__':
    main()
