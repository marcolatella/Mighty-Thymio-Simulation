import torch
from torch import nn


class MLP(nn.Module):
    def __init__(self,
                 input_size,
                 hidden_size,
                 out_size,
                 n_layers=1,
                 activation=nn.ReLU,
                 dropout=0.0,
                 lazy=False):
        """
        Multi Layer Perceptron. It is composed by n_layers of (Linear, activation, Dropout)


        :param input_size: The number of expected features in the input x
        :param hidden_size: The number of features in the hidden state h
        :param out_size: the number of classes in the output
        :param n_layers: number of layers in the MLP, defaults to 1 (optional)
        :param activation: The activation function to use
        :param dropout: The dropout rate to use, defaults 0.0
        :param lazy: If True, the network will use the nn.LazyLayer() which will infer the input dimension.
                    if True the parameter input_size will not be used.
        """
        super(MLP, self).__init__()

        self.input_size = input_size
        self.hidden_size = hidden_size
        self.n_layers = n_layers
        self.out_size = out_size
        self.activation = activation
        self.dropout = dropout
        self.lazy = lazy

        lin = []
        up_scale = nn.LazyLinear(self.hidden_size) if self.lazy else nn.Linear(self.input_size, self.hidden_size)
        for i in range(n_layers):
            linear = up_scale if i == 0 else nn.Linear(self.hidden_size, self.hidden_size)
            lin.append(linear)
            lin.append(activation)
            lin.append(nn.Dropout(self.dropout))

        lin.append(nn.Linear(self.hidden_size, self.out_size))

        self.out = nn.Sequential(*lin)

    def forward(self, x):
        return self.out(x)


class ConvPool(nn.Module):
    def __init__(self, input_size=3, hidden_size=32, kernel_size=3, kernel_pooling=2, dropout=0.0, **kwargs):
        super(ConvPool, self).__init__()
        self.input_size = input_size
        self.hidden_size = hidden_size
        self.kernel_pooling = kernel_pooling
        self.kernel_size = kernel_size
        self.dropout = dropout

        self.layer1 = nn.Sequential(
            nn.Conv2d(self.input_size, self.hidden_size, self.kernel_size),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=self.kernel_pooling),
            nn.Dropout(self.dropout)
        )

    def forward(self, x):
        out = self.layer1(x)
        return out


class CNN(nn.Module):
    def __init__(self,
                 input_size,
                 hidden_size_conv,
                 hidden_size_fc,
                 out_size,
                 image_shape,
                 kernel_size=3,
                 kernel_pooling=2,
                 dropout=0.0,
                 n_layers_mlp=1,
                 stride=1,
                 padding=0):
        self.input_size = input_size
        self.hidden_size_conv = hidden_size_conv
        self.hidden_size_fc = hidden_size_fc
        self.out_size = out_size
        self.n_layers_mlp = n_layers_mlp
        self.kernel_size = kernel_size
        self.kernel_pooling = kernel_pooling
        self.image_shape = image_shape
        self.dropout = dropout
        self.stride = stride
        self.padding = padding
        super(CNN, self).__init__()

        self.conv1 = ConvPool(self.input_size,
                              self.hidden_size_conv, self.kernel_size, self.kernel_pooling, self.dropout)

        self.conv2 = ConvPool(self.hidden_size_conv,
                              2 * self.hidden_size_conv, self.kernel_size, self.kernel_pooling, self.dropout)

        self.fc = MLP(64 * 28 * 38, 512, self.out_size, activation=nn.ReLU(), lazy=False)

    def forward(self, x):
        out = self.conv1(x)
        out = self.conv2(out)
        out = out.view(x.size(0), -1)
        out = self.fc(out)
        return out
