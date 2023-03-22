import numpy as np
class Network:
    upper_and_lower = [
        (1/5*50, 1/2*50),
        (2*50, 7*50),
        (1, 1.5),
        (0.5, 1)
    ]

    def __init__(self, input_size=3*58, hidden_size=50, output_size=4, learning_rate=0.1):
        self.input_size = input_size  # 输入大小
        self.hidden_size = hidden_size  # 隐藏层大小
        self.output_size = output_size  # 输出大小
        self.learning_rate = learning_rate  # 学习率
        # 权重和偏置
        self.W1 = np.random.randn(
            input_size, hidden_size) / np.sqrt(input_size)  # 第一层权重矩阵
        self.b1 = np.zeros((1, hidden_size))  # 第一层偏置矩阵
        self.W2 = np.random.randn(
            hidden_size, hidden_size) / np.sqrt(hidden_size)  # 第二层权重矩阵
        self.b2 = np.zeros((1, hidden_size))  # 第二层偏置矩阵
        self.W3 = np.random.randn(
            hidden_size, output_size) / np.sqrt(hidden_size)  # 输出层权重矩阵
        self.b3 = np.zeros((1, output_size))  # 输出层偏置矩阵

    def normalization(self, num, upper, lower,  normalize=True):
        # 归一化函数，输入要归一化的数字和上限下限
        if normalize:
            return (num-lower)/(upper-lower)
        else:
            res = num*(upper-lower)+lower
            res = max(res, lower)
            res = min(res, upper)
            return res

    def sigmoid(self, x):
        return 1 / (1 + np.exp(-x))

    def sigmoid_derivative(self, x):
        return x * (1 - x)


    def weight_loader(self):
        import weight as weights
        self.W1 = weights.W1
        self.W2 = weights.W2
        self.W3 = weights.W3
        self.b1 = weights.b1
        self.b2 = weights.b2
        self.b3 = weights.b3


    def get_params(self, X: list):
        zeros = self.input_size - len(X)
        X.extend([0]*zeros)
        X = np.array([X])
        hidden_layer1 = self.sigmoid(np.dot(X, self.W1) + self.b1)
        hidden_layer2 = self.sigmoid(np.dot(hidden_layer1, self.W2) + self.b2)
        output_layer = np.dot(hidden_layer2, self.W3) + self.b3
        params = output_layer.tolist()[0]
        return [self.normalization(params[i], *self.upper_and_lower[i], False) for i in range(len(params))]

