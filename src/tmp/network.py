# coding=utf-8
import numpy as np
import os
import json

class Network:
    upper_and_lower = [
        (1/5*50, 1/2*50),
        (2*50, 7*50),
        (1, 1.5),
        (0.5, 1)
    ]

    def __init__(self, input_size=3*54, hidden_size=50, output_size=4, learning_rate=0.1):
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

    def train(self, X, y, learning_rate):
        hidden_layer1 = self.sigmoid(np.dot(X, self.W1) + self.b1)
        hidden_layer2 = self.sigmoid(np.dot(hidden_layer1, self.W2) + self.b2)
        output_layer = np.dot(hidden_layer2, self.W3) + self.b3

        # 计算损失
        error = output_layer - y
        loss = np.mean(np.square(error))

        # 反向传播
        output_delta = error
        hidden_delta2 = output_delta.dot(
            self.W3.T) * self.sigmoid_derivative(hidden_layer2)
        hidden_delta1 = hidden_delta2.dot(
            self.W2.T) * self.sigmoid_derivative(hidden_layer1)

        # 更新权重和偏置
        self.W3 -= learning_rate * hidden_layer2.T.dot(output_delta)
        self.b3 -= learning_rate * np.sum(output_delta, axis=0, keepdims=True)
        self.W2 -= learning_rate * hidden_layer1.T.dot(hidden_delta2)
        self.b2 -= learning_rate * np.sum(hidden_delta2, axis=0)
        self.W1 -= learning_rate * X.T.dot(hidden_delta1)
        self.b1 -= learning_rate * np.sum(hidden_delta1, axis=0)
        return loss

    def data_loader(self, data_path="database/"):
        X_list = []
        y_list = []
        # ['2023-3-20-20_25_1_map1.txt', '2023-3-20-22_19_17_map2.txt', '2023-3-21-0_15_42_map3.txt', '2023-3-21-3_11_44_map4.txt']:#
        for data in os.listdir(data_path):
            file = open(os.path.join(data_path, data), 'r')
            y = list(map(float, file.readline().split(',')))
            y_list.append(
                [self.normalization(y[i], *self.upper_and_lower[i]) for i in range(len(y))])
            tmp = []
            for i, line in enumerate(file.readlines()):
                for j, c in enumerate(line):
                    if c == '.':
                        continue
                    elif c == 'A':
                        tmp.extend([i, j, 10])
                    elif c == '\n':
                        continue
                    else:
                        tmp.extend([i, j, int(c)])
            file.close()
            zeros = self.input_size - len(tmp)
            tmp.extend([0]*zeros)
            # print(y_list)
            # print(tmp)
            X_list.append(tmp[:])
        return np.array([X_list]), np.array(y_list)
    
    def training(self,  epochs=1000, learning_rate=0.02):
            # 加载数据集和目标输出
            X_list, y_list = self.data_loader()
            # 训练神经网络
            for epoch in range(epochs):
                for i in range(len(X_list)):
                    loss = self.train(X_list[i], y_list[i],
                                    learning_rate=learning_rate)
                print(F'epoch:{epoch}, loss:{loss}')

    def save(self, weight_path='src/weight.py'):
        file = open(weight_path, 'w')
        file.write(f'W1 = {json.dumps(self.W1.tolist())}\n')       
        file.write(f'W2 = {json.dumps(self.W2.tolist())}\n')
        file.write(f'W3 = {json.dumps(self.W3.tolist())}\n')
        file.write(f'b1 = {json.dumps(self.b1.tolist())}\n')
        file.write(f'b2 = {json.dumps(self.b2.tolist())}\n')
        file.write(f'b3 = {json.dumps(self.b3.tolist())}\n')
        file.close()

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

if __name__ == '__main__':
    network = Network()
    network.training(2000,0.01)
    network.save()