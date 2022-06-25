# -*- coding:utf-8 _*-
"""
@Author  : Chexqi Yutao
@Time    : 2019/7/1 16:00
"""
from collections import OrderedDict
import torch
import torch.nn as nn
import numpy as np


class UNet(nn.Module):
    def __init__(self, in_channels=3, out_channels=1, init_features=32, WithActivateLast=True, ActivateFunLast=None):
        super(UNet, self).__init__()
        features = init_features
        self.WithActivateLast = WithActivateLast  # True：则最后一层输出增加激活，False:最后一层输出不加激活
        # self.ActivateFunLast = ActivateFunLast  # 如果需要激活层，设置最后激活层函数
        self.encoder1 = UNet._block(in_channels, features, name="enc1")
        self.pool1 = nn.MaxPool2d(kernel_size=2, stride=2)
        self.encoder2 = UNet._block(features, features * 2, name="enc2")
        self.pool2 = nn.MaxPool2d(kernel_size=2, stride=2)
        self.encoder3 = UNet._block(features * 2, features * 4, name="enc3")
        self.pool3 = nn.MaxPool2d(kernel_size=2, stride=2)
        self.encoder4 = UNet._block(features * 4, features * 8, name="enc4")
        self.pool4 = nn.MaxPool2d(kernel_size=2, stride=2)

        self.bottleneck = UNet._block(features * 8, features * 16, name="bottleneck")

        self.upconv4 = nn.ConvTranspose2d(
            features * 16, features * 8, kernel_size=2, stride=2
        )
        self.decoder4 = UNet._block((features * 8) * 2, features * 8, name="dec4")
        self.upconv3 = nn.ConvTranspose2d(
            features * 8, features * 4, kernel_size=2, stride=2
        )
        self.decoder3 = UNet._block((features * 4) * 2, features * 4, name="dec3")
        self.upconv2 = nn.ConvTranspose2d(
            features * 4, features * 2, kernel_size=2, stride=2
        )
        self.decoder2 = UNet._block((features * 2) * 2, features * 2, name="dec2")
        self.upconv1 = nn.ConvTranspose2d(
            features * 2, features, kernel_size=2, stride=2
        )
        self.decoder1 = UNet._block(features * 2, features, name="dec1")
        self.conv = nn.Conv2d(
            in_channels=features, out_channels=out_channels, kernel_size=1
        )

    def forward(self, x):
        enc1 = self.encoder1(x)
        enc2 = self.encoder2(self.pool1(enc1))
        enc3 = self.encoder3(self.pool2(enc2))
        enc4 = self.encoder4(self.pool3(enc3))

        bottleneck = self.bottleneck(self.pool4(enc4))

        dec4 = self.upconv4(bottleneck)
        dec4 = torch.cat((dec4, enc4), dim=1)
        dec4 = self.decoder4(dec4)
        dec3 = self.upconv3(dec4)
        dec3 = torch.cat((dec3, enc3), dim=1)
        dec3 = self.decoder3(dec3)
        dec2 = self.upconv2(dec3)
        dec2 = torch.cat((dec2, enc2), dim=1)
        dec2 = self.decoder2(dec2)
        dec1 = self.upconv1(dec2)
        dec1 = torch.cat((dec1, enc1), dim=1)
        dec1 = self.decoder1(dec1)  # 2*32*256*256
        if self.WithActivateLast:
            return torch.sigmoid(self.conv(dec1))  # BS*1*256*256
            # return self.ActivateFunLast(self.conv(dec1))
        else:
            return self.conv(dec1)  # BS*1*256*256

    '''
    staticmethod用于修饰类中的方法,使其可以在不创建类实例的情况下调用方法，这样做的好处是执行效率比较高。
    当然，也可以像一般的方法一样用实例调用该方法。该方法一般被称为静态方法。静态方法不可以引用类中的属性或方法，
    其参数列表也不需要约定的默认参数self。我个人觉得，静态方法就是类对外部函数的封装，有助于优化代码结构和提高程序的可读性。
    当然了，被封装的方法应该尽可能的和封装它的类的功能相匹配。
    '''

    @staticmethod
    def _block(in_channels, features, name):
        return nn.Sequential(
            OrderedDict(  # 用字典的形式进行网络定义，字典key即为网络每一层的名称
                [
                    (
                        name + "conv1",
                        nn.Conv2d(
                            in_channels=in_channels,
                            out_channels=features,
                            kernel_size=3,
                            padding=1,
                            bias=False,
                        ),
                    ),
                    (name + "norm1", nn.BatchNorm2d(num_features=features)),
                    (name + "relu1", nn.ReLU(inplace=True)),
                    (
                        name + "conv2",
                        nn.Conv2d(
                            in_channels=features,
                            out_channels=features,
                            kernel_size=3,
                            padding=1,
                            bias=False,
                        ),
                    ),
                    (name + "norm2", nn.BatchNorm2d(num_features=features)),
                    (name + "relu2", nn.ReLU(inplace=True)),
                ]
            )
        )


if __name__ == '__main__':
    Input = torch.randn((2, 1, 256, 256))  # 任意生成样本
    Target = torch.empty((2, 1, 256, 256), dtype=torch.long).random_(2)  # 任意生成标签

    Unet = UNet(in_channels=1, out_channels=2)  # 2分类输出为两个通道
    LossFun = nn.CrossEntropyLoss()
    Output = Unet(Input)
    print(Output.shape)
    print(Target.shape)

    # %% 官方函数计算CrossEntropyLoss
    BatchLoss = LossFun(Output, Target[:, 0, :, :])  # CrossEntropyLoss的Target必须没有通道的维度，即(BatchSize, W, H)
    print(BatchLoss)

    # %% 手写验证CrossEntropyLoss
    Errs = []
    for i, Sample in enumerate(Output):  # 遍历每个样本,每个像素点
        for j in range(256):
            for k in range(256):
                temppredict = Output[i, :, j, k]  # 每个像素的5个预测概率
                temptarget = Target[i, 0, j, k]  # 每个像素的真实归类
                err = -temppredict[temptarget] + torch.log(torch.sum(np.e ** temppredict))  # 计算每个样本交叉熵
                Errs.append(err.detach().numpy())
    print(np.mean(Errs))
