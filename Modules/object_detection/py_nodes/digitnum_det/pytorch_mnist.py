import torch
import torchvision as tv
import torchvision.transforms as transforms
import torch.nn as nn
import torch.optim as optim
import argparse
import numpy as np

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")



class LeNet(nn.Module):
    def __init__(self):
        super(LeNet, self).__init__()
        # input_size=(1*28*28)
        self.conv1 = nn.Sequential(
            # in_channels, out_channels, kernel_size
            
            nn.Conv2d(1, 6, 5, padding=2),
            # input_size=(6*28*28)
            nn.ReLU(),
            # output_size=(6*14*14)
            nn.MaxPool2d(kernel_size=2, stride=2),
        )
        self.conv2 = nn.Sequential(
            nn.Conv2d(6, 16, 5),
            # input_size=(16*10*10)
            nn.ReLU(),
            # output_size=(16*5*5)
            nn.MaxPool2d(2, 2)
        )
        self.fc1 = nn.Sequential(
            nn.Linear(16*5*5, 120),
            nn.ReLU()
        )
        self.fc2 = nn.Sequential(
            nn.Linear(120, 84),
            nn.ReLU()
        )
        self.fc3 = nn.Linear(84, 10)

    
    def forward(self, x):
        x = self.conv1(x)
        x = self.conv2(x)
        
        
        x = x.view(x.size()[0], -1)
        x = self.fc1(x)
        x = self.fc2(x)
        x = self.fc3(x)
        return x


EPOCH = 12         #
BATCH_SIZE = 64    #
LR = 0.01          #


def train_lenet(trainloader, testloader, opt):
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.SGD(net.parameters(), lr=LR, momentum=0.9)
    scheduler = optim.lr_scheduler.StepLR(optimizer, step_size=4, gamma=0.1)

    for epoch in range(EPOCH):
        sum_loss = 0.0
        
        for i, data in enumerate(trainloader):
            inputs, labels = data
            inputs, labels = inputs.to(device), labels.to(device)

            
            optimizer.zero_grad()

            # forward + backward
            outputs = net(inputs)
            loss = criterion(outputs, labels)
            loss.backward()
            optimizer.step()

            
            sum_loss += loss.item()
            if i % 100 == 99:
                print('[epoch %d, iter %d] loss: %.03f'
                    % (epoch + 1, i + 1, sum_loss / 100))
                sum_loss = 0.0

        scheduler.step()
        
        with torch.no_grad():
            correct = 0
            total = 0
            for data in testloader:
                images, labels = data
                images, labels = images.to(device), labels.to(device)
                outputs = net(images)
                
                _, predicted = torch.max(outputs.data, 1)
                total += labels.size(0)
                correct += (predicted == labels).sum()

        torch.save(net.state_dict(), '%s/net_%03d.pth'%(opt.outf, epoch + 1))


if __name__ == "__main__":
    from thop import profile

    parser = argparse.ArgumentParser()

    parser.add_argument('--outf', default='model/',
                        help='folder to output images and model checkpoints')
    opt = parser.parse_args()

    transform = transforms.ToTensor()


    trainset = tv.datasets.MNIST(
        root='data/',
        train=True,
        download=True,
        transform=transform)


    trainloader = torch.utils.data.DataLoader(
        trainset,
        batch_size=BATCH_SIZE,
        shuffle=True)


    testset = tv.datasets.MNIST(
        root='data/',
        train=False,
        download=True,
        transform=transform)


    testloader = torch.utils.data.DataLoader(
        testset,
        batch_size=BATCH_SIZE,
        shuffle=False)


    net = LeNet()

    input = torch.randn(1, 1, 28, 28)
    macs, params = profile(net, inputs=(input, ))
    print('macs: {}, params: {}'.format(macs, params))

    net = net.to(device)

    train_lenet(trainloader, testloader, opt)
