import torch.nn as nn
import torch.nn.functional as F
import numpy as np
import torch
from torch.autograd import Variable
from utils import generate_anchor
import cv2


class SiamRPN(nn.Module):
    def __init__(self, size=2, feature_out=512, anchor=5):
        self.batchsize = 32
        configs = [3, 96, 256, 384, 384, 256]
        configs = list(map(lambda x: 3 if x==3 else x*size, configs))
        feat_in = configs[-1]
        super(SiamRPN, self).__init__()
        self.featureExtract = nn.Sequential(
            nn.Conv2d(configs[0], configs[1] , kernel_size=11, stride=2),
            nn.BatchNorm2d(configs[1]),
            nn.MaxPool2d(kernel_size=3, stride=2),
            nn.ReLU(inplace=True),
            nn.Conv2d(configs[1], configs[2], kernel_size=5),
            nn.BatchNorm2d(configs[2]),
            nn.MaxPool2d(kernel_size=3, stride=2),
            nn.ReLU(inplace=True),
            nn.Conv2d(configs[2], configs[3], kernel_size=3),
            nn.BatchNorm2d(configs[3]),
            nn.ReLU(inplace=True),
            
            nn.Conv2d(configs[3], configs[4], kernel_size=3),
            nn.BatchNorm2d(configs[4]),
            nn.ReLU(inplace=True),
            nn.Conv2d(configs[4], configs[5], kernel_size=3),
            nn.BatchNorm2d(configs[5]),
        )
        for i in range(0, len(self.featureExtract)):
            self.featureExtract[i].requires_grad=False
        ratios = [0.33, 0.5, 1, 2, 3]
        scales = [8, ]
        self.anchor = anchor
        self.anchors = Variable( torch.from_numpy(generate_anchor(8, scales, ratios, 19))).float().cuda()
        
        self.feature_out = feature_out

        self.conv_r1 = nn.Conv2d(feat_in, feature_out*4*anchor, 3)
        self.conv_r2 = nn.Conv2d(feat_in, feature_out, 3)
        self.conv_cls1 = nn.Conv2d(feat_in, feature_out*2*anchor, 3)
        self.conv_cls2 = nn.Conv2d(feat_in, feature_out, 3)
        self.regress_adjust = nn.Conv2d(4 * anchor, 4 * anchor, 1)

        self.r1_kernel = []
        self.cls1_kernel = []
        self.cfg = {}

    def forward(self, x):
        
        x_f = self.featureExtract(x)
        x_c = self.conv_cls2(x_f)
        kernel_size = x_c.data.size()[-1]
        x_c = x_c.view(1,self.feature_out*self.batchsize, kernel_size, kernel_size)
        cls = F.conv2d( x_c, self.cls1_kernel, groups=self.batchsize)
        #cls = cls.view(self.batchsize, self.anchor*2, cls.data.size()[-1], -1)

        x_r = self.conv_r2(x_f)
        kernel_size = x_r.data.size()[-1]
        x_r = x_r.view(1, self.feature_out * self.batchsize, kernel_size, kernel_size)
        regress = F.conv2d(x_r, self.r1_kernel, groups=self.batchsize)

        regress = regress.view(self.batchsize, self.anchor * 4, regress.data.size()[-1], -1)

        self.score = cls.view(self.batchsize, 2, self.anchor, cls.data.size()[-1], -1)
        self.delta = self.regress_adjust(regress).view(self.batchsize, 4, self.anchor, cls.data.size()[-1], -1)

        if 0:
            out = F.softmax(self.score, dim=1)[:,1,:,:,:]
            show = out[0, :, :, :].data.cpu().numpy()
            show = np.max(show.reshape(5, 361), axis=0).reshape(19, 19)
            cv2.imshow('show', show)
        
        return self.delta.view(self.batchsize, 4*self.anchor, cls.data.size()[-1], -1), self.score.view(self.batchsize, 2*self.anchor, cls.data.size()[-1], -1)

    def temple(self, z):
        self.batchsize = z.shape[0]
        z_f = self.featureExtract(z)
        r1_kernel_raw = self.conv_r1(z_f)
        cls1_kernel_raw = self.conv_cls1(z_f)
        kernel_size = r1_kernel_raw.data.size()[-1]
        self.r1_kernel = r1_kernel_raw.view(self.anchor*4*self.batchsize,self.feature_out, kernel_size, kernel_size)
        self.cls1_kernel = cls1_kernel_raw.view(self.anchor*2*self.batchsize,self.feature_out, kernel_size, kernel_size)

    def loss(self, lab_cls, lab_reg):

        ths_positive = 0.5
        ths_negative = 0.3

        cls = lab_cls.view(self.batchsize*self.anchor*19*19)
        label_positive = (cls > ths_positive).long()
        label_negative = (cls > ths_negative).long()
        score = self.score.permute(0, 2, 3, 4, 1).contiguous().view(self.batchsize*self.anchor*19*19, 2)

        try:
            closs1 = F.cross_entropy(score[torch.where(label_positive>0)], label_positive[torch.where(label_positive>0)])
        except:
            closs1 = 0.0

        closs2 = F.cross_entropy(score[torch.where(label_negative<1)], label_positive[torch.where(label_negative<1)])
        closs = closs1 + closs2

        if 0:
            show = lab_cls[0,:,:,:].data.cpu().numpy().astype(np.float)
            show = np.max(show.reshape(5, 361), axis=0).reshape(19, 19)
            cv2.imshow('target', show*(show>0.4))
            cv2.waitKey(1)

        #closs = F.cross_entropy(self.score.permute(1, 0), clabel)
        anchor = self.anchors.repeat(self.batchsize, 1).view(self.batchsize, self.anchor, 19, 19, 4).view(-1, 4)
        delta = self.delta.permute(0, 2, 3, 4, 1).contiguous().view(self.batchsize, self.anchor, 19, 19, 4).view(-1, 4)
        reg_target = lab_reg.repeat(1, 19*19*5).contiguous().view(self.batchsize, 19*19*5, 4).permute(1, 0, 2).contiguous().view(-1, 4).float()

        bbox = torch.zeros_like(reg_target).cuda()
        bbox[:, 0] = (reg_target[:, 0]-anchor[:, 0])/anchor[:, 2]
        bbox[:, 1] = (reg_target[:, 1]-anchor[:, 1])/anchor[:, 3]
        bbox[:, 2] = torch.log( (reg_target[:, 2] + 0.001) / (anchor[:, 2] + 0.001) )
        bbox[:, 3] = torch.log( (reg_target[:, 3] + 0.001) / (anchor[:, 3] + 0.001) )

        try:

            rloss = F.smooth_l1_loss(delta[torch.where(label_positive>0)], bbox[torch.where(label_positive>0)], reduction='none')
            rloss = torch.mean(rloss)

        except:
            rloss = 0


        loss = closs + rloss
       
        return loss, closs, rloss

class SiamRPNBIG(SiamRPN):
    def __init__(self):
        super(SiamRPNBIG, self).__init__(size=2)
        self.cfg = {'lr':0.295, 'window_influence': 0.42, 'penalty_k': 0.055, 'instance_size': 271, 'adaptive': True} # 0.383


class SiamRPNvot(SiamRPN):
    def __init__(self):
        super(SiamRPNvot, self).__init__(size=1, feature_out=256)
        self.cfg = {'lr':0.45, 'window_influence': 0.44, 'penalty_k': 0.04, 'instance_size': 271, 'adaptive': False} # 0.355


class SiamRPNotb(SiamRPN):
    def __init__(self):
        super(SiamRPNotb, self).__init__(size=1, feature_out=256)
        self.cfg = {'lr': 0.30, 'window_influence': 0.40, 'penalty_k': 0.22, 'instance_size': 271, 'adaptive': False} # 0.655

