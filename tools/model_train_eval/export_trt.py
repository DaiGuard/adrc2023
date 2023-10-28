import sys
import os

import torch
import torchvision
import torchvision.transforms as transforms
from torch.utils.data import DataLoader

from torch2trt import torch2trt
from xy_dataset import XYDataset

import cv2

if __name__ == '__main__':

    TRANSFORMS = transforms.Compose([
        transforms.ColorJitter(0.2, 0.2, 0.2, 0.2),
        transforms.Resize((224, 224)),
        transforms.ToTensor(),
        transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
    ])

    args = sys.argv

    model_path = args[1]
    dataset = XYDataset(args[2], TRANSFORMS)

    device = torch.device('cuda')
    model = torch.load(model_path)
    model = model.to(device)

    data = torch.zeros((1, 3, 224, 224)).to(device)
    model_trt = torch2trt(model, [data], fp16_mode=True)

    cv2.namedWindow('data_view', cv2.WINDOW_NORMAL)
    cv2.createTrackbar('ID', 'data_view', 0, len(dataset)-1, lambda val: print(val))

    id = -1
    while True:
        current_id = cv2.getTrackbarPos('ID', 'data_view')
        if current_id != id:
            id = current_id
            res_image, res_xy = dataset.getData(id)

            image, xy = dataset[id]
            image = image.unsqueeze(dim=0)
            image = image.to(device)            
            output = model_trt.forward(image)
            output = output.to('cpu')

        v_val = float(res_xy[0])
        u_val = float(res_xy[1])
        v = int((1.0 - v_val) * res_image.shape[0] / 2.0)
        u = int((1.0 - u_val) * res_image.shape[1] / 2.0)

        res_image = cv2.circle(res_image, (u, v), 10, (255, 0, 0), thickness=-1)

        out_v_val = output[0][0]
        out_u_val = output[0][1]
        out_v = int((1.0 - out_v_val) * res_image.shape[0] / 2.0)
        out_u = int((1.0 - out_u_val) * res_image.shape[1] / 2.0)

        res_image = cv2.circle(res_image, (out_u, out_v), 10, (0, 0, 255), thickness=-1)

        cv2.imshow('data_view', res_image)
        key = cv2.waitKey(30)
        print(key)
        if key == 115:
            torch.save(model, 'model_weight.pth')
            break
        elif key == 113:
            break

