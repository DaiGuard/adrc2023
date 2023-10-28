import os
import sys

import torch
import torchvision
import torchvision.transforms as transforms
from torch.utils.data import DataLoader

from torch2trt import torch2trt
from xy_dataset import XYDataset

import cv2

if __name__ == '__main__':

    device = torch.device('cuda')
    model = torchvision.models.resnet18(pretrained=False)
    model.fc = torch.nn.Linear(512, 2)
    model = model.to(device)

    TRANSFORMS = transforms.Compose([
        transforms.ColorJitter(0.2, 0.2, 0.2, 0.2),
        transforms.Resize((224, 224)),
        transforms.ToTensor(),
        transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
    ])

    # 引数取得
    args = sys.argv 

    # 初期の値を取得
    dataset = XYDataset(args[1], TRANSFORMS)
    epoch = int(args[2])
    optimizer = torch.optim.Adam(model.parameters())
    batch_size = int(args[3])

    train_loader = DataLoader(dataset, batch_size=batch_size, shuffle=True)    

    # トレーニング
    model = model.train()
    while epoch > 0:
        count = 0
        sum_loss = 0.0
        for images, xy in iter(train_loader):
            images = images.to(device)
            xy = xy.to(device)

            optimizer.zero_grad()

            outputs = model(images)
            
            loss = 0.0
            for i in range(len(images)):
                loss += torch.mean((outputs[i] - xy[i])**2)
            loss.backward()
            optimizer.step()

            sum_loss += float(loss)
            count += len(images)
            
            print(f'[{epoch}] {count}/{len(dataset)}: sum_loss={sum_loss/count}, loss={float(loss)}')
        
        epoch -= 1

    # 評価
    model = model.eval()

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
            output = model(image)
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

# data = torch.zeros((1, 3, 224, 224)).cuda().half()
# model_trt = torch2trt(model, [data], fp16_mode=True)
# model.load_state_dict(torch.load('road_following_model.pth'))

