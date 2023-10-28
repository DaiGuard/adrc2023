import os
from torch import Tensor
from torch.utils.data import Dataset
from torchvision.io import read_image

import cv2
import PIL.Image


class XYDataset(Dataset):

    def __init__(self, paths, transform=None):
        self.categories = ['throttle', 'steering']
        self.transform = transform

        dataset = []
        for path in paths:
            data_file = os.path.join(path, 'data.csv')            
            with open(data_file, 'r') as f:
                for line in f:
                    d = line.replace(' ', '').split(',')
                    dataset.append({
                        'file': os.path.join(path, d[0]),
                        'throttle': float(d[1]), 
                        'steering': float(d[2])
                        })

        self.dataset = dataset


    def __len__(self):
        return len(self.dataset)

    def __getitem__(self, index):
        image_path = self.dataset[index]['file']
        throttle = self.dataset[index]['throttle']
        steering = self.dataset[index]['steering']
        
        image = cv2.imread(image_path)

        if self.transform:
            image = self.transform(image)

        return image, Tensor([throttle, steering])
    
    def getData(self, index):
        image_path = self.dataset[index]['file']
        throttle = self.dataset[index]['throttle']
        steering = self.dataset[index]['steering']        
        image = cv2.imread(image_path)

        return image, (throttle, steering)


    