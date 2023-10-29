import os
import sys
from glob import glob
import cv2


if __name__ == '__main__':

    args = sys.argv
    if len(args) < 1:
        print('Usage:')
        print('  python data_view.py <DATAPATH>')
        print('')
        print('  DATAPATH: data folder path')
        exit()

    image_files = glob(os.path.join(args[1], '*.jpg'))
    data_file = os.path.join(args[1], 'data.csv')

    dataset = []
    with open(data_file, 'r') as f:
        for line in f:
            d = line.replace(' ', '').split(',')
            dataset.append(d)

    cv2.namedWindow('data_view', cv2.WINDOW_NORMAL)
    cv2.createTrackbar('ID', 'data_view', 0, len(dataset)-1, lambda val: print(val))

    id = -1

    while True:
        current_id = cv2.getTrackbarPos('ID', 'data_view')
        if current_id != id:
            id = current_id
            image = cv2.imread(os.path.join(args[1], dataset[id][0]))

            image = cv2.rectangle(image, (150, 260), (490, 360), (0, 0, 0), thickness=-1)

        v_val = float(dataset[id][1])
        u_val = float(dataset[id][2])

        v = int((1.0 - v_val) * image.shape[0] / 2.0)
        u = int((1.0 - u_val) * image.shape[1] / 2.0)

        image = cv2.circle(image, (u, v), 10, (255, 0, 0), thickness=-1)

        cv2.imshow('data_view', image)
        key = cv2.waitKey(30)
        if key == 113:
            break

    cv2.destroyAllWindows()
