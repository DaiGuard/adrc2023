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
            d = line.replace(' ', '').replace('\n', '').split(',')

            if len(d) < 7:
                d = d + ['0'] * (7 - len(d))
            
            if not d[6]:
                d[6] = '0'
            elif int(d[6]) ==  0 or int(d[6]) ==  1:
                pass
            else:
                d[6] = '0'
            
            dataset.append(d)

    cv2.namedWindow('data_view', cv2.WINDOW_NORMAL)
    cv2.createTrackbar('ID', 'data_view', 0, len(dataset)-1, lambda val: print(val))

    id = -1

    try:
        while True:
            current_id = cv2.getTrackbarPos('ID', 'data_view')
            if current_id != id:
                id = current_id
                image = cv2.imread(os.path.join(args[1], dataset[id][0]))
            
            preview = image.copy()
            preview = cv2.rectangle(preview, (150, 260), (490, 360), (0, 0, 0), thickness=-1)
            if int(dataset[id][6]) > 0:
                cv2.putText(preview, 'Enable', (10, 20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, 
                            (255,0,0), 2, cv2.LINE_4)
            else:
                cv2.putText(preview, 'Disable', (10, 20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, 
                            (0,0,255), 2, cv2.LINE_4)
                
            cv2.putText(preview, '{:.2f}, {:.2f}'.format(float(dataset[id][1]), float(dataset[id][2])),
                            (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1,
                            (0, 0, 0), 2, cv2.LINE_4)

            v_val = float(dataset[id][1])
            u_val = float(dataset[id][2])

            v = int((1.0 - v_val) * image.shape[0] / 2.0)
            u = int((1.0 - u_val) * image.shape[1] / 2.0)

            preview = cv2.circle(preview, (u, v), 10, (255, 0, 0), thickness=-1)

            cv2.imshow('data_view', preview)
            key = cv2.waitKey(30)        
            print(key, id)
            if key == 83:
                if id < len(dataset) - 1:
                    cv2.setTrackbarPos('ID', 'data_view', id+1)            
            elif key == 81:
                if id >= 0:
                    cv2.setTrackbarPos('ID', 'data_view', id-1)            
            elif key == 101:
                dataset[id][6] = 1
            elif key == 100:
                dataset[id][6] = 0
            elif key == 113:
                break
    except KeyboardInterrupt:
        pass

    key = input('save [Y/n]: ')
    if key == 'Y':
        with open(os.path.join(args[1], 'data.csv'), 'w') as f:
            for d in dataset:
                line = ''
                for i in range(7):
                    line += f'{d[i]}, '
                line += '\n'

                f.write(line)

    cv2.destroyAllWindows()
