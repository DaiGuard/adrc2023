import rclpy
from data_capture.data_capture_class import DataCapture

def main(args=None):
    rclpy.init(args=args)

    data_capture = DataCapture()
    data_capture.start_pipeline()

    data_capture.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()