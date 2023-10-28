import rclpy
from adrc_ros.live_run_class import LiveRun

def main(args=None):
    rclpy.init(args=args)

    live_run = LiveRun()
    live_run.start_pipeline()

    live_run.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()