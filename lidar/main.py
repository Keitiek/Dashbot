import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanSubscriber(Node):
    def __init__(self):
        super().__init__('scan_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10  # QoS profile depth
        )
        self.subscription  # prevent unused variable warning

    def scan_callback(self, msg):
        # Process LaserScan message here
        # For example, print the ranges
        print("Received scan data:")
        print("Minimum range value", msg.range_min)
        print("Maximum range value", msg.range_max)
        
        print("Array Length", len(msg.ranges))
        
        # Get array length
        array_len = len(msg.ranges)

        # Divide the length by four
        divided_by_four = array_len // 4

        # First quarter
        first_quarter = msg.ranges[:divided_by_four]

        # Last quarter
        last_quarter = msg.ranges[-divided_by_four:]

        print("Array Length:", array_len)
        print("First quarter:", len(first_quarter))
        print("Last quarter:", len(last_quarter))

        # Get values from 0 to the end of the first quarter
        for i, range_value in enumerate(first_quarter):
            print(f"Range {i}: {range_value}")
            if range_value <= 0.3:
                print("Stop")

        # Get values from the end of the last quarter back to 0
        for i, range_value in enumerate(reversed(last_quarter)):
            adjusted_index = array_len - divided_by_four + i
            print(f"Range {adjusted_index}: {range_value}")
            if range_value <= 0.3:
                print("Stop")

def main(args=None):
    rclpy.init(args=args)
    node = ScanSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__': 
    main()
