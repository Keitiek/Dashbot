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
        print("Received scan data:")
        print("Minimum range value", msg.range_min)
        print("Maximum range value", msg.range_max)

        print("Array Length", len(msg.ranges))

        # Get array length
        array_len = len(msg.ranges)

        # Divide the length by eight (for 8 segments)
        divided_by_eight = array_len // 8

        # Get the data for 0 to 45 degrees
        first_quarter = msg.ranges[:divided_by_eight]

        # Get the data for 45 to 90 degrees
        second_quarter_start = divided_by_eight
        second_quarter_end = 2 * divided_by_eight
        second_quarter = msg.ranges[second_quarter_start:second_quarter_end]

        # Get the data for 270 to 315 degrees
        seventh_quarter_start = 6 * divided_by_eight
        seventh_quarter_end = 7 * divided_by_eight
        seventh_quarter = msg.ranges[seventh_quarter_start:seventh_quarter_end]

        # Get the data for 315 to 360 degrees
        last_quarter = msg.ranges[-divided_by_eight:]

        print("Array Length:", array_len)
        print("First quarter:", len(first_quarter))
        print("Second quarter:", len(second_quarter))
        print("Seventh quarter:", len(seventh_quarter))
        print("Last quarter:", len(last_quarter))

        # Print "Stop" for ranges 0-45 degrees if distance is <= 1.0m
        for i, range_value in enumerate(first_quarter):
            print(f"Range {i}: {range_value}")
            if range_value <= 1.0:
                print("Stop  0-45 ")
                print(" 1m ")

        # Print "Stop" for ranges 45-90 degrees if distance is <= 0.3m
        for i, range_value in enumerate(second_quarter):
            adjusted_index = second_quarter_start + i
            print(f"Range {adjusted_index}: {range_value}")
            if range_value <= 0.3:
                print("Stop 45-90")
                print(" 30cm ")


        # Print "Stop" for ranges 270-315 degrees if distance is <= 0.3m
        for i, range_value in enumerate(seventh_quarter):
            adjusted_index = seventh_quarter_start + i
            print(f"Range {adjusted_index}: {range_value}")
            if range_value <= 0.3:
                print("Stop 270-315")
                print(" 30cm ")


        # Print "Stop" for ranges 315-360 degrees if distance is <= 1.0m
        for i, range_value in enumerate(last_quarter):
            adjusted_index = array_len - divided_by_eight + i
            print(f"Range {adjusted_index}: {range_value}")
            if range_value <= 1.0:
                print("Stop 315-360")
                print(" 1m ")



def main(args=None):
    rclpy.init(args=args)
    node = ScanSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__': 
    main()
