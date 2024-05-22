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
        array_length = 16
        array = [0] * array_length
       # print(array)
        
        for i, range_value in enumerate(msg.ranges):
            if range_value <= 1.0:
                index = int(i * array_length / len(msg.ranges))  # calculating index, what sector the range will go
                print(index)
                if index == 3 or index == 2 or index == 12 or index == 13:
                    if range_value <= 0.3:
                        array[index] = True
                if index == 1 or index == 0 or index == 15 or index == 14: 
                    if range_value <= 1.0:
                        array[index] = True
                    
        #print(array)
        eight_array = [array[3], array[2], array[1], array[0], array[15],array[14], array[13], array[12]]
        print(eight_array)
              

def main(args=None):
    rclpy.init(args=args)
    node = ScanSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__': 
    main()
