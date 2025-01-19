import rclpy
import sys
import threading
import time

def test_function(a_node):
    start = time.time()
    period = 0.001 # 1 ms
    duration = 10  # seconds
    samples = duration / period
    sleep_rate = a_node.create_rate(1.0 / period)
    print(f'{samples} samples')
    for i in range(int(samples)):
        sleep_rate.sleep()

    actual_duration = time.time() - start
    print('servo_jp complete in %2.2f seconds (expected %2.2f)' % (actual_duration, duration))

    
if __name__ == '__main__':
    rclpy.init(args = sys.argv[1:])
    test_node =  rclpy.create_node('test_node')

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(test_node)
    executor_thread = threading.Thread(target = executor.spin, daemon = True)
    executor_thread.start()

    test_function(test_node)

    rclpy.shutdown()
    executor_thread.join()
