#! /usr/bin/env python3

import crtk
import sys
import time

def do_work():
    # fake example
    pass

def rate_example(ral):
    start = time.time()
    period = 0.001 # 1 ms
    duration = 10  # seconds
    samples = duration / period
    sleep_rate = ral.create_rate(1.0 / period)
    print(f'Sleeping for {period} seconds, {samples} times')

    for _ in range(int(samples)):
        do_work()
        sleep_rate.sleep()

    actual_duration = time.time() - start
    print(f'Completed in {actual_duration:2.2f} seconds (expected {duration:2.2f})')

    
if __name__ == '__main__':
    crtk.ral.parse_argv(sys.argv[1:])
    ral = crtk.ral('crtk_rate_example')
    ral.spin_and_execute(lambda: rate_example(ral))
