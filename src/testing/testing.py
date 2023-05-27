#!/usr/bin/env python
import time

def test(value, expected):
    if value == expected:
        print("PASS")
        return True
    else:
        print(f"FAIL: expected {expected}, got {value}")
        return False
    
def stress_test(function, n):
    start_time = time.time()
    errors = 0
    for i in range(n):
        try:
            function()
        except AssertionError:
            errors += 1

    total_time = time.time() - start_time
    print(f"total time: {total_time * 1000:.2f} ms, avg time: {total_time / n * 1000:.2f} ms, success rate: {100 * (1 - errors / n):.2f}%")

def interval_test(function, interval_time):
    while True:
        function()
        time.sleep(interval_time)