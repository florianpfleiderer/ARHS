#!/usr/bin/env python

def test(value, expected):
    if value == expected:
        print("PASS")
    else:
        print(f"FAIL: expected {expected}, got {value}")