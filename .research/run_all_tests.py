#!/usr/bin/env python3
"""
Run all boundary condition verification tests and report results.
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))
from test_b6_rtt_asymmetry import test as test_b6
from test_b7_b8_bandwidth import test as test_b7_b8
from test_b9_random_loss import test as test_b9
from test_b19_ack_jitter import test as test_b19
from test_b29_reordering import test as test_b29


def main():
    print()
    print("=" * 72)
    print("GEODESIC BOUNDARY CONDITION VERIFICATION")
    print("Source: tcp_kcc.c SECTION 5 (B1-B51)")
    print("=" * 72)

    tests = [
        ("B6", "RTT Asymmetry", test_b6),
        ("B7/B8", "Bandwidth Drop/Increase", test_b7_b8),
        ("B9", "Random Packet Loss", test_b9),
        ("B19", "ACK Timing Jitter", test_b19),
        ("B29", "Packet Reordering", test_b29),
    ]

    results = {}
    total_start = time.time()

    for bid, _bname, bfunc in tests:
        start = time.time()
        try:
            passed = bfunc()
            elapsed = time.time() - start
            results[bid] = (passed, elapsed)
        except Exception as e:
            elapsed = time.time() - start
            print()
            print(f"  ERROR in {bid}: {e!s}")
            results[bid] = (False, elapsed)

    total_elapsed = time.time() - total_start

    print()
    print("=" * 72)
    print("SUMMARY")
    print("=" * 72)
    passed_count = sum(1 for v in results.values() if v[0])
    total_count = len(results)
    print()
    print("  Passed: %d/%d" % (passed_count, total_count))
    print(f"  Total time: {total_elapsed:.1f}s")
    print()
    for bid, (passed, elapsed) in sorted(results.items()):
        status = "PASS" if passed else "FAIL"
        print("  %-8s %-6s  %.1fs" % (bid, status, elapsed))

    print()
    return all(v[0] for v in results.values())


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
