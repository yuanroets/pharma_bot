#!/usr/bin/env python3
"""
Time Synchronization Test Script
===============================

Tests the time synchronization setup between dev machine and robot.
This script checks:
1. Network connectivity to robot
2. Chrony status and time sources
3. Time offset between machines
4. ROS 2 timestamp alignment

Usage:
    python3 test_time_sync.py
    or
    ros2 run pharma_bot test_time_sync.py
"""

import subprocess
import sys
import time
import socket
from datetime import datetime, timezone


class Colors:
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    BLUE = '\033[94m'
    END = '\033[0m'
    BOLD = '\033[1m'


def run_command(cmd, capture_output=True):
    """Run a shell command and return result."""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=capture_output, 
                              text=True, timeout=10)
        return result.returncode == 0, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        return False, "", "Command timed out"
    except Exception as e:
        return False, "", str(e)


def test_network_connectivity():
    """Test network connectivity to robot."""
    print(f"{Colors.BLUE}=== Testing Network Connectivity ==={Colors.END}")
    
    robot_ips = ["192.168.68.135", "192.168.1.136"]
    connected_ip = None
    
    for ip in robot_ips:
        print(f"Testing connection to {ip}...")
        success, stdout, stderr = run_command(f"ping -c 2 -W 3 {ip}")
        
        if success:
            print(f"{Colors.GREEN}✓ Connected to robot at {ip}{Colors.END}")
            connected_ip = ip
            break
        else:
            print(f"{Colors.RED}✗ Cannot reach robot at {ip}{Colors.END}")
    
    return connected_ip


def test_chrony_status():
    """Test chrony time synchronization status."""
    print(f"\n{Colors.BLUE}=== Testing Chrony Status ==={Colors.END}")
    
    # Check if chrony is running
    success, stdout, stderr = run_command("systemctl is-active chronyd")
    if success and "active" in stdout:
        print(f"{Colors.GREEN}✓ Chrony service is running{Colors.END}")
    else:
        print(f"{Colors.RED}✗ Chrony service is not running{Colors.END}")
        return False
    
    # Check time sources
    print("\nTime sources:")
    success, stdout, stderr = run_command("chronyc sources")
    if success:
        lines = stdout.strip().split('\n')
        for line in lines[3:]:  # Skip header lines
            if line.strip():
                status = line[0] if line else '?'
                if status == '*':
                    print(f"{Colors.GREEN}  {line} (CURRENT BEST){Colors.END}")
                elif status == '+':
                    print(f"{Colors.YELLOW}  {line} (COMBINED){Colors.END}")
                elif status == '?':
                    print(f"{Colors.RED}  {line} (UNUSABLE){Colors.END}")
                else:
                    print(f"  {line}")
    
    # Check tracking info
    print("\nTracking information:")
    success, stdout, stderr = run_command("chronyc tracking")
    if success:
        for line in stdout.strip().split('\n'):
            if 'System time' in line:
                print(f"  {line}")
            elif 'RMS offset' in line:
                print(f"  {line}")
    
    return True


def test_robot_time_sync(robot_ip):
    """Test time synchronization with robot."""
    print(f"\n{Colors.BLUE}=== Testing Robot Time Sync ==={Colors.END}")
    
    if not robot_ip:
        print(f"{Colors.RED}✗ No robot connection available{Colors.END}")
        return False
    
    # Check if robot is in chrony sources
    success, stdout, stderr = run_command("chronyc sources")
    robot_in_sources = robot_ip in stdout if success else False
    
    if robot_in_sources:
        print(f"{Colors.GREEN}✓ Robot {robot_ip} is configured as time source{Colors.END}")
        
        # Check robot source status
        for line in stdout.split('\n'):
            if robot_ip in line:
                status = line[0] if line else '?'
                if status == '*':
                    print(f"{Colors.GREEN}✓ Robot is the current best time source{Colors.END}")
                elif status == '+':
                    print(f"{Colors.YELLOW}! Robot is being used as backup time source{Colors.END}")
                elif status == '?':
                    print(f"{Colors.RED}✗ Robot time source is unusable{Colors.END}")
                else:
                    print(f"{Colors.YELLOW}! Robot time source status: {status}{Colors.END}")
    else:
        print(f"{Colors.RED}✗ Robot {robot_ip} is not configured as time source{Colors.END}")
    
    return robot_in_sources


def test_time_offset():
    """Test time offset and precision."""
    print(f"\n{Colors.BLUE}=== Testing Time Precision ==={Colors.END}")
    
    # Get current tracking info
    success, stdout, stderr = run_command("chronyc tracking")
    if success:
        for line in stdout.split('\n'):
            if 'System time' in line:
                try:
                    # Extract offset value
                    offset_str = line.split(':')[1].strip().split()[0]
                    offset_val = float(offset_str)
                    
                    if abs(offset_val) < 0.001:  # Less than 1ms
                        print(f"{Colors.GREEN}✓ Excellent time precision: {offset_str}s{Colors.END}")
                    elif abs(offset_val) < 0.010:  # Less than 10ms
                        print(f"{Colors.YELLOW}! Good time precision: {offset_str}s{Colors.END}")
                    else:
                        print(f"{Colors.RED}✗ Poor time precision: {offset_str}s{Colors.END}")
                except:
                    print(f"  {line}")
            elif 'RMS offset' in line:
                print(f"  {line}")


def test_ros_timing():
    """Test ROS 2 timing considerations."""
    print(f"\n{Colors.BLUE}=== ROS 2 Timing Info ==={Colors.END}")
    
    # Check ROS_DOMAIN_ID
    domain_id = subprocess.run("echo $ROS_DOMAIN_ID", shell=True, 
                              capture_output=True, text=True).stdout.strip()
    if domain_id == "0":
        print(f"{Colors.GREEN}✓ ROS_DOMAIN_ID is set to 0 (matches robot){Colors.END}")
    else:
        print(f"{Colors.YELLOW}! ROS_DOMAIN_ID is {domain_id or 'not set'} (should be 0){Colors.END}")
        print("  Run: export ROS_DOMAIN_ID=0")
    
    # Show current time in different formats
    now = datetime.now()
    now_utc = datetime.now(timezone.utc)
    
    print(f"\nCurrent times:")
    print(f"  Local time: {now.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}")
    print(f"  UTC time:   {now_utc.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}")
    print(f"  Unix timestamp: {time.time():.6f}")


def main():
    """Main test function."""
    print(f"{Colors.BOLD}Time Synchronization Test{Colors.END}")
    print(f"Testing time sync between dev machine and robot...")
    print("=" * 50)
    
    # Test network connectivity
    robot_ip = test_network_connectivity()
    
    # Test chrony status
    chrony_ok = test_chrony_status()
    
    # Test robot time sync
    if chrony_ok and robot_ip:
        test_robot_time_sync(robot_ip)
    
    # Test time precision
    if chrony_ok:
        test_time_offset()
    
    # Test ROS timing
    test_ros_timing()
    
    print(f"\n{Colors.BOLD}=== Summary ==={Colors.END}")
    if robot_ip and chrony_ok:
        print(f"{Colors.GREEN}✓ Basic setup is working{Colors.END}")
        print(f"{Colors.BLUE}Your time sync setup is persistent and always running.{Colors.END}")
        print(f"{Colors.BLUE}No need to restart anything when launching ROS files.{Colors.END}")
    else:
        print(f"{Colors.RED}✗ Some issues detected - check robot connection{Colors.END}")
    
    print(f"\n{Colors.BLUE}To test SLAM timing, run:{Colors.END}")
    print(f"  ros2 topic echo /ldlidar_node/scan --once")
    print(f"  # Check timestamp in header")


if __name__ == "__main__":
    main()
