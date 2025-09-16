# Chrony Time Synchronization Setup for ROS 2 SLAM

## Overview

This document describes the setup and troubleshooting of chrony time synchronization between a ROS 2 robot and development machine to eliminate timestamp misalignment issues in SLAM applications.

## Problem Statement

### Issues Before Chrony Setup
- **SLAM mapping failures** due to temporal inconsistencies
- **LiDAR point clouds misaligned** with odometry data
- **Clock drift** between robot and development machine
- **Timestamp rejection** by SLAM Toolbox
- **Inconsistent map building** and localization

### Root Cause
Different system clocks on robot and dev machine causing:
- LiDAR sensor timestamps from robot
- SLAM processing timestamps from dev machine
- Time drift causing data correlation failures

## Solution: Chrony Time Synchronization

### Architecture
```
Internet NTP Servers
        ↓
    Robot (Pi) ←→ WiFi Network ←→ Development Machine
        ↓                              ↓
   Local NTP Server              Chrony Client
   (192.168.x.x)                (syncs to robot)
```

## Current Network Configuration

### Robot IP Addresses
- **Primary Network**: `192.168.1.136` (current active)
- **Secondary Network**: `192.168.68.135` (backup)
- **ROS Domain ID**: `0` (both devices)

### Chrony Status (Working Configuration)
```bash
# Time precision achieved
System time: 0.000330202 seconds slow of NTP time
RMS offset: 0.000602437 seconds

# Status indicators
✓ Excellent precision: <1ms offset
✓ Stable RMS offset: <1ms
✓ Robot configured as time source
✓ Multiple redundant NTP sources
```

## Setup Instructions

### 1. Robot Configuration (Raspberry Pi)
```bash
# Install chrony
sudo apt update
sudo apt install chrony

# Configure as NTP server + client
sudo nano /etc/chrony/chrony.conf

# Add these lines:
server pool.ntp.org iburst
server time.nist.gov iburst
allow 192.168.1.0/24
allow 192.168.68.0/24
local stratum 10

# Enable and start service
sudo systemctl enable chronyd
sudo systemctl start chronyd
sudo systemctl restart chronyd

# Open firewall (if enabled)
sudo ufw allow 123/udp
```

### 2. Development Machine Configuration
```bash
# Install chrony
sudo apt update
sudo apt install chrony

# Configure to sync with robot + internet
sudo nano /etc/chrony/chrony.conf

# Add these lines (robot IPs with higher priority):
server 192.168.1.136 prefer iburst
server 192.168.68.135 prefer iburst
server pool.ntp.org iburst
server time.nist.gov iburst

# Enable and start service
sudo systemctl enable chronyd
sudo systemctl start chronyd
sudo systemctl restart chronyd
```

### 3. Verification
```bash
# Run the test script
python3 /home/ubuntu/dev_ws/src/pharma_bot/scripts/test_time_sync.py

# Manual verification
chronyc sources -v
chronyc tracking
ping <robot_ip>
```

## Troubleshooting Guide

### Common Issues and Solutions

#### 1. Robot Time Source Shows "?" (Unusable)
**Symptoms:**
```
^? 192.168.1.136    0   8     0     -     +0ns[   +0ns] +/-    0ns
```

**Causes & Solutions:**
- **Network connectivity**: `ping <robot_ip>` to verify connection
- **Robot chrony not running**: SSH to robot, `sudo systemctl start chronyd`
- **Firewall blocking**: Robot needs UDP port 123 open
- **Wrong IP address**: Update chrony.conf with current robot IP

#### 2. Poor Time Precision (>10ms offset)
**Symptoms:**
```
System time: 0.050000000 seconds fast of NTP time
```

**Solutions:**
- Check network latency: `ping -c 10 <robot_ip>`
- Restart chrony: `sudo systemctl restart chronyd`
- Wait for convergence (can take 5-10 minutes)
- Verify multiple time sources are available

#### 3. ROS Timestamp Misalignment
**Symptoms:**
- SLAM warnings about old/future timestamps
- Map building inconsistencies

**Verification:**
```bash
# Check topic timestamps
ros2 topic echo /ldlidar_node/scan --once
# Timestamp should be current time ±1 second

# Check time synchronization
chronyc tracking | grep "System time"
```

#### 4. Network Switching Issues
**Problem**: Robot changes networks, chrony can't reach time source

**Solution**: Multi-network configuration already implemented
```
server 192.168.1.136 prefer iburst    # Network 1
server 192.168.68.135 prefer iburst   # Network 2  
```

### Monitoring Commands

```bash
# Check chrony status
systemctl status chronyd

# View time sources and their status
chronyc sources -v

# Check current synchronization
chronyc tracking

# Monitor time offset over time
watch -n 5 'chronyc tracking | grep "System time"'

# Test network connectivity to robot
ping -c 3 192.168.1.136
ping -c 3 192.168.68.135
```

## Status Indicators Explanation

### Chrony Sources Status
- `*` = Current best time source (actively used)
- `+` = Combined/backup source (good quality)
- `-` = Not combined (acceptable but not used)
- `?` = Unusable (unreachable or poor quality)
- `x` = May be in error
- `~` = Too variable

### Time Precision Guidelines
- **Excellent**: <1ms offset (perfect for SLAM)
- **Good**: 1-10ms offset (acceptable for most robotics)
- **Poor**: >10ms offset (may cause SLAM issues)

## Integration with Launch Files

### No Changes Required
- Chrony runs as **system service** (persistent)
- **Automatic startup** on boot
- **No restart needed** when launching ROS files
- Works with existing launch files:
  - `pi_test_launch.py` (robot)
  - `dev_test_launch.py` (dev machine)
  - `dev_slam_launch.py` (SLAM processing)

### Expected Benefits
After chrony setup, SLAM should show:
- ✅ **No timestamp warnings** in logs
- ✅ **Consistent map building**
- ✅ **Stable point cloud alignment**
- ✅ **Reliable localization**

## Configuration Files

### Robot /etc/chrony/chrony.conf
```
# Internet time sources
server pool.ntp.org iburst
server time.nist.gov iburst

# Act as local NTP server
allow 192.168.1.0/24
allow 192.168.68.0/24
local stratum 10

# Other standard chrony settings
driftfile /var/lib/chrony/chrony.drift
makestep 1.0 3
rtcsync
```

### Dev Machine /etc/chrony/chrony.conf
```
# Robot time sources (preferred)
server 192.168.1.136 prefer iburst
server 192.168.68.135 prefer iburst

# Internet backup sources
server pool.ntp.org iburst
server time.nist.gov iburst

# Standard chrony settings
driftfile /var/lib/chrony/chrony.drift
makestep 1.0 3
rtcsync
```

## Quick Setup for New IP Address

### If Robot Gets New IP (e.g., 192.168.2.100):

1. **Update dev machine chrony config:**
```bash
sudo nano /etc/chrony/chrony.conf
# Add line: server 192.168.2.100 prefer iburst
sudo systemctl restart chronyd
```

2. **Update robot chrony config (if needed):**
```bash
# SSH to robot
sudo nano /etc/chrony/chrony.conf  
# Add line: allow 192.168.2.0/24
sudo systemctl restart chronyd
```

3. **Update test script:**
```bash
# Edit robot_ips list in test_time_sync.py
robot_ips = ["192.168.68.135", "192.168.1.136", "192.168.2.100"]
```

4. **Verify setup:**
```bash
python3 /home/ubuntu/dev_ws/src/pharma_bot/scripts/test_time_sync.py
```

## Performance Metrics

### Target Values
- **Time offset**: <1ms (excellent), <10ms (acceptable)
- **RMS offset**: <1ms (stable synchronization)
- **Network latency**: <50ms (good), <100ms (acceptable)
- **Reach**: 377 (octal) = perfect reachability

### Current Achieved Performance
- ✅ **Time offset**: 0.33ms (excellent)
- ✅ **RMS offset**: 0.6ms (excellent)
- ✅ **Network connectivity**: Active to 192.168.1.136
- ✅ **Service status**: Running and persistent

## Notes

- **Persistent Service**: Chrony starts automatically on boot, no manual intervention needed
- **Network Resilient**: Automatically switches between available robot IP addresses
- **Backup Sources**: Falls back to internet NTP if robot unavailable
- **ROS Agnostic**: Works with any ROS 2 setup, no changes to launch files needed
- **Low Overhead**: Minimal CPU/network usage once synchronized

---

*Document created: 2025-09-16*  
*Last tested with: Robot IP 192.168.1.136, Dev machine Ubuntu 22.04*
