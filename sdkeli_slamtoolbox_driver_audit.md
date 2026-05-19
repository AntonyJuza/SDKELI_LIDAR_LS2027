# SDKELI Lidar ROS2 Package Audit for SLAM Toolbox

I checked your custom package `sdkeli_ls_udp`.

The good news:
- Your package already publishes `sensor_msgs/LaserScan`
- Timestamp support already exists
- Angle/range fields are present
- Intensities are supported
- Frame ID support exists
- SLAM Toolbox can probably run with this driver already

But there are a few important problems and missing parts that can cause:
- bad localization
- map distortion
- delayed scans
- TF issues
- unstable scan matching
- poor timing synchronization

---

# 1. Timestamp Actually EXISTS

In:

`src/sdkeli_ls1207de_parser.cpp`

You already have:

```cpp
rclcpp::Time stamp = rclcpp::Clock().now();
msg.header.stamp = stamp;
```

So the scan IS timestamped.

But the implementation is not ideal for SLAM.

Current code:

```cpp
rclcpp::Time stamp = rclcpp::Clock().now();
stamp = stamp - rclcpp::Duration::from_seconds(scan_duration);
```

Problems:

- Uses raw system clock instead of node clock
- Can create time mismatch with `/use_sim_time`
- Slam Toolbox prefers synchronized ROS clock
- Multi-sensor fusion may drift

---

# Recommended Fix

Pass node clock into parser OR stamp in publisher layer.

Better:

```cpp
msg.header.stamp = node_->now();
```

OR:

```cpp
rclcpp::Clock ros_clock(RCL_ROS_TIME);
msg.header.stamp = ros_clock.now();
```

---

# 2. Missing QoS for Sensor Data

Current publisher:

```cpp
node_->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
```

This uses default QoS.

For SLAM Toolbox and Nav2, sensor QoS is strongly recommended.

---

# Recommended Fix

Replace with:

```cpp
scan_publisher_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(
    "scan",
    rclcpp::SensorDataQoS());
```

This is VERY important.

Without this:
- dropped scans
- delayed scans
- RViz mismatch
- slam_toolbox scan lag

can happen.

---

# 3. Missing Parameters Needed for SLAM Tuning

Your package currently exposes:

```cpp
range_min
range_max
time_increment
frame_id
```

But SLAM usually also needs:

## Recommended Parameters

### angle_min

```cpp
node->declare_parameter<double>("angle_min", -2.35619);
```

### angle_max

```cpp
node->declare_parameter<double>("angle_max", 2.35619);
```

### scan_time

```cpp
node->declare_parameter<double>("scan_time", 0.043);
```

### inverted

Useful if scans appear mirrored.

```cpp
node->declare_parameter<bool>("inverted", false);
```

### publish_intensity

```cpp
node->declare_parameter<bool>("publish_intensity", true);
```

### time_offset

VERY useful for SLAM alignment.

```cpp
node->declare_parameter<double>("time_offset", 0.0);
```

### skip

Already exists internally but not exposed.

```cpp
node->declare_parameter<int>("skip", 0);
```

---

# 4. Hardcoded Timing Values

You currently hardcode:

```cpp
const uint16_t scanning_freq = 1000 / 43 * 100;
time_increment_ = 0.000040;
```

This is dangerous.

Because:
- actual lidar frequency may vary
- UDP latency varies
- motor RPM may drift
- slam_toolbox depends heavily on timing consistency

---

# Recommended Fix

Make them parameters.

Example:

```cpp
node->declare_parameter<double>("scan_frequency", 23.0);
```

Then:

```cpp
msg.scan_time = 1.0 / scan_frequency;
msg.time_increment = msg.scan_time / data_count;
```

This is MUCH cleaner.

---

# 5. Missing Diagnostics

Currently there is:

- no packet loss counter
- no scan frequency monitor
- no corrupted frame statistics
- no latency diagnostics

For SLAM debugging this matters.

At minimum add:

```cpp
RCLCPP_WARN_THROTTLE(...)
```

for:
- dropped packets
- invalid checksum
- skipped frames
- inconsistent scan sizes

---

# 6. Missing TF Validation

Your driver publishes scan only.

That is OK.

But SLAM Toolbox REQUIRES:

```text
base_link -> laser
```

TF transform.

If missing:
- map rotates
- scans drift
- localization breaks

Verify:

```bash
ros2 run tf2_ros tf2_echo base_link laser
```

---

# 7. Angle Calculation Looks Suspicious

You use:

```cpp
const int starting_angle = 0xFFF92230;
```

This is treated as signed but stored as int.

Could cause:
- mirrored scans
- rotated scans
- wrong angle_min

You should verify actual values.

---

# Better Method

Use explicit signed type:

```cpp
const int32_t starting_angle = static_cast<int32_t>(0xFFF92230);
```

Then verify output.

---

# 8. Missing Frame Validation

SLAM Toolbox expects:

- stable scan size
- stable angle_increment
- stable timing

Your code does not validate:

```cpp
output_size
angle_increment
scan_time
```

Add checks.

---

# 9. Recommended Launch Parameters for SLAM Toolbox

Recommended:

```yaml
sdkeli_ls1207de:
  ros__parameters:
    frame_id: laser
    range_min: 0.05
    range_max: 12.0
    scan_frequency: 23.0
    time_offset: 0.0
    skip: 0
```

---

# 10. MOST IMPORTANT FIXES

If you only do 5 things, do these:

## Critical

### 1. Use SensorDataQoS

```cpp
rclcpp::SensorDataQoS()
```

### 2. Use node clock

```cpp
msg.header.stamp = node_->now();
```

### 3. Add proper TF

```text
base_link -> laser
```

### 4. Remove hardcoded timing

Make scan frequency parameterized.

### 5. Verify angle direction

Check in RViz.

---

# Final Verdict

Your package is already surprisingly close to working with SLAM Toolbox.

Main issue is NOT missing LaserScan fields.

Main issue is:

- timing quality
- QoS
- TF correctness
- hardcoded scan timing
- angle validation

Those are the things that usually break SLAM.

The package architecture itself is fine.

