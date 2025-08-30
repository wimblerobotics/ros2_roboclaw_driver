// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Michael Wimble. https://github.com/wimblerobotics/ros2_roboclaw_driver

# RoboClaw Driver Improvement Plan v2

## Updated Architectural Review (Post Stage 1 Partial Implementation)

### 1. Concurrency & synchronization (PARTIALLY ADDRESSED)
**Status**: IoExecutor implemented for basic serialization, but full transaction atomicity pending.
- ✅ Added IoExecutor singleton with priority queue (HIGH/NORMAL/LOW operations)
- ✅ cmdVelCallback now enqueues CMD46 operations instead of direct hardware calls
- ✅ Added retry logic with 10ms quiet period per RoboClaw manual
- ⏳ **Remaining**: Full snapshot reads still need serialization; sensor polling refactor incomplete
- ⏳ **Remaining**: DeviceCache not yet implemented for incremental state acquisition

### 2. High‑level architecture (MINIMAL PROGRESS)
**Status**: Layers still collapsed; IoExecutor provides foundation but separation incomplete.
- ✅ Basic I/O serialization layer introduced
- ❌ Driver node still mixes ROS interfaces, motion conversion, safety, telemetry
- ❌ No clear abstraction boundary between hardware interface and business logic
- ❌ Safety state machine remains informal

### 3. Protocol & command handling (ADDRESSED)
**Status**: Maintained existing cmd 46 dual-distance framing as requested.
- ✅ Kept 21-byte payload format with buffer=1 override semantics
- ✅ IoExecutor handles command deduplication (latest CMD46 replaces pending)
- ✅ Added latency measurement infrastructure (enqueue timestamp tracking)

### 4. Odometry (NOT IMPLEMENTED)
**Status**: Still empty odomTimer(); integration deferred to Stage 2.
- ❌ No encoder delta computation with signed wrap handling
- ❌ No differential drive integration (x,y,yaw pose tracking)
- ❌ No odom message publishing despite configuration flags
- ❌ No covariance matrix population

### 5. Parameter usage & performance (PARTIAL)
**Status**: Some caching added, but hot-path optimization incomplete.
- ✅ Basic parameter validation on pulses_per_meter (fail-fast if zero)
- ⏳ **Remaining**: Cache immutable parameters (max speeds, accel, timeouts) in hot paths
- ⏳ **Remaining**: Replace repeated get_parameter() calls with cached struct

### 6. Safety & estop (BASIC STRUCTURE)
**Status**: Auto-clear logic added; refined detection algorithms pending.
- ✅ Auto-clear estop on zero cmd_vel receipt
- ❌ Runaway detection still uses fragile baseline heuristic
- ❌ No exponential moving average for overcurrent
- ❌ No formal state machine (RUN → MONITORING → ESTOP transitions)

### 7. Telemetry & diagnostics (MINIMAL)
**Status**: Basic stats infrastructure; comprehensive counters pending.
- ✅ Added IoExecutor operation statistics logging (5s intervals)
- ❌ cmd_stats_ still not wired to published counters
- ❌ No diagnostic_msgs integration
- ❌ Logic battery voltage = 0.0 still reported without benign classification

### 8. Logging & observability (IMPROVED)
**Status**: Structured logging foundation added.
- ✅ Minimal SPDX headers applied to key sources
- ✅ IoExecutor logs operations with timing and retry info
- ✅ Added frame_trace parameter (default false) for debugging
- ⏳ **Remaining**: Remove stderr fprintf in favor of unified rclcpp logging

### 9. Thread timing & latency (FOUNDATION READY)
**Status**: Infrastructure for <20ms latency added; full optimization pending.
- ✅ IoExecutor provides immediate CMD46 dispatch (high priority)
- ✅ Latency measurement from enqueue to ACK completion
- ⏳ **Remaining**: Incremental sensor polling (5ms chunks) instead of bulk snapshot
- ⏳ **Remaining**: Single control loop driving odom + joint states at 50Hz

### 10. Command shaping & distance limiting (MAINTAINED)
**Status**: Existing dual-distance algorithm preserved per requirements.
- ✅ 21-byte cmd 46 format maintained
- ⏳ **Remaining**: Per-wheel distance computation for turning accuracy
- ⏳ **Remaining**: Minimum distance scaling with velocity

---

## Implementation Plan: Remaining Stages

### Stage 2: Device Cache & Incremental Sensor Acquisition
**Objective**: Replace bulk snapshot with incremental cached reads; implement proper odometry.

**Tasks**:
1. **DeviceCache Implementation**
   ```cpp
   struct DeviceCache {
     std::mutex mutex_;
     int32_t enc_left_, enc_right_;
     int32_t vel_left_qpps_, vel_right_qpps_;
     float current_m1_, current_m2_;
     float volt_main_, volt_logic_;
     float temp1_, temp2_;
     uint32_t status_bits_;
     std::chrono::steady_clock::time_point timestamps_[8]; // per category
   };
   ```

2. **Incremental Sensor Scheduling**
   - Replace `readSensorGroup()` with individual operation scheduling
   - Rotate through: encoders (HIGH when stale), currents, voltages, temps, status
   - Budget 6ms total per control loop; skip low-priority if time constrained

3. **Odometry Integration**
   ```cpp
   void integrateOdometry() {
     auto cache = getDeviceCache();
     int32_t delta_L = cache.enc_left_ - last_enc_left_;
     int32_t delta_R = cache.enc_right_ - last_enc_right_;
     
     // Handle 32-bit signed wrap
     if (delta_L > 0x7FFFFFFF) delta_L -= 0x100000000;
     if (delta_L < -0x7FFFFFFF) delta_L += 0x100000000;
     // Same for delta_R
     
     double dist_L = delta_L / pulses_per_meter_;
     double dist_R = delta_R / pulses_per_meter_;
     double d_center = (dist_L + dist_R) / 2.0;
     double d_theta = (dist_R - dist_L) / wheel_separation_;
     
     // Midpoint integration
     x_ += d_center * cos(yaw_ + d_theta/2.0);
     y_ += d_center * sin(yaw_ + d_theta/2.0);
     yaw_ = normalizeAngle(yaw_ + d_theta);
     
     publishOdomMessage();
   }
   ```

4. **Joint States from Cache**
   - Replace direct `getM1Encoder()` calls with cached values
   - Add staleness checks; warn if encoders >100ms old

**Acceptance Criteria**:
- Odom message published at 50Hz with <2ms jitter
- Encoder staleness warnings only if cache >150ms old
- No direct hardware calls outside IoExecutor thread

### Stage 3: Safety System Refinement
**Objective**: Implement robust runaway/overcurrent detection with proper state machine.

**Tasks**:
1. **Runaway Detection Enhancement**
   ```cpp
   bool checkRunaway(int32_t cmd_qpps, int32_t meas_qpps) {
     if (abs(cmd_qpps) < runaway_min_cmd_qpps_) return false;
     
     int32_t threshold = runaway_factor_ * std::max(abs(cmd_qpps), runaway_floor_qpps_);
     return abs(meas_qpps) > threshold;
   }
   ```

2. **Exponential Moving Average Overcurrent**
   ```cpp
   void updateCurrentEMA(float instant_current) {
     float alpha = dt_ / (current_tau_ + dt_);
     ema_current_ = alpha * instant_current + (1-alpha) * ema_current_;
     
     bool ema_fault = ema_current_ > overcurrent_limit_;
     bool spike_fault = instant_current > (overcurrent_limit_ + spike_margin_);
     
     if (ema_fault || spike_fault) {
       incrementOvercurrentTimer();
     } else {
       resetOvercurrentTimer();
     }
   }
   ```

3. **Safety State Machine**
   ```cpp
   enum class SafetyState { RUN, MONITORING_RUNAWAY, MONITORING_OVERCURRENT, ESTOP };
   
   void updateSafetyState() {
     switch (safety_state_) {
       case RUN:
         if (detectRunaway()) safety_state_ = MONITORING_RUNAWAY;
         if (detectOvercurrent()) safety_state_ = MONITORING_OVERCURRENT;
         break;
       case MONITORING_RUNAWAY:
         if (runaway_timer_ > runaway_detect_time_) {
           safety_state_ = ESTOP;
           triggerEstop("RUNAWAY");
         }
         break;
       // ... etc
     }
   }
   ```

**Acceptance Criteria**:
- Simulated encoder disconnect triggers runaway estop within configured time
- Injected current spikes trigger overcurrent estop; normal microspikes ignored
- Zero cmd_vel clears all estops and returns to RUN state

### Stage 4: Per-Wheel Distance & Parameter Optimization
**Objective**: Improve turning accuracy and cache hot-path parameters.

**Tasks**:
1. **Dual Per-Wheel Distance Computation**
   ```cpp
   std::pair<uint32_t, uint32_t> computeWheelDistances(
       double v_left_mps, double v_right_mps) {
     double stopping_L = v_left_mps * v_left_mps / (2 * accel_mps2_);
     double stopping_R = v_right_mps * v_right_mps / (2 * accel_mps2_);
     
     double window_L = std::abs(v_left_mps) * cmd_timeout_;
     double window_R = std::abs(v_right_mps) * cmd_timeout_;
     
     uint32_t dist_L = std::max(min_distance_pulses_, 
                               (stopping_L + window_L) * pulses_per_meter_);
     uint32_t dist_R = std::max(min_distance_pulses_,
                               (stopping_R + window_R) * pulses_per_meter_);
     return {dist_L, dist_R};
   }
   ```

2. **Parameter Cache Structure**
   ```cpp
   struct ParameterCache {
     double max_linear_vel_, max_angular_vel_;
     double wheel_separation_, pulses_per_meter_;
     double cmd_timeout_, accel_qpps_;
     double runaway_factor_, overcurrent_limit_;
     bool safety_enabled_, publish_odom_, publish_joint_states_;
     
     void updateFromNode(rclcpp::Node* node);
   };
   ```

3. **Hot-Path Optimization**
   - Replace all `get_parameter()` calls in cmdVelCallback with cache access
   - Update cache only in parameter change callbacks

**Acceptance Criteria**:
- During constant turn commands: inner wheel distance < outer wheel distance
- cmdVelCallback latency <1ms (excluding IoExecutor enqueue)
- Parameter changes propagate within next control cycle

### Stage 5: Diagnostics & Statistics Integration
**Objective**: Wire cmd_stats_ to published counters; add comprehensive health reporting.

**Tasks**:
1. **Statistics Aggregation**
   ```cpp
   void aggregateStats(RoboClawStatus& status) {
     status.crc_error_count = 0;
     status.io_error_count = 0;
     status.retry_count = 0;
     
     for (const auto& cmd_stat : roboclaw_device_->cmd_stats_) {
       status.crc_error_count += cmd_stat.crc_fail;
       status.io_error_count += cmd_stat.io_fail;
       status.retry_count += (cmd_stat.total_attempts - cmd_stat.success_count);
     }
   }
   ```

2. **Health Classification**
   ```cpp
   std::string classifyHealth() {
     if (safety_state_ == SafetyState::ESTOP) return "ERROR";
     if (retry_rate_ > 0.05) return "DEGRADED";  // >5% retry rate
     return "OK";
   }
   ```

3. **Enhanced RoboClawStatus Message**
   - Add m1_current_avg, m2_current_avg (EMA values)
   - Add safety_state enum
   - Add health classification string
   - Mark WARN_OVERREGEN as severity: "benign"

**Acceptance Criteria**:
- Status message reflects actual I/O statistics
- Health classification correlates with observed communication issues
- Persistent OVERREGEN warnings marked as benign, not error-level

### Stage 6: Logging & Observability Cleanup
**Objective**: Unified structured logging; remove stderr artifacts.

**Tasks**:
1. **Structured Frame Logging**
   ```cpp
   void logFrame(const std::string& direction, uint8_t cmd, 
                const std::vector<uint8_t>& payload, bool success) {
     if (!frame_trace_enabled_) return;
     
     RCLCPP_DEBUG(logger_, "[FRAME] dir=%s cmd=0x%02X len=%zu success=%s", 
                  direction.c_str(), cmd, payload.size(), 
                  success ? "true" : "false");
   }
   ```

2. **Remove Legacy fprintf**
   - Eliminate all `fprintf(stderr, ...)` calls
   - Replace with appropriate RCLCPP_* macros
   - Provide debug/info/warn/error classification

3. **Throttled Diagnostic Logging**
   - Rate-limit repetitive warnings (e.g., encoder staleness)
   - Emergency messages (estop) always unthrottled

**Acceptance Criteria**:
- No stderr output during normal operation
- frame_trace=true provides detailed I/O debugging without overwhelming logs
- Repeated warnings throttled to 1/minute maximum

### Stage 7: Launch Configuration & Node Consolidation
**Objective**: Fix duplicate node names; provide complete configuration examples.

**Tasks**:
1. **Launch File Audit**
   - Identify why two nodes start with same name
   - Consolidate to single driver node
   - Separate test/capture tools as optional components

2. **Configuration Template**
   ```yaml
   # roboclaw_driver.yaml
   roboclaw_driver:
     ros__parameters:
       # Hardware
       device_name: "/dev/ttyACM0"
       baud_rate: 115200
       
       # Kinematics
       wheel_separation: 0.5
       pulses_per_meter: 3591.84  # Document derivation
       
       # Performance
       max_linear_vel: 1.0
       max_angular_vel: 2.0
       cmd_timeout: 0.5
       
       # Safety
       safety_enabled: true
       runaway_factor: 1.5
       overcurrent_limit: 10.0
       
       # Publishing
       publish_odom: true
       publish_joint_states: true
       sensor_update_rate: 50.0
       
       # Debugging
       frame_trace: false
   ```

3. **README Documentation**
   - Parameter reference table
   - pulses_per_meter derivation example
   - Architecture diagram showing IoExecutor flow
   - Troubleshooting guide

**Acceptance Criteria**:
- Single driver node launches successfully
- All parameters documented with units and defaults
- README provides sufficient setup guidance

### Stage 8: Testing Framework Foundation
**Objective**: Add minimal unit tests for critical algorithms without major refactoring.

**Tasks**:
1. **Protocol Unit Tests**
   ```cpp
   TEST(RoboClawProtocol, CrcCalculation) {
     std::vector<uint8_t> payload = {0x80, 0x2E, 0x01, 0x02};
     uint16_t expected_crc = 0x1234;  // Known good value
     EXPECT_EQ(calculateCrc(payload), expected_crc);
   }
   ```

2. **Safety Logic Tests**
   ```cpp
   TEST(SafetySystem, RunawayDetection) {
     SafetyManager mgr;
     mgr.setRunawayThreshold(1.5, 100);
     
     EXPECT_FALSE(mgr.checkRunaway(50, 75));   // Within threshold
     EXPECT_TRUE(mgr.checkRunaway(100, 200));  // Exceeds threshold
   }
   ```

3. **Odometry Integration Tests**
   ```cpp
   TEST(OdometryIntegrator, StraightLine) {
     OdometryIntegrator odom;
     odom.integrate(1000, 1000, 0.02);  // 1000 pulses each wheel, 20ms
     
     EXPECT_NEAR(odom.getX(), expected_x, 0.001);
     EXPECT_NEAR(odom.getYaw(), 0.0, 0.001);
   }
   ```

**Acceptance Criteria**:
- CRC function validated against known test vectors
- Safety thresholds mathematically verified
- Odometry integration accuracy within 1% over test distances

## Success Metrics

### Performance Targets
- **Latency**: cmd_vel → motor command <15ms (99th percentile)
- **Reliability**: <0.1% command retry rate under normal operation
- **Update Rate**: Odom/joint states at 50Hz ±2Hz
- **Safety Response**: Estop trigger within 100ms of fault detection

### Quality Indicators
- **No ACK timeouts** over 1-hour continuous operation
- **Encoder accuracy**: <2% error over 10m straight-line test
- **Memory stability**: No memory leaks over 24-hour run
- **Parameter validation**: All invalid configs rejected at startup

### Integration Readiness
- **Nav2 compatibility**: Smooth path following without oscillation
- **SLAM integration**: Consistent odometry for mapping
- **Diagnostic clarity**: Operators can identify issues from status messages
- **Configuration simplicity**: Setup from template in <10 minutes

---

*This plan builds incrementally on Stage 1 foundations while maintaining the existing dual-distance cmd 46 algorithm and incremental sensor acquisition strategy as requested.*