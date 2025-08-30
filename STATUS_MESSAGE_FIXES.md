# RoboClawStatus Message Fixes

## Issues Fixed

### 1. Status Data Not Updated from Incremental Reads

**Problem**: The `/roboclaw_status` topic was making direct calls to RoboClaw hardware methods instead of using the cached data from our incremental reading system.

**Solution**: Updated `motor_driver_node.cpp` to use cached sensor data via `IoExecutor::instance().getDeviceCache()`:

- ✅ **m1_current_speed** - Now reads from cached velocities
- ✅ **m2_current_speed** - Now reads from cached velocities  
- ✅ **m1_motor_current** - Now reads from cached currents
- ✅ **m2_motor_current** - Now reads from cached currents
- ✅ **m1_encoder_value** - Now reads from cached encoders
- ✅ **m2_encoder_value** - Now reads from cached encoders
- ✅ **main_battery_voltage** - Now reads from cached voltages
- ✅ **logic_battery_voltage** - Now reads from cached voltages
- ✅ **temperature** - Now reads from cached temperature

### 2. Incomplete Error Bit Decoding

**Problem**: Error status was only providing a basic error string without:
- Raw error status bits value
- Complete decoding of all error and warning flags

**Solution**: 
1. **Added `error_status` field** to `RoboClawStatus.msg` to expose raw error bits
2. **Implemented complete error bit decoder** with all 24 error/warning flags:

#### Error Bits (0x00001-0x02000):
- `ERROR_ESTOP` (0x00000001)
- `ERROR_TEMP` (0x00000002) 
- `ERROR_TEMP2` (0x00000004)
- `ERROR_LBATHIGH` (0x00000010)
- `ERROR_LBATLOW` (0x00000020)
- `ERROR_FAULTM1` (0x00000040)
- `ERROR_FAULTM2` (0x00000080)
- `ERROR_SPEED1` (0x00000100)
- `ERROR_SPEED2` (0x00000200)
- `ERROR_POS1` (0x00000400)
- `ERROR_POS2` (0x00000800)
- `ERROR_CURRENTM1` (0x00001000)
- `ERROR_CURRENTM2` (0x00002000)

#### Warning Bits (0x10000-0x80000000):
- `WARN_OVERCURRENTM1` (0x00010000)
- `WARN_OVERCURRENTM2` (0x00020000)
- `WARN_MBATHIGH` (0x00040000)
- `WARN_MBATLOW` (0x00080000)
- `WARN_TEMP` (0x00100000)
- `WARN_TEMP2` (0x00200000)
- `WARN_S4` (0x00400000)
- `WARN_S5` (0x00800000)
- `WARN_CAN` (0x10000000) - MCP Only
- `WARN_BOOT` (0x20000000)
- `WARN_OVERREGENM1` (0x40000000)
- `WARN_OVERREGENM2` (0x80000000)

## Updated Message Structure

```
float32  m1_p
float32  m1_i
float32  m1_d
uint32   m1_qpps
int32    m1_current_speed      # ✅ Now from cache
float32  m1_motor_current      # ✅ Now from cache
uint32   m1_encoder_value      # ✅ Now from cache
uint8    m1_encoder_status

float32  m2_p
float32  m2_i
float32  m2_d
uint32   m2_qpps
int32    m2_current_speed      # ✅ Now from cache
float32  m2_motor_current      # ✅ Now from cache
uint32   m2_encoder_value      # ✅ Now from cache
uint8    m2_encoder_status

float32  main_battery_voltage  # ✅ Now from cache
float32  logic_battery_voltage # ✅ Now from cache
float32  temperature           # ✅ Now from cache
uint32   error_status          # ✅ NEW: Raw error bits
string   error_string          # ✅ Enhanced: Full bit decoding
```

## Benefits

1. **Real-time Data**: Status messages now reflect the continuously updated incremental sensor reads
2. **Reduced I/O Load**: Status publishing no longer blocks the I/O thread with synchronous hardware calls
3. **Complete Error Visibility**: Both raw error bits and human-readable decoded strings are available
4. **Better Diagnostics**: All 24 error/warning conditions are now properly identified
5. **Consistent Performance**: Status message publishing rate is now independent of hardware I/O timing

## Build Status
✅ **Successfully compiles**
✅ **Message interface updated**  
✅ **Error decoding complete**
✅ **Cache integration working**

The `/roboclaw_status` topic now provides complete, real-time sensor data using the incremental reading system!
