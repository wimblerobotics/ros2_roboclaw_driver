// MIT License
// Simplified direct RoboClaw device interface (replaces transport/protocol/hardware layers)
#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace roboclaw_driver {

  struct EncoderSnapshot {
    uint32_t value = 0;
    uint8_t status = 0;
    int32_t speed_qpps = 0;
  };

  struct VoltageSnapshot {
    float main = 0.0f;
    float logic = 0.0f;
  };

  struct CurrentSnapshot {
    float m1_inst = 0.0f;
    float m2_inst = 0.0f;
  };

  struct TemperatureSnapshot {
    float t1 = 0.0f;
    float t2 = 0.0f;
  };

  struct Snapshot {
    EncoderSnapshot m1_enc;
    EncoderSnapshot m2_enc;
    VoltageSnapshot volts;
    CurrentSnapshot currents;
    TemperatureSnapshot temps;
    uint32_t status_bits = 0;
  };

  /**
   * @brief RoboClaw motor controller device interface
   *
   * Provides direct communication with RoboClaw motor controllers over serial.
   * Replaces the previous multi-layer abstraction (transport/protocol/hardware).
   */
  class RoboClawDevice {
  public:
    /**
     * @brief Construct and initialize RoboClaw device
     * @param device Serial device path (e.g., "/dev/ttyACM0")
     * @param baud_rate Communication baud rate
     * @param address RoboClaw device address (0-127)
     * @throws std::runtime_error if device cannot be opened or initialized
     */
    RoboClawDevice(const std::string& device, int baud_rate, uint8_t address);
    ~RoboClawDevice();

    /**
     * @brief Check if device is properly initialized and communicating
     * @return true if device responded to firmware version request
     */
    bool isInitialized() const { return initialized_; }

    /**
     * @brief Read comprehensive device status snapshot
     * @param out Output snapshot structure to populate
     * @param err Error message if operation fails
     * @return true on success, false on error
     */
    bool readSnapshot(Snapshot& out, std::string& err);

    /**
     * @brief Command motor speeds in quadrature pulses per second
     * @param m1_qpps Motor 1 speed in QPPS (positive = forward)
     * @param m2_qpps Motor 2 speed in QPPS (positive = forward)
     * @param err Error message if operation fails
     * @return true on success, false on error
     */
    bool driveSpeeds(int32_t m1_qpps, int32_t m2_qpps, std::string& err);

    /**
     * @brief Configure PID parameters for specified motor
     * @param motor Motor number (1 or 2)
     * @param p Proportional gain
     * @param i Integral gain
     * @param d Derivative gain
     * @param qpps Maximum quadrature pulses per second
     * @param err Error message if operation fails
     * @return true on success, false on error
     */
    bool setPID(int motor, float p, float i, float d, uint32_t qpps, std::string& err);

    /**
     * @brief Reset encoder counts to zero for both motors
     * @param err Error message if operation fails
     * @return true on success, false on error
     */
    bool resetEncoders(std::string& err);

    /**
     * @brief Read firmware version string from device
     * @return Firmware version string, empty if communication failed
     */
    std::string version();

    /** @brief Get last commanded speed for motor 1 */
    int32_t getLastCommand1() const { return last_cmd_m1_; }

    /** @brief Get last commanded speed for motor 2 */
    int32_t getLastCommand2() const { return last_cmd_m2_; }

  private:
    std::string device_;
    int baud_;
    uint8_t addr_;
    int fd_ = -1;
    bool initialized_ = false;  ///< True if device communication verified
    int32_t last_cmd_m1_ = 0;
    int32_t last_cmd_m2_ = 0;

    bool openPort(std::string& err);
    void updateCrc(uint16_t& crc, uint8_t byte);
    bool writeBytes(const uint8_t* data, size_t len, std::string& err);
    uint8_t readByte(double timeout_sec, std::string& err);
    bool readBytes(uint8_t* dst, size_t len, double timeout_sec, std::string& err);
    bool commandTxNeedsCRC(uint8_t cmd) const;
    bool commandRxHasCRC(uint8_t cmd) const;
    bool sendSimple(uint8_t command, const uint8_t* payload, size_t len, std::string& err);
    bool sendCrc(uint8_t command, const uint8_t* payload, size_t len, std::string& err);
    bool readU16(uint8_t cmd, uint16_t& val, std::string& err);
    bool readCurrents(uint8_t cmd, uint16_t& m1, uint16_t& m2, std::string& err);
    bool readU32(uint8_t cmd, uint32_t& val, std::string& err);
    bool readU32WithStatus(uint8_t cmd, uint32_t& value, uint8_t& status, std::string& err);
    bool readVelocity(uint8_t cmd, int32_t& vel, std::string& err);
  };

} // namespace roboclaw_driver
