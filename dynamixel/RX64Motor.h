#ifndef RX64MOTOR_H
#define RX64MOTOR_H

#include "dynamixel_sdk/dynamixel_sdk.h"

// Control table addresses for RX64 (Protocol 1.0)
#define ADDR_TORQUE_ENABLE       24
#define ADDR_GOAL_POSITION       30
#define ADDR_PRESENT_POSITION    36
#define ADDR_MOVING_SPEED        32
#define ADDR_PRESENT_SPEED       39

// Default values
#define DEFAULT_BAUDRATE          57600
#define TORQUE_ENABLE            1
#define TORQUE_DISABLE           0

/**
 * RX64Motor - WPILib-style class for controlling RX64 Dynamixel motors
 * 
 * Similar to CANSparkMax or TalonSRX in FIRST Robotics
 */
class RX64Motor {
private:
    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;
    int motor_id;
    bool is_initialized;

public:
    /**
     * Constructor - like creating a CANSparkMax
     * @param port Serial port name (e.g., "/dev/ttyUSB0" or "/dev/tty.usbserial-*")
     * @param motor_id Motor ID (default is 1)
     */
    RX64Motor(const char* port, int motor_id);

    /**
     * Destructor - cleanup resources
     */
    ~RX64Motor();

    /**
     * Initialize the motor connection
     * Similar to motor.restoreFactoryDefaults() + basic setup
     * @return true if successful, false otherwise
     */
    bool init();

    /**
     * Enable or disable torque
     * Similar to setIdleMode(kBrake) vs setIdleMode(kCoast)
     * @param enable true to enable (motor holds position), false to disable (free to move)
     */
    void enableTorque(bool enable);

    /**
     * Set target position
     * Similar to pidController.setReference(position, ControlType.kPosition)
     * @param position Target position (0-1023, where 512 is center)
     */
    void setPosition(uint16_t position);

    /**
     * Get current position
     * Similar to encoder.getPosition()
     * @return Current position (0-1023)
     */
    uint16_t getPosition();

    /**
     * Set movement speed
     * Similar to setting max velocity limit
     * @param speed Movement speed (0-1023, where 0 is max speed)
     */
    void setSpeed(uint16_t speed);

    /**
     * Get current movement speed
     * @return Current speed (0-1023)
     */
    uint16_t getSpeed();

    /**
     * Check if motor is initialized and connected
     * @return true if ready to use
     */
    bool isInitialized() const { return is_initialized; }

    /**
     * Ping the motor to verify connection
     * Similar to checking if CAN device is online
     * @return true if motor responds
     */
    bool ping();
};

#endif /* RX64MOTOR_H */

