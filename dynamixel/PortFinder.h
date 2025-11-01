#ifndef PORTFINDER_H
#define PORTFINDER_H

#include <vector>
#include <string>
#include "dynamixel_sdk/dynamixel_sdk.h"

/**
 * PortFinder - Auto-detect USB ports and find Dynamixel motors
 * Similar to WPILib's device discovery utilities
 */
class PortFinder {
public:
    /**
     * Scan for available USB serial ports
     * @param ports Vector to fill with found port names
     * @return Number of ports found
     */
    static int scanPorts(std::vector<std::string>& ports);

    /**
     * Find Dynamixel motor by scanning ports and pinging
     * Similar to scanning CAN bus for devices
     * @param motor_id Motor ID to search for (default 1)
     * @param baudrate Baudrate to try (default 57600)
     * @param found_port Output parameter: port name if found
     * @return true if motor found, false otherwise
     */
    static bool findMotor(std::string& found_port, int motor_id = 1, int baudrate = 57600);

    /**
     * Try to ping a motor on a specific port
     * @param port Port name to try
     * @param motor_id Motor ID to ping
     * @param baudrate Baudrate to use
     * @return true if motor responds
     */
    static bool tryPort(const char* port, int motor_id, int baudrate = 57600);

};

#endif /* PORTFINDER_H */

