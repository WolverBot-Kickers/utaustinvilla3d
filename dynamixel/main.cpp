/**
 * Example usage of RX64Motor
 * Similar to Robot.java in WPILib
 */

#include "RX64Motor.h"
#include "PortFinder.h"
#include <unistd.h>  // for sleep()
#include <stdio.h>
#include <string>

int main() {
    printf("=== RX64 Motor Control Example ===\n\n");

    std::string port_name;
    int motor_id = 1;

    // AUTO-DETECT: Try to find motor automatically (like CAN device scanning)
    printf("Attempting auto-detection...\n");
    if (PortFinder::findMotor(motor_id, 57600, port_name)) {
        printf("Auto-detection successful! Found motor on: %s\n\n", port_name.c_str());
    } else {
        // Fallback: Try common port names
        printf("Auto-detection failed. Trying common ports...\n");
        const char* common_ports[] = {
            "/dev/ttyUSB0",
            "/dev/ttyUSB1",
            "/dev/tty.usbserial-0001",
            "/dev/tty.usbmodem14101",
            NULL
        };
        
        bool found = false;
        for (int i = 0; common_ports[i] != NULL; i++) {
            if (PortFinder::tryPort(common_ports[i], motor_id, 57600)) {
                port_name = common_ports[i];
                printf("Found motor on: %s\n\n", port_name.c_str());
                found = true;
                break;
            }
        }
        
        if (!found) {
            printf("ERROR: Could not find motor!\n");
            printf("  Please specify port manually or check:\n");
            printf("  - Motor power\n");
            printf("  - USB connection\n");
            printf("  - Motor ID (currently: %d)\n", motor_id);
            return -1;
        }
    }

    // Create motor object (like creating CANSparkMax in Robot.java)
    RX64Motor motor(port_name.c_str(), motor_id);

    // Initialize (like in robotInit() or autonomousInit())
    if (!motor.init()) {
        printf("ERROR: Failed to initialize motor!\n");
        return -1;
    }

    printf("Motor initialized successfully!\n\n");

    // Set movement speed (like setting max velocity)
    motor.setSpeed(300);  // Medium speed (0-1023)
    printf("Speed set to 300\n\n");

    // Move motor through test positions (like autonomous commands)
    uint16_t positions[] = {512, 200, 823, 512};  // Center, left, right, center
    int num_positions = sizeof(positions) / sizeof(positions[0]);

    for (int i = 0; i < num_positions; i++) {
        printf("Moving to position %d...\n", positions[i]);
        
        // Set goal position (like pidController.setReference())
        motor.setPosition(positions[i]);
        
        // Wait for movement (like Command.wait() or delay)
        sleep(2);
        
        // Read back position (like encoder.getPosition())
        uint16_t current = motor.getPosition();
        printf("  Current position: %d\n\n", current);
    }

    // Disable torque when done (like end of match)
    motor.enableTorque(false);
    printf("Motor disabled. Done!\n");

    return 0;
}

