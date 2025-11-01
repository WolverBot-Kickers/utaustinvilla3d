#include "RX64Motor.h"
#include <stdio.h>

RX64Motor::RX64Motor(const char* port, int motor_id) 
    : portHandler(nullptr), 
      packetHandler(nullptr), 
      motor_id(motor_id),
      is_initialized(false)
{
    // Initialize handlers (like creating CANSparkMax object)
    portHandler = dynamixel::PortHandler::getPortHandler(port);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(1.0); // Protocol 1.0
}

RX64Motor::~RX64Motor() {
    // Cleanup - disable torque and close port
    if (is_initialized && portHandler) {
        enableTorque(false);
        portHandler->closePort();
    }
}

bool RX64Motor::init() {
    // Step 1: Open the port (like connecting CAN bus)
    if (!portHandler->openPort()) {
        printf("RX64Motor: Failed to open port\n");
        return false;
    }

    // Step 2: Set baudrate (communication speed)
    if (!portHandler->setBaudRate(DEFAULT_BAUDRATE)) {
        printf("RX64Motor: Failed to set baudrate\n");
        portHandler->closePort();
        return false;
    }

    // Step 3: Ping motor to verify connection (like checking CAN device online)
    printf("RX64Motor: Pinging motor ID %d...\n", motor_id);
    uint16_t model_number = 0;
    uint8_t error = 0;
    int comm_result = packetHandler->ping(portHandler, motor_id, &model_number, &error);
    
    if (comm_result != COMM_SUCCESS) {
        printf("RX64Motor: Ping failed! %s\n", 
               packetHandler->getTxRxResult(comm_result));
        printf("  - Check motor ID (currently: %d)\n", motor_id);
        printf("  - Check motor power and connections\n");
        portHandler->closePort();
        return false;
    } else if (error != 0) {
        printf("RX64Motor: Motor error: %s\n", 
               packetHandler->getRxPacketError(error));
        portHandler->closePort();
        return false;
    } else {
        printf("RX64Motor: Motor found! Model number: %d\n", model_number);
    }

    // Step 4: Enable torque (like setIdleMode(kBrake))
    enableTorque(true);

    is_initialized = true;
    return true;
}

void RX64Motor::enableTorque(bool enable) {
    if (!is_initialized) return;

    uint8_t error = 0;
    int comm_result = packetHandler->write1ByteTxRx(
        portHandler, 
        motor_id, 
        ADDR_TORQUE_ENABLE, 
        enable ? TORQUE_ENABLE : TORQUE_DISABLE, 
        &error
    );

    // Error checking (like checking CAN errors in WPILib)
    if (comm_result != COMM_SUCCESS) {
        printf("RX64Motor: Communication error enabling torque: %s\n", 
               packetHandler->getTxRxResult(comm_result));
    } else if (error != 0) {
        printf("RX64Motor: Motor error: %s\n", 
               packetHandler->getRxPacketError(error));
    }
}

void RX64Motor::setPosition(uint16_t position) {
    if (!is_initialized) return;

    uint8_t error = 0;
    int comm_result = packetHandler->write2ByteTxRx(
        portHandler,
        motor_id,
        ADDR_GOAL_POSITION,
        position,
        &error
    );

    // Like checking if set() succeeded in WPILib
    if (comm_result != COMM_SUCCESS) {
        printf("RX64Motor: Failed to set position\n");
    }
}

uint16_t RX64Motor::getPosition() {
    if (!is_initialized) return 0;

    uint16_t position = 0;
    uint8_t error = 0;
    int comm_result = packetHandler->read2ByteTxRx(
        portHandler,
        motor_id,
        ADDR_PRESENT_POSITION,
        &position,
        &error
    );

    if (comm_result != COMM_SUCCESS) {
        printf("RX64Motor: Failed to read position\n");
        return 0;
    }

    return position;
}

void RX64Motor::setSpeed(uint16_t speed) {
    if (!is_initialized) return;

    uint8_t error = 0;
    int comm_result = packetHandler->write2ByteTxRx(
        portHandler,
        motor_id,
        ADDR_MOVING_SPEED,
        speed,
        &error
    );

    if (comm_result != COMM_SUCCESS) {
        printf("RX64Motor: Failed to set speed\n");
    }
}

uint16_t RX64Motor::getSpeed() {
    if (!is_initialized) return 0;

    uint16_t speed = 0;
    uint8_t error = 0;
    int comm_result = packetHandler->read2ByteTxRx(
        portHandler,
        motor_id,
        ADDR_PRESENT_SPEED,
        &speed,
        &error
    );

    if (comm_result != COMM_SUCCESS) {
        printf("RX64Motor: Failed to read speed\n");
        return 0;
    }

    return speed;
}

bool RX64Motor::ping() {
    if (!is_initialized) return false;

    uint16_t model_number = 0;
    uint8_t error = 0;
    int comm_result = packetHandler->ping(
        portHandler,
        motor_id,
        &model_number,
        &error
    );

    if (comm_result == COMM_SUCCESS) {
        printf("RX64Motor: Ping successful! Model number: %d\n", model_number);
        return true;
    } else {
        printf("RX64Motor: Ping failed: %s\n", 
               packetHandler->getTxRxResult(comm_result));
        return false;
    }
}

