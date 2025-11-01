#include "PortFinder.h"
#include <dirent.h>
#include <sys/stat.h>
#include <string.h>
#include <stdio.h>

#ifdef __APPLE__
#include <IOKit/IOKitLib.h>
#include <IOKit/serial/IOSerialKeys.h>
#endif

int PortFinder::scanPorts(std::vector<std::string>& ports) {
    ports.clear();
    
#ifdef __APPLE__
    // Mac: Look for /dev/tty.usbserial-* and /dev/tty.usbmodem*
    const char* patterns[] = {
        "/dev/tty.usbserial-",
        "/dev/tty.usbmodem",
        "/dev/tty.SLAB_USBtoUART",
        "/dev/ttyUSB",
        NULL
    };
    
    for (int i = 0; patterns[i] != NULL; i++) {
        // Try common USB port numbers (0-9)
        for (int j = 0; j < 10; j++) {
            char portname[256];
            if (strstr(patterns[i], "usbserial") || strstr(patterns[i], "usbmodem")) {
                snprintf(portname, sizeof(portname), "%s%04d", patterns[i], j);
            } else {
                snprintf(portname, sizeof(portname), "%s%d", patterns[i], j);
            }
            
            struct stat st;
            if (stat(portname, &st) == 0 && S_ISCHR(st.st_mode)) {
                ports.push_back(std::string(portname));
            }
        }
    }
    
#elif __linux__
    // Linux: Look for /dev/ttyUSB* and /dev/ttyACM*
    DIR *dir = opendir("/dev");
    if (dir != NULL) {
        struct dirent *entry;
        while ((entry = readdir(dir)) != NULL) {
            if (strncmp(entry->d_name, "ttyUSB", 6) == 0 ||
                strncmp(entry->d_name, "ttyACM", 6) == 0) {
                std::string port = std::string("/dev/") + entry->d_name;
                ports.push_back(port);
            }
        }
        closedir(dir);
    }
    
#elif _WIN32
    // Windows: COM1-COM256
    for (int i = 1; i <= 256; i++) {
        char portname[16];
        snprintf(portname, sizeof(portname), "COM%d", i);
        // On Windows, we'd need to check if port exists differently
        // For now, just add common ports
        if (i <= 20) {
            ports.push_back(std::string(portname));
        }
    }
#endif

    return ports.size();
}

bool PortFinder::tryPort(const char* port, int motor_id, int baudrate) {
    // Try to open port
    dynamixel::PortHandler *portHandler = 
        dynamixel::PortHandler::getPortHandler(port);
    
    if (!portHandler->openPort()) {
        return false;
    }
    
    // Try to set baudrate
    if (!portHandler->setBaudRate(baudrate)) {
        portHandler->closePort();
        return false;
    }
    
    // Try to ping motor
    dynamixel::PacketHandler *packetHandler = 
        dynamixel::PacketHandler::getPacketHandler(1.0);
    
    uint16_t model_number = 0;
    uint8_t error = 0;
    int comm_result = packetHandler->ping(
        portHandler,
        motor_id,
        &model_number,
        &error
    );
    
    portHandler->closePort();
    
    return (comm_result == COMM_SUCCESS && error == 0);
}

bool PortFinder::findMotor(std::string& found_port, int motor_id, int baudrate) {
    printf("PortFinder: Scanning for RX64 motor (ID=%d, Baudrate=%d)...\n", 
           motor_id, baudrate);
    
    std::vector<std::string> ports;
    int num_ports = scanPorts(ports);
    
    if (num_ports == 0) {
        printf("PortFinder: No USB ports found!\n");
        return false;
    }
    
    printf("PortFinder: Found %d potential ports, testing each...\n", num_ports);
    
    // Try each port
    for (size_t i = 0; i < ports.size(); i++) {
        printf("PortFinder: Trying %s... ", ports[i].c_str());
        fflush(stdout);
        
        if (tryPort(ports[i].c_str(), motor_id, baudrate)) {
            printf("SUCCESS! Motor found!\n");
            found_port = ports[i];
            return true;
        } else {
            printf("no response\n");
        }
    }
    
    printf("PortFinder: Motor not found on any port.\n");
    printf("  - Check motor power\n");
    printf("  - Check motor ID (currently searching for ID=%d)\n", motor_id);
    printf("  - Check baudrate (currently trying %d)\n", baudrate);
    
    return false;
}

