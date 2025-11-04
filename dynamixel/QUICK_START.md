# Quick Start Guide - Running RX64 Motor Control

## 📋 Prerequisites Checklist

Before running, make sure:
- [ ] RX64 motor is powered (12V power supply connected)
- [ ] USB-to-serial adapter (U2D2 or compatible) is connected to your computer
- [ ] Motor is connected to the USB adapter via daisy chain (if using multiple motors)
- [ ] Motor ID is known (default is 1 - check with Dynamixel Wizard if unsure)

## 🚀 Running the Program

### Step 1: Navigate to the build directory
```bash
cd dynamixel/build
```

### Step 2: Run the executable
```bash
./rx64_test
```

That's it! The program will automatically:
1. Scan for USB ports
2. Find your motor
3. Connect and verify
4. Move through test positions

## 📊 What You Should See

### ✅ Success Output:
```
=== RX64 Motor Control Example ===

Attempting auto-detection...
PortFinder: Scanning for RX64 motor (ID=1, Baudrate=57600)...
PortFinder: Found 1 potential ports, testing each...
PortFinder: Trying /dev/tty.usbserial-0001... SUCCESS! Motor found!

Auto-detection successful! Found motor on: /dev/tty.usbserial-0001

RX64Motor: Pinging motor ID 1...
RX64Motor: Motor found! Model number: 106
RX64Motor: Pinging motor ID 1...
RX64Motor: Motor found! Model number: 106

Motor initialized successfully!

Speed set to 300

Moving to position 512...
  ✓ Goal: 512, Actual: 512

Moving to position 200...
  ✓ Goal: 200, Actual: 200

Moving to position 823...
  ✓ Goal: 823, Actual: 823

Moving to position 512...
  ✓ Goal: 512, Actual: 512

Motor disabled. Done!
```

### ❌ Common Issues:

**Issue 1: "No USB ports found"**
```
PortFinder: No USB ports found!
```
**Fix:** 
- Check USB cable connection
- Try a different USB port
- On Mac, check System Settings → Privacy & Security → Allow USB accessories

**Issue 2: "Ping failed"**
```
PortFinder: Trying /dev/ttyUSB0... no response
RX64Motor: Ping failed!
```
**Fixes:**
- **Wrong Motor ID:** Motor might be ID 2, 3, etc. Edit `main.cpp` line 16:
  ```cpp
  int motor_id = 2;  // Try different IDs
  ```
- **Wrong Baudrate:** Motor might be configured for 1000000 instead of 57600. Edit `main.cpp` line 20:
  ```cpp
  if (PortFinder::findMotor(port_name, motor_id, 1000000)) {  // Try 1000000
  ```
- **Motor not powered:** Check 12V power supply connection
- **USB permissions:** On Linux, you might need:
  ```bash
  sudo chmod 666 /dev/ttyUSB0
  ```

**Issue 3: "Library not loaded"**
```
dyld: Library not loaded: @rpath/libdxl_mac_cpp.dylib
```
**Fix:** Rebuild the project:
```bash
cd dynamixel/build
make
```

## 🔧 Changing Motor ID or Baudrate

If your motor has a different ID or baudrate, edit `dynamixel/main.cpp`:

```cpp
// Line 16: Change motor ID
int motor_id = 1;  // Change this if your motor ID is different

// Line 20: Change baudrate
if (PortFinder::findMotor(port_name, motor_id, 57600)) {  // Try 57600, 1000000, etc.
```

Then rebuild:
```bash
cd dynamixel/build
make
./rx64_test
```

## 🎯 Testing Different Positions

The program moves through preset positions. To customize, edit `dynamixel/main.cpp` line 35:

```cpp
uint16_t positions[] = {512, 200, 823, 512};  // Customize these values
```

- `512` = Center position
- `200` = Counter-clockwise
- `823` = Clockwise
- Range: 0-1023 (where 0 is fully CCW, 1023 is fully CW)

## 🔄 Rebuilding After Changes

After editing code:
```bash
cd dynamixel/build
make
./rx64_test
```

Or clean build:
```bash
cd dynamixel/build
make clean
cmake ..
make
./rx64_test
```

## 📝 Using in Your Own Code

```cpp
#include "RX64Motor.h"
#include <unistd.h>

int main() {
    // Auto-detect or specify port
    RX64Motor motor("/dev/ttyUSB0", 1);  // Port, Motor ID
    
    if (motor.init()) {
        motor.setSpeed(300);           // Set speed
        motor.setPosition(512);        // Move to center
        sleep(2);                      // Wait
        uint16_t pos = motor.getPosition();  // Read position
        motor.enableTorque(false);     // Disable when done
    }
    
    return 0;
}
```

## 🆘 Still Having Issues?

1. **Verify hardware:**
   - Use Dynamixel Wizard 2.0 to test motor connection first
   - Confirm motor ID and baudrate

2. **Check USB port:**
   ```bash
   # Mac
   ls /dev/tty.usbserial-* /dev/tty.usbmodem*
   
   # Linux
   ls /dev/ttyUSB* /dev/ttyACM*
   ```

3. **Try manual port:**
   Edit `main.cpp` to skip auto-detection:
   ```cpp
   // Comment out auto-detection, use manual:
   std::string port_name = "/dev/ttyUSB0";  // Your actual port
   RX64Motor motor(port_name.c_str(), 1);
   ```

