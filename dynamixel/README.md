# RX64 Motor Control

WPILib-style class for controlling RX64 Dynamixel motors using Protocol 1.0.

## ✅ Pre-Flight Checklist (Before First Run)

### 1. **Hardware Setup**
- [ ] RX64 motor is powered (12V supply connected)
- [ ] USB-to-serial adapter (U2D2 or compatible) connected
- [ ] Motor ID is known (default is 1)
- [ ] Motor baudrate matches code (default is 57600)

### 2. **Auto-Detection Feature** ✨

**NEW:** The code now automatically finds your USB port! No need to manually specify.

Just run the program and it will:
1. Scan all USB ports
2. Try to ping motor on each port
3. Use the port where motor responds

**Manual port specification (if needed):**
```cpp
RX64Motor motor("/dev/ttyUSB0", 1);  // Specify port manually
```

**Auto-detection:**
```cpp
std::string port;
PortFinder::findMotor(1, 57600, port);  // Finds port automatically
RX64Motor motor(port.c_str(), 1);
```

### 3. **Verify Motor Connection**

If you have Dynamixel Wizard 2.0 installed:
1. Open Dynamixel Wizard
2. Scan for motors
3. Verify motor ID and baudrate
4. Test basic movement

## 🚀 Will It Work First Try?

**Most likely YES, IF:**

1. ✅ **Control table addresses are correct** - Verified against official SDK examples
2. ✅ **Protocol 1.0** - RX64 uses Protocol 1.0 (correct)
3. ✅ **Baudrate 57600** - Standard default for RX64
4. ✅ **Ping verification** - Code now pings motor on init to verify connection
5. ✅ **Error handling** - All operations check for errors

**Common Issues That Might Prevent First-Try Success:**

| Issue | Symptom | Fix |
|-------|---------|-----|
| Wrong port name | `Failed to open port` | Check USB port with `ls /dev/tty*` |
| Wrong motor ID | `Ping failed` | Check ID in Dynamixel Wizard, change in code |
| Wrong baudrate | `Ping failed` or no response | Check/change baudrate (common: 57600, 1000000) |
| Motor not powered | No response | Check 12V power supply |
| USB permissions | Permission denied | `sudo chmod 666 /dev/ttyUSB0` or add user to dialout group |

## 📝 Code Structure (WPILib Comparison)

```cpp
// Create motor (like CANSparkMax)
RX64Motor motor("/dev/ttyUSB0", 1);

// Initialize (like robotInit)
if (motor.init()) {  // This now pings motor automatically!
    motor.setPosition(512);    // Like setReference()
    uint16_t pos = motor.getPosition();  // Like getEncoder()
}
```

## 🔧 Building

See main project CMakeLists.txt or create a simple Makefile.

## 📚 Key Differences from MX Series

RX64 uses **same Protocol 1.0 addresses** as MX series:
- Torque Enable: Address 24 ✅
- Goal Position: Address 30 ✅  
- Present Position: Address 36 ✅
- Moving Speed: Address 32 ✅

These addresses are **verified correct** from official Dynamixel SDK examples.

## ⚠️ Important Notes

1. **Position Range**: 0-1023 (where 512 is center)
2. **Speed**: 0-1023 (where 0 is max speed)
3. **Always disable torque** when done to allow free movement
4. **Check motor ID** - if motor doesn't respond, ID might be different than 1

