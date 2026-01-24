    # Raspberry Pi / Orange Pi Vision Diagnostic Guide

## Quick Diagnostic Checklist

Use this guide to verify if your Raspberry Pi/Orange Pi is being seen by the robot.

## Step 1: Check Physical Connection

- [ ] Raspberry Pi is powered on (check for power LED)
- [ ] Raspberry Pi is connected to robot network via Ethernet
- [ ] Robot radio is powered and working
- [ ] Driver Station can connect to robot

## Step 2: Verify IP Address

Your code expects the Raspberry Pi at: **10.80.46.11**

### Test Network Connection:

Open PowerShell or Command Prompt and run:

```powershell
ping 10.80.46.11
```

**Expected Results:**
- ✅ **Success**: You see replies like "Reply from 10.80.46.11: bytes=32 time=1ms TTL=64"
- ❌ **Failure**: You see "Request timed out" or "Destination host unreachable"

### If Ping Fails:

1. **Check Raspberry Pi IP Configuration**:
   - SSH into Raspberry Pi (if possible)
   - Run: `ip addr show` or `ifconfig`
   - Verify IP is set to 10.80.46.11

2. **Check Robot Network**:
   - Verify robot radio is at 10.80.46.1
   - Verify roboRIO is at 10.80.46.2
   - Raspberry Pi should be at 10.80.46.11

3. **Check Ethernet Cable**:
   - Try a different cable
   - Check for physical damage
   - Verify connection lights on both ends

## Step 3: Check PhotonVision Service

### Access PhotonVision Web Interface:

Open a web browser and navigate to:
```
http://10.80.46.11:5800
```

**Expected Results:**
- ✅ **Success**: PhotonVision dashboard loads
- ❌ **Failure**: "This site can't be reached" or timeout

### If Web Interface Doesn't Load:

1. **PhotonVision may not be running**:
   - SSH into Raspberry Pi
   - Check service status: `sudo systemctl status photonvision`
   - Start if needed: `sudo systemctl start photonvision`
   - Enable on boot: `sudo systemctl enable photonvision`

2. **Wrong port or IP**:
   - Try: `http://10.80.46.11:5800`
   - Try: `http://photonvision.local:5800`
   - Check PhotonVision documentation for correct port

## Step 4: Check NetworkTables Connection

### Using Driver Station:

1. Connect Driver Station to robot
2. Open SmartDashboard or Shuffleboard
3. Look for NetworkTables entries under "photonvision"

**Expected Results:**
- ✅ **Success**: You see `photonvision/CAM_BL/...` and `photonvision/CAM_BR/...` entries
- ❌ **Failure**: No photonvision entries

### Check Robot Code Output:

Deploy your code and check the Driver Station console for:
- ✅ **Good**: No "Could not find PhotonVision coprocessors" errors
- ⚠️ **Warning**: "PhotonVision coprocessor has not reported UUID" (this is OK, we disabled version check)
- ❌ **Bad**: "Could not find any PhotonVision coprocessors on NetworkTables"

## Step 5: Check Camera Connections

### In PhotonVision Web Interface:

1. Go to "Cameras" tab
2. Check if cameras are detected

**Expected Results:**
- ✅ **Success**: You see your cameras listed
- ❌ **Failure**: No cameras detected

### If Cameras Not Detected:

1. **Check USB connections**:
   - Cameras plugged into Raspberry Pi USB ports
   - Try different USB ports
   - Check for loose connections

2. **Check camera compatibility**:
   - Verify cameras are supported by PhotonVision
   - Check PhotonVision documentation for compatible cameras

3. **Restart Raspberry Pi**:
   - Sometimes cameras need a reboot to be recognized

## Step 6: Verify Camera Names

In PhotonVision, your cameras MUST be named:
- `CAM_BL` (Back Left)
- `CAM_BR` (Back Right)

**Case sensitive!** `CAM_BL` ≠ `cam_bl` ≠ `Cam_BL`

### To Rename Cameras:

1. In PhotonVision web interface
2. Click on each camera
3. Change name to exactly `CAM_BL` or `CAM_BR`
4. Save settings

## Step 7: Check SmartDashboard for Vision Data

After deploying code, check SmartDashboard for these values:

### Camera Connection Status:
- `BL Camera Connected` - Should be **true**
- `BR Camera Connected` - Should be **true**

### If False:
- PhotonVision isn't running
- Camera names don't match
- NetworkTables connection issue

## Common Issues and Solutions

### Issue 1: "Raspberry Pi not responding to ping"

**Possible Causes:**
- Raspberry Pi is off or not booted
- Wrong IP address
- Network cable issue
- Not on same network as robot

**Solutions:**
1. Check power to Raspberry Pi
2. Verify IP configuration on Raspberry Pi
3. Try different Ethernet cable
4. Check robot network configuration

### Issue 2: "PhotonVision web interface won't load"

**Possible Causes:**
- PhotonVision service not running
- Wrong port number
- Firewall blocking connection

**Solutions:**
1. SSH to Raspberry Pi and start PhotonVision service
2. Verify port 5800 is correct
3. Check firewall settings on Raspberry Pi

### Issue 3: "Cameras not detected in PhotonVision"

**Possible Causes:**
- USB cameras not connected
- Incompatible cameras
- USB power issue

**Solutions:**
1. Check USB connections
2. Verify camera compatibility
3. Use powered USB hub if needed
4. Restart Raspberry Pi

### Issue 4: "NetworkTables shows no photonvision entries"

**Possible Causes:**
- PhotonVision not connected to robot network
- NetworkTables not configured correctly
- Robot code not running

**Solutions:**
1. Verify Raspberry Pi can ping roboRIO (10.80.46.2)
2. Check PhotonVision NetworkTables settings
3. Restart PhotonVision service
4. Deploy robot code again

## Diagnostic Commands

### On Your Computer:

```powershell
# Test if Raspberry Pi is reachable
ping 10.80.46.11

# Test if roboRIO is reachable
ping 10.80.46.2

# Test if robot radio is reachable
ping 10.80.46.1
```

### On Raspberry Pi (via SSH):

```bash
# Check PhotonVision service status
sudo systemctl status photonvision

# Start PhotonVision
sudo systemctl start photonvision

# Enable PhotonVision on boot
sudo systemctl enable photonvision

# Check network configuration
ip addr show

# Check if cameras are detected
ls /dev/video*

# View PhotonVision logs
sudo journalctl -u photonvision -f
```

## What's in Your Code

Your Robot.java already has the correct configuration:

```java
NetworkTableInstance inst = NetworkTableInstance.getDefault();
inst.startClient3("10.80.46.11"); // IP of the Raspberry Pi
CameraServer.addServer("http://10.80.46.11:1181/stream.mjpg"); // BL Camera
CameraServer.addServer("http://10.80.46.11:1184/stream.mjpg"); // BR Camera
```

This tells the robot to:
1. Connect to NetworkTables at 10.80.46.11
2. Stream camera feeds from ports 1181 (BL) and 1184 (BR)

## Missing Vision Pieces?

Based on last season's code, you have **everything needed**:

✅ VisionSubsystem.java - Complete
✅ VisionSim.java - Complete
✅ Constants.Vision - Complete
✅ Robot.java vision initialization - Complete
✅ RobotContainer vision integration - Complete
✅ SmartDashboard outputs - Complete
✅ Field2d visualization - Complete
✅ Pose estimation - Complete

**Nothing is missing from the code!** The issue is likely:
1. Raspberry Pi network configuration
2. PhotonVision service not running
3. Camera names not matching
4. Physical connection issue

## Next Steps

1. **Start with ping test**: Can you ping 10.80.46.11?
   - ✅ Yes → Go to Step 3 (Check PhotonVision)
   - ❌ No → Fix network/IP configuration

2. **Check PhotonVision web interface**: Can you access http://10.80.46.11:5800?
   - ✅ Yes → Go to Step 5 (Check cameras)
   - ❌ No → Start PhotonVision service

3. **Verify cameras**: Are cameras detected in PhotonVision?
   - ✅ Yes → Rename to CAM_BL and CAM_BR
   - ❌ No → Check USB connections

4. **Deploy code and test**: Check SmartDashboard for camera connection status

## Getting Help

If you're still having issues, provide this information:
1. Result of `ping 10.80.46.11`
2. Can you access PhotonVision web interface?
3. What do you see in SmartDashboard for "BL Camera Connected"?
4. Any error messages in Driver Station console?
5. PhotonVision service status (if you can SSH)

This will help diagnose exactly where the connection is failing.
