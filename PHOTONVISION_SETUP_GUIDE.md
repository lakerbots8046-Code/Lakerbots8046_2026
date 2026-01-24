# PhotonVision Setup and Troubleshooting Guide

## Error: "Could not find any PhotonVision coprocessors on NetworkTables"

This error is **normal** and expected in these situations:
1. ✅ You're running the code in simulation (no physical cameras)
2. ✅ The Orange Pi/coprocessor isn't powered on
3. ✅ PhotonVision isn't running on the coprocessor
4. ✅ The cameras aren't configured in PhotonVision yet
5. ✅ Network connection issues between robot and coprocessor

**This error will NOT prevent your code from compiling or deploying!** It just means the vision system can't find cameras right now.

---

## PhotonVision Setup Checklist

### Step 1: Verify Hardware Connections
- [ ] Orange Pi (or coprocessor) is powered on
- [ ] Orange Pi is connected to robot network (via Ethernet or WiFi)
- [ ] Both cameras are physically connected to Orange Pi USB ports
- [ ] Orange Pi has PhotonVision installed

### Step 2: Access PhotonVision Web Interface
1. Connect to robot network (WiFi or USB)
2. Open web browser
3. Navigate to: `http://10.80.46.11:5800` (or your Orange Pi's IP)
4. You should see the PhotonVision dashboard

**If you can't access PhotonVision:**
- Verify Orange Pi IP address (should be 10.80.46.11 based on your code)
- Check if PhotonVision service is running on Orange Pi
- Verify network connectivity

### Step 3: Configure Cameras in PhotonVision

#### For CAM_BL (Back Left Camera):
1. In PhotonVision web interface, go to "Cameras" tab
2. Find your back-left camera in the list
3. Click on it to configure
4. **Important**: Set the camera name to exactly `CAM_BL`
5. Configure camera settings:
   - Resolution: 640x480 or 960x720 (balance between speed and accuracy)
   - FPS: 30-60 (higher = more updates but more CPU)
   - Exposure: Auto or manual (adjust based on lighting)
6. Go to "AprilTag" pipeline
7. Enable AprilTag detection
8. Save settings

#### For CAM_BR (Back Right Camera):
1. Repeat the same process for your back-right camera
2. **Important**: Set the camera name to exactly `CAM_BR`
3. Configure same settings as CAM_BL

### Step 4: Verify Camera Names Match Code

**Critical**: The camera names in PhotonVision **must exactly match** the names in your code:
- Code expects: `CAM_BL` and `CAM_BR`
- PhotonVision must have cameras named: `CAM_BL` and `CAM_BR`

**Case sensitive!** `CAM_BL` ≠ `cam_bl` ≠ `Cam_BL`

### Step 5: Test Camera Streams

In PhotonVision web interface:
1. Select each camera
2. Verify you can see the camera feed
3. Point camera at an AprilTag
4. Verify the tag is detected (should see green box around it)
5. Check that tag ID is displayed

### Step 6: Verify NetworkTables Connection

1. Open SmartDashboard or Shuffleboard
2. Look for "photonvision" table in NetworkTables
3. You should see entries for:
   - `photonvision/CAM_BL/...`
   - `photonvision/CAM_BR/...`
4. If you see these, PhotonVision is connected!

---

## Common Issues and Solutions

### Issue 1: "PhotonVision coprocessor has not reported a message interface UUID"

**Cause**: Version mismatch or camera still starting up

**Solution**: This has been fixed in the code by disabling version checking:
```java
cameraBL.setVersionCheckEnabled(false);
cameraBR.setVersionCheckEnabled(false);
```

This warning is harmless and can be ignored. The cameras will work correctly once they're fully started.

**Alternative solutions if you still see issues**:
1. Update PhotonVision on your coprocessor to match v2026.1.1-rc-2
2. Restart PhotonVision service: `sudo systemctl restart photonvision`
3. Wait a few seconds after robot startup for cameras to initialize

### Issue 2: "Could not find any PhotonVision coprocessors"

**Cause**: Robot code can't find PhotonVision on network

**Solutions**:
1. **Check if PhotonVision is running**:
   - Try accessing `http://10.80.46.11:5800` in browser
   - If you can't access it, PhotonVision isn't running

2. **Verify IP address**:
   - Your code expects Orange Pi at `10.80.46.11`
   - Check actual IP of your Orange Pi
   - Update IP in `Robot.java` if different:
   ```java
   inst.startClient3("YOUR_ACTUAL_IP");
   CameraServer.addServer("http://YOUR_ACTUAL_IP:1181/stream.mjpg");
   CameraServer.addServer("http://YOUR_ACTUAL_IP:1184/stream.mjpg");
   ```

3. **Check network connection**:
   - Ensure Orange Pi is on same network as robot
   - Try pinging Orange Pi: `ping 10.80.46.11`

4. **Restart PhotonVision service**:
   - SSH into Orange Pi
   - Restart PhotonVision: `sudo systemctl restart photonvision`

### Issue 2: Camera names don't match

**Cause**: PhotonVision camera names don't match code expectations

**Solution**:
1. In PhotonVision, rename cameras to exactly `CAM_BL` and `CAM_BR`
2. Or update code in `Constants.java`:
   ```java
   public static final String kCameraNameBL = "YOUR_ACTUAL_CAMERA_NAME";
   public static final String kCameraNameBR = "YOUR_ACTUAL_CAMERA_NAME";
   ```

### Issue 3: Cameras not detected in PhotonVision

**Cause**: Cameras not connected or not recognized

**Solutions**:
1. Check USB connections
2. Try different USB ports
3. Restart Orange Pi
4. Check camera compatibility with PhotonVision

### Issue 4: No AprilTags detected

**Cause**: Camera settings, lighting, or tag visibility issues

**Solutions**:
1. Adjust camera exposure in PhotonVision
2. Ensure adequate lighting
3. Verify AprilTags are in camera field of view
4. Check tag size settings match your actual tags
5. Ensure tags are not damaged or obscured

### Issue 5: Poor pose estimation accuracy

**Cause**: Incorrect camera transforms or calibration

**Solutions**:
1. Verify camera measurements are accurate (see CAMERA_MEASUREMENT_GUIDE.md)
2. Calibrate cameras in PhotonVision (Camera Calibration tab)
3. Adjust standard deviations in Constants.java
4. Ensure cameras have clear view of multiple tags

---

## Testing Without Physical Hardware

If you want to test the code without physical cameras:

### Option 1: Simulation Mode
The code includes simulation support via `VisionSim.java`. When running in simulation:
- Vision errors are expected and can be ignored
- Use WPILib simulation GUI to test robot code
- Vision pose estimation won't work, but other code will

### Option 2: Disable Vision Temporarily
If you want to deploy without vision errors:

1. Comment out vision initialization in `Robot.java`:
```java
// Temporarily disable vision for testing
// NetworkTableInstance inst = NetworkTableInstance.getDefault();
// inst.startClient3("10.80.46.11");
// CameraServer.addServer("http://10.80.46.11:1181/stream.mjpg");
// CameraServer.addServer("http://10.80.46.11:1184/stream.mjpg");
```

2. Comment out vision updates in `Robot.java`:
```java
@Override
public void robotPeriodic() {
    m_timeAndJoystickReplay.update();
    CommandScheduler.getInstance().run();
    
    // Temporarily disable vision updates
    // m_robotContainer.updateVisionMeasurements();
    // m_robotContainer.updateField2d();
}
```

3. Re-enable when cameras are ready!

---

## Deployment Checklist

Before deploying to robot:

- [ ] Orange Pi is powered and connected
- [ ] PhotonVision is running (can access web interface)
- [ ] Both cameras are configured in PhotonVision
- [ ] Camera names match: `CAM_BL` and `CAM_BR`
- [ ] AprilTag detection is enabled
- [ ] Camera transforms are measured and updated in Constants.java
- [ ] Code compiles without errors: `./gradlew build`

Then deploy:
```bash
./gradlew deploy
```

---

## Verification After Deployment

1. **Check SmartDashboard/Shuffleboard**:
   - Look for camera connection status
   - `BL Camera Connected` should be true
   - `BR Camera Connected` should be true

2. **Point cameras at AprilTags**:
   - `BL Target Visible` should become true when tag is visible
   - `BR Target Visible` should become true when tag is visible
   - `Detected Tag ID` should show the tag number

3. **Check pose estimation**:
   - `Vision/BL/Pose Valid` should be true when tag is detected
   - `Vision/BR/Pose Valid` should be true when tag is detected
   - Field2d should show robot position updating

4. **Monitor for errors**:
   - Check Driver Station for any error messages
   - Watch console output for vision-related errors

---

## Getting Help

If you're still having issues:

1. **Check PhotonVision logs**:
   - In PhotonVision web interface, go to "Settings" → "Logs"
   - Look for error messages

2. **Check robot logs**:
   - In Driver Station, check the console output
   - Look for vision-related errors

3. **Verify network**:
   - Use `ping` to verify Orange Pi is reachable
   - Check that robot radio is working properly

4. **PhotonVision documentation**:
   - https://docs.photonvision.org
   - Comprehensive guides for setup and troubleshooting

---

## Summary

The "Could not find PhotonVision coprocessors" error is **normal** until you:
1. ✅ Have PhotonVision running on your coprocessor
2. ✅ Have cameras configured with correct names
3. ✅ Have network connection established
4. ✅ Deploy code to actual robot (not simulation)

**Your code is correct!** This is just a connection/configuration issue that will resolve once your hardware is set up.
