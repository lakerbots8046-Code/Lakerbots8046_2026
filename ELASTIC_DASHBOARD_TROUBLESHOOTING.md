# Elastic Dashboard Troubleshooting Guide

## Camera Streams Not Showing in Elastic

### Quick Checklist

1. **Is the robot code deployed and running?**
   - Deploy the code to the robot
   - Verify robot is powered on and connected

2. **Is Elastic connected to the robot?**
   - Check the connection indicator in Elastic (top right)
   - Should show "Connected" with robot IP
   - Default team number: 8046
   - Robot IP should be: `10.80.46.2` or `roboRIO-8046-FRC.local`

3. **Are you looking in the right place?**
   - In Elastic, go to NetworkTables view
   - Look for these keys:
     - `SmartDashboard/Camera/BL/Stream URL`
     - `SmartDashboard/Camera/BR/Stream URL`
     - `CameraPublisher/CAM_BL`
     - `CameraPublisher/CAM_BR`

4. **Is PhotonVision running?**
   - Navigate to `http://photonvision.local:5800` in browser
   - Verify both cameras are connected and streaming
   - Check that camera names match: `CAM_BL` and `CAM_BR`

### Step-by-Step: Adding Camera Feeds to Elastic

#### Method 1: Using NetworkTables Camera Publisher

1. **Open Elastic Dashboard**
2. **Connect to Robot:**
   - Enter team number: `8046`
   - Or IP: `10.80.46.2`
   - Click "Connect"

3. **Add Camera Widget:**
   - Right-click on dashboard → "Add Widget"
   - Select "Camera Stream" or "MJPEG Stream"
   - Look for `CameraPublisher/CAM_BL` or `CameraPublisher/CAM_BR`

4. **If cameras don't appear in dropdown:**
   - Use "MJPEG Stream" widget instead
   - Manually enter URL: `http://photonvision.local:1184/stream.mjpg` (BL)
   - Or: `http://photonvision.local:1182/stream.mjpg` (BR)

#### Method 2: Direct MJPEG Stream

1. **Add MJPEG Stream Widget**
2. **Enter Stream URL:**
   - For BL camera: `http://photonvision.local:1184/stream.mjpg`
   - For BR camera: `http://photonvision.local:1182/stream.mjpg`
3. **Adjust widget size** as needed

### Verifying NetworkTables Data

#### Check if data is being published:

1. **Open NetworkTables Viewer in Elastic**
2. **Look for these tables:**
   ```
   SmartDashboard/
   ├── Camera/
   │   ├── BL/
   │   │   ├── Stream URL
   │   │   └── Name
   │   └── BR/
   │       ├── Stream URL
   │       └── Name
   ├── Vision Tag Selector
   ├── Target Tag ID
   ├── Detected Tags
   ├── BL Target Visible
   ├── BR Target Visible
   └── ... (50+ other vision keys)
   
   CameraPublisher/
   ├── CAM_BL/
   │   ├── source
   │   ├── streams
   │   ├── description
   │   ├── connected
   │   └── mode
   └── CAM_BR/
       ├── source
       ├── streams
       ├── description
       ├── connected
       └── mode
   ```

3. **If you don't see these:**
   - Robot code may not be running
   - Check Driver Station for errors
   - Verify code deployed successfully

### Common Issues and Solutions

#### Issue: "No cameras in dropdown"
**Solution:**
- Use MJPEG Stream widget with manual URL entry
- Verify PhotonVision is running at `photonvision.local:5800`
- Check that camera names in PhotonVision match `CAM_BL` and `CAM_BR`

#### Issue: "Stream shows black screen"
**Solution:**
- Check PhotonVision web interface - can you see the stream there?
- Verify camera is connected and working in PhotonVision
- Check network connectivity to `photonvision.local`
- Try accessing stream directly in browser: `http://photonvision.local:1184/stream.mjpg`

#### Issue: "Can't connect to photonvision.local"
**Solution:**
- Try using IP address instead: `http://10.80.46.11:1184/stream.mjpg`
- Verify coprocessor (Orange Pi/Raspberry Pi) is powered on
- Check network connection between robot and coprocessor
- Ping the coprocessor: `ping photonvision.local` or `ping 10.80.46.11`

#### Issue: "NetworkTables shows no data"
**Solution:**
- Verify robot code is deployed and running
- Check Driver Station for code errors
- Ensure Elastic is connected to correct robot IP
- Try restarting robot code
- Check console output for camera initialization messages

### Testing Camera Streams Directly

#### Test in Web Browser:
1. **Open browser**
2. **Navigate to:**
   - BL Camera: `http://photonvision.local:1184/stream.mjpg`
   - BR Camera: `http://photonvision.local:1182/stream.mjpg`
3. **You should see live video**
   - If not, PhotonVision isn't streaming properly

#### Test PhotonVision Interface:
1. **Navigate to:** `http://photonvision.local:5800`
2. **Check:**
   - Both cameras appear in camera list
   - Camera names are `CAM_BL` and `CAM_BR`
   - Streams are active (you can see video)
   - AprilTag pipeline is running

### Network Configuration

#### Verify Network Setup:
- **Robot IP:** `10.80.46.2` (or `roboRIO-8046-FRC.local`)
- **PhotonVision IP:** `10.80.46.11` (or `photonvision.local`)
- **Driver Station IP:** `10.80.46.5`
- **Your Computer:** Should be on `10.80.46.x` network

#### Check Robot.java Configuration:
The camera streams are published in `Robot.java` constructor:
```java
NetworkTable cameraTable = inst.getTable("CameraPublisher");

// BL Camera
NetworkTable camBLTable = cameraTable.getSubTable("CAM_BL");
camBLTable.getEntry("streams").setStringArray(new String[]{
    "http://photonvision.local:1184/stream.mjpg"
});

// BR Camera  
NetworkTable camBRTable = cameraTable.getSubTable("CAM_BR");
camBRTable.getEntry("streams").setStringArray(new String[]{
    "http://photonvision.local:1182/stream.mjpg"
});
```

### Alternative: Use SmartDashboard (Classic)

If Elastic isn't working, try SmartDashboard:

1. **Open SmartDashboard** (comes with WPILib)
2. **Connect to robot** (team number 8046)
3. **Add Camera Widget:**
   - View → Add → Camera Stream
   - Select `CAM_BL` or `CAM_BR`
4. **Or add MJPEG Stream:**
   - View → Add → MJPEG Stream
   - Enter URL manually

### Console Output to Check

When robot code starts, you should see:
```
Camera streams published to NetworkTables:
  BL: http://photonvision.local:1184/stream.mjpg
  BR: http://photonvision.local:1182/stream.mjpg
```

If you don't see this, the camera publisher isn't initializing.

### Still Not Working?

1. **Check Driver Station Console:**
   - Look for errors related to NetworkTables or cameras
   - Check for PhotonVision connection errors

2. **Verify PhotonVision Settings:**
   - Camera names must be exactly `CAM_BL` and `CAM_BR`
   - Streams must be enabled
   - Ports must be 1184 (BL) and 1182 (BR)

3. **Try Restarting:**
   - Restart robot code
   - Restart PhotonVision
   - Restart Elastic Dashboard
   - Power cycle robot if needed

4. **Check Firewall:**
   - Ensure firewall isn't blocking ports 1182, 1184, or 5800
   - Try disabling firewall temporarily for testing

### Getting Help

If still having issues, provide:
- Driver Station console output
- PhotonVision web interface screenshot
- Elastic NetworkTables view screenshot
- Robot code deployment log
- Network configuration details

## Quick Reference

### Camera Stream URLs
- **BL Camera:** `http://photonvision.local:1184/stream.mjpg`
- **BR Camera:** `http://photonvision.local:1182/stream.mjpg`

### PhotonVision Interface
- **URL:** `http://photonvision.local:5800`
- **Alternative:** `http://10.80.46.11:5800`

### NetworkTables Keys
- **Camera URLs:** `SmartDashboard/Camera/BL/Stream URL` and `SmartDashboard/Camera/BR/Stream URL`
- **Camera Publisher:** `CameraPublisher/CAM_BL` and `CameraPublisher/CAM_BR`
- **Vision Data:** `SmartDashboard/Vision/*` (50+ keys)
- **Field2d:** `SmartDashboard/Field2d`

### Team Configuration
- **Team Number:** 8046
- **Robot IP:** 10.80.46.2
- **PhotonVision IP:** 10.80.46.11
