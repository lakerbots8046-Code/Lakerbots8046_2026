# Elastic Dashboard Outputs - Quick Reference

## 🎯 **TO SEE IF TAGS 9 & 10 ARE DETECTED - USE THESE:**

### **Step 1: Check Total Tags Detected (Shows ALL tags, not just selected)**
```
BL Total Targets: 2          ← How many tags BL camera sees
BR Total Targets: 2          ← How many tags BR camera sees
Vision/BL/Tags Used IDs: "9, 10"  ← ALL tag IDs BL camera sees
Vision/BR/Tags Used IDs: "9, 10"  ← ALL tag IDs BR camera sees
```
**👆 START HERE - These show ALL detected tags regardless of selector**

### **Step 2: Select Specific Tag to Track**
```
Vision Tag Selector: [Dropdown - Select Tag 9 or 10]
Target Tag ID: 9             ← Shows which tag you selected
Target Tag Found: true       ← Is selected tag visible?
Target Visible On: "Back-Left Camera"  ← Which camera sees it
Detected Tags: "BL: Tag 9"   ← Summary of what's detected
```

### **Step 3: Individual Camera Details (for selected tag only)**
```
BL Detected Tag ID: 9        ← Tag ID BL camera sees (only selected tag)
BL Target Visible: true      ← Is selected tag visible on BL?
BL Target Yaw (deg): -5.2    ← Horizontal angle to tag
BL Target Pitch (deg): 12.3  ← Vertical angle to tag
BL Target Area (%): 2.5      ← How big tag appears
BL Approx Distance: 6.3      ← Estimated distance

BR Detected Tag ID: -1       ← Tag ID BR camera sees (only selected tag)
BR Target Visible: false     ← Is selected tag visible on BR?
BR Target Yaw (deg): 0.0
BR Target Pitch (deg): 0.0
BR Target Area (%): 0.0
BR Approx Distance: 0.0
```

---

## 📊 **COMPLETE LIST OF ALL ACTIVE OUTPUTS**

### **🎮 Tag Selection & Status**
| Output Name | Type | Description |
|------------|------|-------------|
| `Vision Tag Selector` | Chooser | Dropdown to select which tag to track (1-24) |
| `Target Tag ID` | Number | Currently selected tag ID |
| `Target Tag Found` | Boolean | Is the selected tag visible by any camera? |
| `Target Visible On` | String | Which camera sees selected tag: "Back-Left Camera", "Back-Right Camera", "Both Cameras", or "None" |
| `Detected Tags` | String | Summary of detected tags (e.g., "BL: Tag 9 \| BR: Tag 10") |

### **📷 Back-Left (BL) Camera - Selected Tag Only**
| Output Name | Type | Description |
|------------|------|-------------|
| `BL Camera Connected` | Boolean | Is BL camera connected? |
| `BL Target Visible` | Boolean | Is selected tag visible? |
| `BL Detected Tag ID` | Number | ID of selected tag (-1 if not visible) |
| `BL Target Yaw (deg)` | Number | Horizontal angle to tag |
| `BL Target Pitch (deg)` | Number | Vertical angle to tag |
| `BL Target Area (%)` | Number | Tag size in camera view |
| `BL Approx Distance` | Number | Estimated distance to tag |
| `BL Total Targets` | Number | **Total number of tags BL sees (ALL tags)** |

### **📷 Back-Right (BR) Camera - Selected Tag Only**
| Output Name | Type | Description |
|------------|------|-------------|
| `BR Camera Connected` | Boolean | Is BR camera connected? |
| `BR Target Visible` | Boolean | Is selected tag visible? |
| `BR Detected Tag ID` | Number | ID of selected tag (-1 if not visible) |
| `BR Target Yaw (deg)` | Number | Horizontal angle to tag |
| `BR Target Pitch (deg)` | Number | Vertical angle to tag |
| `BR Target Area (%)` | Number | Tag size in camera view |
| `BR Approx Distance` | Number | Estimated distance to tag |
| `BR Total Targets` | Number | **Total number of tags BR sees (ALL tags)** |

### **🗺️ Vision Pose Estimation - ALL Tags**
| Output Name | Type | Description |
|------------|------|-------------|
| `Vision/BL/Pose Valid` | Boolean | Does BL have valid pose estimate? |
| `Vision/BL/Pose X` | Number | Robot X position from BL camera |
| `Vision/BL/Pose Y` | Number | Robot Y position from BL camera |
| `Vision/BL/Pose Rotation` | Number | Robot rotation from BL camera |
| `Vision/BL/Pose Timestamp` | Number | When pose was calculated |
| `Vision/BL/Tags Used` | Number | How many tags used for pose |
| `Vision/BL/Tags Used IDs` | String | **ALL tag IDs used (e.g., "9, 10")** |
| `Vision/BR/Pose Valid` | Boolean | Does BR have valid pose estimate? |
| `Vision/BR/Pose X` | Number | Robot X position from BR camera |
| `Vision/BR/Pose Y` | Number | Robot Y position from BR camera |
| `Vision/BR/Pose Rotation` | Number | Robot rotation from BR camera |
| `Vision/BR/Pose Timestamp` | Number | When pose was calculated |
| `Vision/BR/Tags Used` | Number | How many tags used for pose |
| `Vision/BR/Tags Used IDs` | String | **ALL tag IDs used (e.g., "9, 10")** |
| `Vision/Any Pose Valid` | Boolean | Does any camera have valid pose? |
| `Vision/Total Tags Used` | Number | Total tags used by both cameras |

### **📹 Camera Streams**
| Output Name | Type | Description |
|------------|------|-------------|
| `Camera/BL/Stream URL` | String | http://10.80.46.11:1184/stream.mjpg |
| `Camera/BR/Stream URL` | String | http://10.80.46.11:1182/stream.mjpg |
| `Camera/BL/Name` | String | CAM_BL |
| `Camera/BR/Name` | String | CAM_BR |

### **⚠️ Legacy Outputs (For Compatibility - Ignore These)**
| Output Name | Type | Description |
|------------|------|-------------|
| `AprilTag [ID] Yaw BL` | Number | OLD - Use "BL Target Yaw (deg)" instead |
| `AprilTag [ID] Yaw BR` | Number | OLD - Use "BR Target Yaw (deg)" instead |
| `Vision Target Visible BL` | Boolean | OLD - Use "BL Target Visible" instead |
| `Vision Target Visible BR` | Boolean | OLD - Use "BR Target Visible" instead |

---

## 🔍 **TROUBLESHOOTING: PhotonVision Sees Tags But Dashboard Doesn't**

### **Check These in Order:**

1. **Camera Connection**
   - Look at: `BL Camera Connected` and `BR Camera Connected`
   - Should both be `true`
   - If `false`: Check NetworkTables connection (10.80.46.2:5810)

2. **Total Tags Detected**
   - Look at: `BL Total Targets` and `BR Total Targets`
   - Should show `2` if both tags 9 & 10 are visible
   - If `0`: PhotonVision is running but not publishing to NetworkTables

3. **Tag IDs Being Used**
   - Look at: `Vision/BL/Tags Used IDs` and `Vision/BR/Tags Used IDs`
   - Should show: `"9, 10"` or similar
   - If empty: Pose estimation is not running

4. **Selected Tag Tracking**
   - Set `Vision Tag Selector` to Tag 9
   - Look at: `BL Detected Tag ID` - should show `9`
   - Look at: `Target Tag Found` - should show `true`
   - If `-1` or `false`: Tag selector is working but tag 9 is not in view

### **Common Issues:**

**Issue:** PhotonVision shows tags but `BL Total Targets` = 0
- **Fix:** Check NetworkTables connection in Robot.java (should be 10.80.46.2)
- **Fix:** Restart robot code
- **Fix:** Check PhotonVision is publishing to NetworkTables (Settings → Output)

**Issue:** `Vision/BL/Tags Used IDs` is empty but `BL Total Targets` > 0
- **Fix:** This is normal - pose estimation only uses tags when it can calculate a valid pose
- **Check:** `Vision/BL/Pose Valid` - if false, pose estimation failed

**Issue:** Selected tag shows `-1` even though `BL Total Targets` = 2
- **Explanation:** The tag selector only tracks ONE tag at a time
- **Fix:** Make sure you selected the correct tag in `Vision Tag Selector`
- **Example:** If tags 9 & 10 are visible but you selected tag 14, `BL Detected Tag ID` will be `-1`

---

## ✅ **QUICK TEST PROCEDURE**

1. **Open Elastic Dashboard**
2. **Add these widgets:**
   - `BL Total Targets` (Number)
   - `BR Total Targets` (Number)
   - `Vision/BL/Tags Used IDs` (Text)
   - `Vision/BR/Tags Used IDs` (Text)
   - `Vision Tag Selector` (Chooser)
   - `Target Tag Found` (Boolean)
   - `Target Visible On` (Text)

3. **Point cameras at tags 9 & 10**
4. **Check:**
   - `BL Total Targets` should show `2`
   - `Vision/BL/Tags Used IDs` should show `"9, 10"`

5. **Select Tag 9 from dropdown**
6. **Check:**
   - `Target Tag Found` should be `true`
   - `Target Visible On` should show which camera sees it
   - `BL Detected Tag ID` should show `9`

7. **Select Tag 10 from dropdown**
8. **Repeat step 6**

---

## 📝 **NOTES**

- **Selected Tag vs All Tags:** The tag selector only affects individual camera outputs (BL/BR Detected Tag ID, Yaw, Pitch, etc.). Pose estimation and "Tags Used IDs" show ALL detected tags.
- **Pose Estimation:** Uses multiple tags automatically for better accuracy. Check `Vision/BL/Tags Used IDs` to see which tags are being used.
- **Camera Streams:** Add camera widgets in Elastic using the URLs from `Camera/BL/Stream URL` and `Camera/BR/Stream URL`
