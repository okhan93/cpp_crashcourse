# Session 4: OptiTrack Motion Tracking - Study Notes

## Table of Contents
1. [What is OptiTrack?](#what-is-optitrack)
2. [How Motion Capture Works](#how-it-works)
3. [Key Concepts](#key-concepts)
4. [The Tracking Pipeline](#tracking-pipeline)
5. [Rigid Bodies](#rigid-bodies)
6. [Markers and Marker Sets](#markers)
7. [Tracking Quality](#tracking-quality)
8. [Frame Data Structure](#frame-data)
9. [Real OptiTrack SDK Patterns](#sdk-patterns)
10. [Common Challenges](#challenges)
11. [Interview Questions](#interview-questions)

---

<a name="what-is-optitrack"></a>
## 1. What is OptiTrack?

### Overview
**OptiTrack** is a motion capture system that tracks objects in 3D space using:
- **Infrared cameras** (usually 6-12+ cameras)
- **Reflective markers** (small spheres that reflect IR light)
- **Triangulation** (multiple camera views → 3D position)

### Common Applications
- **VR/AR:** Track headsets, controllers, full body
- **Medical Research:** Track surgical tools, patient movement, rehabilitation
- **Robotics:** Track robot position, end-effector position
- **Animation:** Motion capture for films and games
- **Sports Science:** Analyze athlete biomechanics

### Why OptiTrack for Medical VR?
- **Sub-millimeter accuracy** (critical for surgery simulation)
- **Low latency** (~5ms, 120+ FPS)
- **Large tracking volume** (entire operating room)
- **No drift** (unlike IMU-based tracking)
- **Occlusion handling** (if some markers blocked, still tracks)

---

<a name="how-it-works"></a>
## 2. How Motion Capture Works

### The Hardware Setup

```
Camera Setup (typical):
        [CAM1]                    [CAM2]
          \                        /
           \                      /
            \    [Headset]       /
             \   with markers   /
              \                /
               \              /
        [CAM3]  \            /  [CAM4]
                 \          /
                  v        v
               Tracking Volume
```

**Each camera:**
- Emits infrared light
- Captures reflections from markers
- Sends 2D marker positions to PC
- Runs at 120-240 FPS

**The PC:**
- Receives 2D marker positions from all cameras
- Triangulates to find 3D positions
- Groups markers into rigid bodies
- Calculates position + rotation for each rigid body
- Streams data to applications

### The Basic Process

```
┌─────────────────────────────────────────────────────┐
│ STEP 1: Camera Capture                              │
│ Each camera sees markers as bright 2D blobs         │
│ CAM1: marker at pixel (452, 301)                    │
│ CAM2: marker at pixel (198, 287)                    │
│ CAM3: marker at pixel (523, 344)                    │
└─────────────────────────────────────────────────────┘
                      ↓
┌─────────────────────────────────────────────────────┐
│ STEP 2: Triangulation                               │
│ Combine 2D views → 3D world position                │
│ Marker 3D position: (1.23, 1.87, 0.45) meters       │
└─────────────────────────────────────────────────────┘
                      ↓
┌─────────────────────────────────────────────────────┐
│ STEP 3: Marker Grouping                             │
│ "These 4 markers belong to the headset"             │
│ "These 3 markers belong to left controller"         │
└─────────────────────────────────────────────────────┘
                      ↓
┌─────────────────────────────────────────────────────┐
│ STEP 4: Pose Estimation                             │
│ Calculate rigid body position + rotation            │
│ Headset: pos=(1.2, 1.8, 0.5), rot=quaternion        │
└─────────────────────────────────────────────────────┘
                      ↓
┌─────────────────────────────────────────────────────┐
│ STEP 5: Send to Application                         │
│ VR app receives tracking data via NatNet SDK        │
│ Updates virtual camera/objects                      │
└─────────────────────────────────────────────────────┘
```

### Triangulation Explained

**How do we get 3D from 2D?**

```
Side View:

CAM1 sees marker         CAM2 sees marker
at pixel (x1, y1)        at pixel (x2, y2)

   CAM1                      CAM2
     \                        /
      \                      /
       \    ray1        ray2/
        \                  /
         \                /
          \              /
           \            /
            \          /
             \        /
              v      v
            MARKER (3D intersection)
```

**The math:**
- Each camera creates a **ray** from its lens through the pixel
- Rays from 2+ cameras intersect at the marker's 3D position
- More cameras = more accurate (OptiTrack uses 3+ for redundancy)

**Real calculation:**
```cpp
// For each camera:
Ray ray = camera.pixelToRay(pixelX, pixelY);

// Find intersection of all rays (least-squares optimization)
Vec3 marker3D = triangulate(ray1, ray2, ray3, ...);
```

---

<a name="key-concepts"></a>
## 3. Key Concepts

### 3.1 Markers

**What they are:**
- Small reflective spheres (usually 8-14mm diameter)
- Covered in retro-reflective material (like road signs)
- Passive (no batteries, just reflect IR light)

**Types:**
1. **Individual markers** - Tracked separately, can be anywhere
2. **Rigid body markers** - Fixed to an object, move together
3. **Skeleton markers** - Attached to body joints for full-body tracking

### 3.2 Rigid Bodies

**Definition:** A set of markers that maintain **fixed relative positions**

```
Headset Rigid Body:
    M2
    |
M1--+--M3    <- 4 markers in fixed configuration
    |
    M4

When headset moves/rotates, all 4 markers move together
maintaining their relative positions.
```

**Key properties:**
- **ID:** Unique identifier (e.g., headset = ID 1, controller = ID 2)
- **Position:** Center point in world space (Vec3)
- **Rotation:** Orientation (Quaternion or rotation matrix)
- **Marker count:** How many markers define this rigid body
- **Marker offsets:** Fixed local positions of each marker

**Why rigid bodies?**
- Track complex objects with single position/rotation
- More robust (if 1 marker occluded, still tracks)
- Easier for applications (get headset pose, not 4 marker positions)

### 3.3 Tracking Quality

**Mean Error:** Average distance between:
- Where markers **should be** (based on calculated pose)
- Where markers **actually are** (from cameras)

```cpp
float calculateMeanError(RigidBody& rb, Vec3 markerPositions[]) {
    float totalError = 0.0f;

    for (int i = 0; i < rb.numMarkers; i++) {
        // Where marker should be
        Vec3 expected = rb.transform.transformPoint(rb.markerOffsets[i]);

        // Where it actually is
        Vec3 actual = markerPositions[i];

        // Distance between them
        float error = (expected - actual).magnitude();
        totalError += error;
    }

    return totalError / rb.numMarkers;
}
```

**Interpretation:**
- **< 0.5mm:** Excellent tracking
- **0.5-1.0mm:** Good tracking
- **1.0-2.0mm:** Acceptable (maybe some occlusion)
- **> 2.0mm:** Poor (check for reflections, occlusion, calibration)

### 3.4 Occlusion

**Problem:** Markers can be blocked from camera view

```
    CAM1         CAM2         CAM3
      \            |            /
       \           |           /
        \          |          /
         \      [HAND]       /
          \        X        /     <- Marker blocked by hand
           \       |       /
            \  [Marker]  /
             \          /

CAM2 can't see marker, but CAM1 and CAM3 can!
```

**OptiTrack's solution:**
- Need **at least 2 cameras** to see a marker for 3D reconstruction
- Rigid bodies can tolerate some markers being occluded
- **Minimum marker requirement:** Need to see 3+ markers to track a rigid body

**Tracking states:**
```cpp
enum TrackingState {
    NOT_TRACKED,        // < 3 markers visible
    TRACKED_PARTIAL,    // 3+ markers, but some occluded
    TRACKED_FULL        // All markers visible
};
```

---

<a name="tracking-pipeline"></a>
## 4. The Tracking Pipeline

### Frame-by-Frame Process

```cpp
// OptiTrack runs this every frame (120 FPS)

void processFrame() {
    // 1. CAPTURE: Get 2D marker positions from all cameras
    std::vector<Marker2D> markers2D = captureFromCameras();

    // 2. TRIANGULATE: Convert 2D → 3D
    std::vector<Vec3> markers3D = triangulate(markers2D);

    // 3. IDENTIFY: Which markers belong to which rigid body?
    assignMarkersToRigidBodies(markers3D);

    // 4. SOLVE: Calculate position + rotation for each rigid body
    for (RigidBody& rb : rigidBodies) {
        solvePose(rb, markers3D);
    }

    // 5. VALIDATE: Check tracking quality
    for (RigidBody& rb : rigidBodies) {
        rb.meanError = calculateMeanError(rb, markers3D);
        rb.isTracked = (rb.visibleMarkers >= 3);
    }

    // 6. STREAM: Send data to applications
    sendFrameData(rigidBodies);
}
```

### Pose Estimation (The Hard Part)

**Problem:** Given marker positions, find rigid body's position and rotation.

**Input:**
- 4 marker positions in world space (from triangulation)
- 4 marker positions in local space (from rigid body definition)

**Output:**
- Rigid body position (Vec3)
- Rigid body rotation (Quaternion)

**Algorithm (simplified):**
```cpp
Transform solvePose(Vec3 markersWorld[], Vec3 markersLocal[], int count) {
    // 1. Find centroid (average position)
    Vec3 centroidWorld = average(markersWorld, count);
    Vec3 centroidLocal = average(markersLocal, count);

    // 2. Center the marker sets
    Vec3 worldCentered[count];
    Vec3 localCentered[count];
    for (int i = 0; i < count; i++) {
        worldCentered[i] = markersWorld[i] - centroidWorld;
        localCentered[i] = markersLocal[i] - centroidLocal;
    }

    // 3. Find rotation that best aligns local → world
    // (Uses SVD/Kabsch algorithm - complex, but standard)
    Quaternion rotation = findOptimalRotation(localCentered, worldCentered, count);

    // 4. Position is world centroid
    Vec3 position = centroidWorld;

    return Transform(position, rotation);
}
```

**Real implementation:** OptiTrack uses optimized algorithms like:
- **Kabsch algorithm** (SVD-based)
- **RANSAC** (handles outliers)
- **Iterative refinement** (improves accuracy)

---

<a name="rigid-bodies"></a>
## 5. Rigid Bodies

### Creating a Rigid Body

**In Motive (OptiTrack software):**
1. Place object with markers in tracking volume
2. Select all markers belonging to object
3. Click "Create Rigid Body"
4. Motive records marker offsets automatically
5. Export rigid body definition

**The definition file contains:**
```
Rigid Body ID: 1
Name: "Headset"
Marker Count: 4
Marker Offsets (local space):
  Marker 0: (0.05, 0.03, 0.02)
  Marker 1: (-0.05, 0.03, 0.02)
  Marker 2: (0.05, 0.03, -0.02)
  Marker 3: (-0.05, 0.03, -0.02)
```

### Rigid Body Constraints

**What makes it "rigid"?**
- Marker distances **never change**
- Markers only move together through **rotation + translation**
- No bending, stretching, or deformation

**Validation:**
```cpp
bool validateRigidBody(Vec3 markerOffsets[], int count) {
    // Calculate all pairwise distances
    float distances[count][count];
    for (int i = 0; i < count; i++) {
        for (int j = i+1; j < count; j++) {
            distances[i][j] = (markerOffsets[i] - markerOffsets[j]).magnitude();
        }
    }

    // Later, check if distances are maintained
    for (int i = 0; i < count; i++) {
        for (int j = i+1; j < count; j++) {
            float currentDist = (currentMarkers[i] - currentMarkers[j]).magnitude();
            float expectedDist = distances[i][j];
            float error = abs(currentDist - expectedDist);

            if (error > 2.0f) {  // 2mm tolerance
                return false;  // Not rigid!
            }
        }
    }
    return true;
}
```

### Multi-Rigid-Body Tracking

**Typical VR setup:**
```cpp
RigidBody headset;      // ID 1, 4 markers
RigidBody leftHand;     // ID 2, 3 markers
RigidBody rightHand;    // ID 3, 3 markers
RigidBody tracker1;     // ID 4, 3 markers (foot/hip)
```

**Challenge:** Identifying which markers belong to which rigid body

```
Frame has 13 markers total detected.
Which 4 belong to headset?
Which 3 belong to left hand?
...
```

**OptiTrack's approach:**
1. **Pattern matching:** Unique marker configurations
2. **Proximity:** Markers close together likely belong to same object
3. **Temporal coherence:** Track frame-to-frame (headset can't teleport)
4. **Unique marker sizes:** Different sized markers for different objects

---

<a name="markers"></a>
## 6. Markers and Marker Sets

### Marker States

```cpp
enum MarkerState {
    UNIDENTIFIED,    // Seen, but don't know which rigid body
    IDENTIFIED,      // Assigned to a rigid body
    OCCLUDED,        // Should exist but not visible
    PREDICTED        // Position estimated from previous frames
};
```

### Active Markers vs Passive Markers

**Passive (OptiTrack standard):**
- Just reflective spheres
- No electronics
- Cheap, lightweight
- Can't be individually identified
- Need unique spatial patterns

**Active (some systems):**
- LED-based markers
- Each has unique ID
- More expensive, heavier
- Need batteries
- Easier to identify

### Marker Labeling

**The problem:** Frame has 13 markers. Which is which?

```
Frame N:
Marker A at (1.2, 1.8, 0.5)
Marker B at (1.15, 1.83, 0.52)
Marker C at (1.25, 1.83, 0.48)
Marker D at (1.2, 1.77, 0.5)
...

Which is headset marker 0?
Which is headset marker 1?
```

**Solution: Pattern matching**
```cpp
// Check if 4 markers match headset's expected pattern
bool matchesHeadsetPattern(Vec3 markers[4]) {
    // Calculate distances between all pairs
    float d01 = (markers[0] - markers[1]).magnitude();
    float d02 = (markers[0] - markers[2]).magnitude();
    float d03 = (markers[0] - markers[3]).magnitude();
    // ... etc

    // Compare to expected distances from rigid body definition
    float expectedD01 = 0.10f;  // 10cm between markers 0 and 1
    float expectedD02 = 0.05f;  // 5cm between markers 0 and 2
    // ... etc

    float tolerance = 0.002f;  // 2mm
    if (abs(d01 - expectedD01) > tolerance) return false;
    if (abs(d02 - expectedD02) > tolerance) return false;
    // ... check all pairs

    return true;
}
```

### Marker Prediction

**When markers are temporarily occluded:**

```cpp
Vec3 predictMarkerPosition(Marker& marker, float deltaTime) {
    // Use velocity to estimate position
    Vec3 velocity = marker.velocity;  // From previous frames
    Vec3 predicted = marker.lastPosition + (velocity * deltaTime);

    marker.state = PREDICTED;
    return predicted;
}
```

**Limits:**
- Only predict for 2-3 frames
- After that, mark rigid body as "not tracked"
- Prevents drift from accumulating

---

<a name="tracking-quality"></a>
## 7. Tracking Quality

### Metrics

**1. Mean Marker Error**
```cpp
float meanError;  // Average distance between expected and actual marker positions
```
- **Best case:** < 0.2mm (ideal conditions)
- **Typical:** 0.5-1.0mm (good tracking)
- **Problematic:** > 2.0mm (check setup)

**2. Marker Visibility**
```cpp
int visibleMarkers;  // How many markers currently visible
int totalMarkers;    // Total markers in rigid body
float visibilityRatio = (float)visibleMarkers / totalMarkers;
```
- **100%:** All markers visible
- **75-99%:** Partial occlusion, still tracking
- **< 50%:** May lose tracking

**3. Frame Rate**
```cpp
float fps;  // Actual frames per second
```
- **120 FPS:** Standard for VR
- **240 FPS:** High-speed applications
- **< 100 FPS:** System overloaded

**4. Latency**
```cpp
float latency;  // Time from capture to application (milliseconds)
```
- **< 5ms:** Excellent (VR standard)
- **5-10ms:** Acceptable
- **> 10ms:** Noticeable lag (check network/CPU)

### Common Tracking Issues

**Issue 1: High Mean Error**
- **Cause:** Poor calibration, reflections, marker swap
- **Solution:** Recalibrate cameras, check for reflective surfaces

**Issue 2: Marker Occlusion**
- **Cause:** Body/object blocking line of sight
- **Solution:** Add more cameras, reposition cameras, add more markers

**Issue 3: Jitter (shaky tracking)**
- **Cause:** Marker reflections, camera noise, insufficient markers
- **Solution:** Smooth with filters, check marker quality, add markers

**Issue 4: ID Swapping**
- **Cause:** Similar rigid bodies, markers too close, poor patterns
- **Solution:** Use unique marker configurations, increase marker spacing

### Tracking Quality in Code

```cpp
class RigidBody {
public:
    float meanError;
    int visibleMarkers;
    int totalMarkers;

    TrackingQuality getQuality() const {
        if (visibleMarkers < 3) {
            return NOT_TRACKED;
        }

        float visibilityRatio = (float)visibleMarkers / totalMarkers;

        if (visibilityRatio == 1.0f && meanError < 0.5f) {
            return EXCELLENT;
        } else if (visibilityRatio >= 0.75f && meanError < 1.0f) {
            return GOOD;
        } else if (visibleMarkers >= 3 && meanError < 2.0f) {
            return ACCEPTABLE;
        } else {
            return POOR;
        }
    }
};
```

---

<a name="frame-data"></a>
## 8. Frame Data Structure

### What You Receive Each Frame

```cpp
struct Frame {
    int frameNumber;
    float timestamp;

    // All markers detected (before grouping)
    int numUnidentifiedMarkers;
    Vec3 unidentifiedMarkers[MAX_MARKERS];

    // Rigid bodies
    int numRigidBodies;
    RigidBodyData rigidBodies[MAX_RIGID_BODIES];

    // Latency info
    float latency;
};

struct RigidBodyData {
    int id;
    Vec3 position;
    Quaternion rotation;

    // Individual markers
    int numMarkers;
    Vec3 markerPositions[MAX_MARKERS_PER_BODY];
    int markerIDs[MAX_MARKERS_PER_BODY];
    float markerSizes[MAX_MARKERS_PER_BODY];

    // Quality metrics
    float meanError;
    bool isTracked;
};
```

### Frame Processing Example

```cpp
void onFrameReceived(Frame& frame) {
    std::cout << "Frame " << frame.frameNumber
              << " at " << frame.timestamp << "s" << std::endl;

    // Process headset
    RigidBodyData* headset = findRigidBody(frame, HEADSET_ID);
    if (headset && headset->isTracked) {
        updateVRCamera(headset->position, headset->rotation);

        if (headset->meanError > 1.0f) {
            std::cout << "WARNING: High tracking error on headset: "
                      << headset->meanError << "mm" << std::endl;
        }
    } else {
        std::cout << "LOST TRACKING: Headset not visible" << std::endl;
        showTrackingLostMessage();
    }

    // Process controllers
    RigidBodyData* leftController = findRigidBody(frame, LEFT_CONTROLLER_ID);
    if (leftController && leftController->isTracked) {
        updateControllerVisual(leftController->position, leftController->rotation);
    }
}
```

---

<a name="sdk-patterns"></a>
## 9. Real OptiTrack SDK Patterns

### NatNet SDK Basics

**NatNet** is OptiTrack's networking protocol for streaming tracking data.

**Connection setup:**
```cpp
#include <NatNetTypes.h>
#include <NatNetClient.h>

NatNetClient* client = new NatNetClient();

// Connect to Motive (OptiTrack software)
client->Connect("127.0.0.1", 1510);  // Local connection

// Set frame callback
client->SetFrameReceivedCallback(onFrameReceived, nullptr);
```

**Frame callback:**
```cpp
void onFrameReceived(sFrameOfMocapData* data, void* pUserData) {
    // Access rigid bodies
    for (int i = 0; i < data->nRigidBodies; i++) {
        sRigidBodyData rb = data->RigidBodies[i];

        // Position
        Vec3 pos = {rb.x, rb.y, rb.z};

        // Rotation (quaternion)
        Quaternion rot = {rb.qx, rb.qy, rb.qz, rb.qw};

        // Tracking info
        bool tracked = rb.params & 0x01;  // Bit flag
        float error = rb.MeanError;

        // Use transform
        updateRigidBody(rb.ID, pos, rot, tracked, error);
    }
}
```

### Common SDK Operations

**1. Get rigid body by ID:**
```cpp
sRigidBodyData* getRigidBody(sFrameOfMocapData* frame, int id) {
    for (int i = 0; i < frame->nRigidBodies; i++) {
        if (frame->RigidBodies[i].ID == id) {
            return &frame->RigidBodies[i];
        }
    }
    return nullptr;
}
```

**2. Transform marker to world space:**
```cpp
Vec3 getMarkerWorldPosition(sRigidBodyData* rb, int markerIndex) {
    // Rigid body transform
    Transform rbTransform(
        Vec3{rb->x, rb->y, rb->z},
        Quaternion{rb->qx, rb->qy, rb->qz, rb->qw}
    );

    // Marker local offset (from rigid body definition)
    Vec3 markerLocal = rigidBodyDefinitions[rb->ID].markerOffsets[markerIndex];

    // Transform to world space
    return rbTransform.transformPoint(markerLocal);
}
```

**3. Check tracking quality:**
```cpp
bool isTrackingGood(sRigidBodyData* rb) {
    bool tracked = rb->params & 0x01;
    float error = rb->MeanError;

    return tracked && (error < 1.0f);  // < 1mm error
}
```

### Coordinate System

**OptiTrack coordinate system (right-handed):**
```
Y (up)
|
|
+------ X (right)
/
/
Z (forward, toward you)
```

**Important:** Some engines use different conventions:
- **Unity:** Y-up, Z-forward, **left-handed**
- **Unreal:** Z-up, X-forward, **left-handed**
- **OpenGL:** Y-up, -Z-forward, **right-handed**

**Conversion example (OptiTrack → Unity):**
```cpp
Vec3 optitrackToUnity(Vec3 optitrack) {
    return {
        optitrack.x,   // X → X
        optitrack.y,   // Y → Y
        -optitrack.z   // Z → -Z (flip for left-handed)
    };
}

Quaternion optitrackToUnity(Quaternion optitrack) {
    return {
        -optitrack.x,
        -optitrack.y,
        optitrack.z,
        optitrack.w
    };
}
```

---

<a name="challenges"></a>
## 10. Common Challenges

### Challenge 1: Marker Identification

**Problem:** Which marker is which?

**Solutions:**
- **Asymmetric patterns:** Make marker configurations unique
- **Temporal tracking:** Track frame-to-frame continuity
- **Active markers:** Use LEDs with IDs (expensive)

### Challenge 2: Occlusion

**Problem:** Markers blocked from camera view

**Solutions:**
- **More cameras:** 360° coverage
- **More markers:** Redundancy
- **Prediction:** Estimate position for 2-3 frames
- **Skeleton constraints:** Use biomechanical limits

### Challenge 3: Reflections

**Problem:** Shiny surfaces create false markers

**Solutions:**
- **Cover reflective surfaces:** Matte paint, cloth
- **Camera masking:** Ignore specific regions
- **Marker size filtering:** Real markers have consistent size
- **Calibration wands:** Define expected marker sizes

### Challenge 4: Calibration Drift

**Problem:** Camera positions shift slightly over time

**Solutions:**
- **Regular recalibration:** Daily or weekly
- **Continuous calibration:** Use fixed reference markers
- **Temperature compensation:** Cameras expand/contract with heat
- **Rigid mounting:** Secure cameras to structure

### Challenge 5: Latency

**Problem:** Data arrives late, causing VR lag

**Solutions:**
- **Wired connection:** Ethernet faster than WiFi
- **Prediction:** Extrapolate position based on velocity
- **Optimize pipeline:** Reduce processing overhead
- **Direct mode:** Bypass Windows compositor

---

<a name="interview-questions"></a>
## 11. Common Interview Questions

### Conceptual Questions

**Q: How does OptiTrack determine a rigid body's position and rotation?**

A: OptiTrack uses triangulation to find 3D positions of markers, then uses a pose estimation algorithm (like Kabsch/SVD) to find the rotation and translation that best maps the rigid body's known local marker positions to the observed world marker positions.

**Q: What's the minimum number of markers needed to track a rigid body?**

A: **3 markers minimum** for tracking, but 4+ recommended for robustness. With 3 points, you can uniquely determine position and rotation. Fewer than 3 is insufficient (2 points have rotational ambiguity around their axis).

**Q: Why use infrared instead of visible light?**

A: Infrared is invisible to humans (doesn't distract users), works in various lighting conditions, has less interference from ambient light, and reflective markers are highly efficient in IR spectrum.

**Q: What causes marker ID swapping?**

A: When rigid bodies have similar marker patterns, pass very close to each other, or when tracking is lost and reacquired. Solution: use asymmetric marker patterns and sufficient marker spacing.

**Q: How does OptiTrack handle partial occlusion?**

A: As long as **3+ markers remain visible**, OptiTrack can still track the rigid body. It uses the visible markers to estimate pose. If fewer than 3 are visible, tracking is lost and position may be predicted for 2-3 frames.

### Technical Questions

**Q: How would you detect if tracking quality is degrading?**

```cpp
bool isTrackingDegrading(RigidBody& rb, float previousError) {
    // Check mean error trend
    if (rb.meanError > previousError * 1.5f) {
        return true;  // Error increasing
    }

    // Check marker visibility
    float visibilityRatio = (float)rb.visibleMarkers / rb.totalMarkers;
    if (visibilityRatio < 0.75f) {
        return true;  // Losing markers
    }

    return false;
}
```

**Q: How would you smooth jittery tracking data?**

```cpp
// Low-pass filter (exponential smoothing)
Vec3 smoothedPos = Vec3{0, 0, 0};
float smoothingFactor = 0.3f;  // 0 = no smoothing, 1 = raw data

void updatePosition(Vec3 newPos) {
    smoothedPos = smoothedPos * (1.0f - smoothingFactor)
                + newPos * smoothingFactor;
}
```

**Q: How do you convert marker positions to rigid body pose?**

```cpp
Transform solvePose(Vec3 markersWorld[], Vec3 markersLocal[], int count) {
    // 1. Find centroids
    Vec3 centroidWorld = computeCentroid(markersWorld, count);
    Vec3 centroidLocal = computeCentroid(markersLocal, count);

    // 2. Center markers
    Vec3 worldCentered[count];
    Vec3 localCentered[count];
    for (int i = 0; i < count; i++) {
        worldCentered[i] = markersWorld[i] - centroidWorld;
        localCentered[i] = markersLocal[i] - centroidLocal;
    }

    // 3. Find optimal rotation (Kabsch algorithm)
    Quaternion rotation = findOptimalRotation(localCentered, worldCentered, count);

    // 4. Position is world centroid
    return Transform(centroidWorld, rotation);
}
```

### Practical Questions

**Q: In a medical VR application, how would you handle tracking loss of a surgical tool?**

A:
1. **Detect loss:** Check `isTracked` flag and `visibleMarkers < 3`
2. **Predict briefly:** Use last known velocity to estimate position for 1-2 frames
3. **Alert user:** Visual/audio warning that tool is not tracked
4. **Freeze tool:** Don't allow virtual tool to move unpredictably
5. **Log event:** Record tracking loss for quality analysis
6. **Recovery:** When tracking resumes, validate position hasn't jumped significantly

**Q: How would you set up OptiTrack for a surgical training scenario?**

A:
1. **Coverage:** 6-8 cameras for 360° coverage of workspace
2. **Rigid bodies:**
   - Headset (4-5 markers)
   - Each surgical tool (3-4 markers)
   - Patient tracker (3-4 markers)
3. **Calibration:** Sub-millimeter accuracy required for surgery
4. **Frequency:** 120 FPS minimum for smooth VR
5. **Validation:** Test occlusion scenarios (hands blocking tools)
6. **Backup:** Consider IMU fusion for temporary occlusion

**Q: What's the difference between world space and local space in motion tracking?**

A:
- **World space:** Fixed global coordinate system (the room)
- **Local space:** Relative to an object's center and rotation

Example:
```cpp
// Marker is 5cm to the right of headset center (local space)
Vec3 markerLocal = {0.05f, 0.0f, 0.0f};

// Headset is at (2, 1.5, 3) and rotated 90° around Y
Transform headset(Vec3{2, 1.5, 3}, Quaternion(Vec3{0,1,0}, M_PI/2));

// Where is the marker in the room? (world space)
Vec3 markerWorld = headset.transformPoint(markerLocal);
// Result: (2.0, 1.5, 2.95) - rotated and translated
```

---

## Quick Reference

### Key Formulas

```cpp
// Centroid
Vec3 centroid = sum(positions) / count;

// Mean error
float meanError = sum(distances) / count;

// Visibility ratio
float visibility = (float)visibleMarkers / totalMarkers;

// Transform point (local → world)
Vec3 worldPos = rotation.rotate(localPos) + position;
```

### Tracking Thresholds

| Metric | Excellent | Good | Acceptable | Poor |
|--------|-----------|------|------------|------|
| Mean Error | < 0.5mm | 0.5-1.0mm | 1.0-2.0mm | > 2.0mm |
| Visibility | 100% | 75-99% | 50-74% | < 50% |
| Latency | < 5ms | 5-10ms | 10-20ms | > 20ms |
| FPS | 120+ | 100-120 | 60-100 | < 60 |

### Rigid Body States

```cpp
NOT_TRACKED     // < 3 markers visible
TRACKED_PARTIAL // 3+ markers, some occluded
TRACKED_FULL    // All markers visible
```

---

## Key Takeaways

✓ **OptiTrack uses triangulation** to convert 2D camera views → 3D marker positions

✓ **Rigid bodies** are sets of markers with fixed relative positions

✓ **Minimum 3 markers** needed to track a rigid body

✓ **Pose estimation** finds position + rotation from marker positions

✓ **Mean error** measures tracking accuracy (< 1mm is good)

✓ **Occlusion handling** allows tracking with partial marker visibility

✓ **120 FPS** is standard for VR applications

✓ **NatNet SDK** streams data to applications via network

**You're now ready to implement an OptiTrack simulation!** 🚀
