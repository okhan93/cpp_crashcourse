# Session 3: Motion Tracking Fundamentals - Study Notes

## Table of Contents
1. [Why This Matters for OptiTrack/VR](#why-this-matters)
2. [3D Coordinate Systems](#coordinate-systems)
3. [Vectors - The Foundation](#vectors)
4. [Vector Operations](#vector-operations)
5. [Quaternions - The Right Way to Rotate](#quaternions)
6. [Transforms - Position + Rotation](#transforms)
7. [Real-World Applications](#real-world-applications)
8. [Common Interview Questions](#interview-questions)

---

<a name="why-this-matters"></a>
## 1. Why This Matters for OptiTrack/VR

### What OptiTrack Does
OptiTrack cameras track **reflective markers** in 3D space. When you wear a VR headset with markers attached:
1. Cameras detect marker positions (x, y, z coordinates)
2. Software calculates the headset's **position** and **rotation**
3. VR system uses this to update what you see

### The Math You Need
- **Vectors:** Represent positions, directions, velocities
- **Quaternions:** Represent rotations (better than Euler angles)
- **Transforms:** Combine position + rotation to describe object state

**In interviews, they want to see you understand the 3D math behind tracking systems.**

---

<a name="coordinate-systems"></a>
## 2. 3D Coordinate Systems

### Right-Handed Coordinate System (OptiTrack Standard)

```
      Y (up)
      |
      |
      +------ X (right)
     /
    /
   Z (forward toward you)
```

**Conventions:**
- **X-axis:** Right
- **Y-axis:** Up
- **Z-axis:** Forward (toward you in OpenGL/OptiTrack)

### World Space vs Local Space

**World Space:** The global coordinate system (the room)
```
VR Headset at world position (2, 1.5, 3)
```

**Local Space:** Relative to an object
```
Marker on headset at local position (0.1, 0.05, -0.1) relative to headset center
```

**Transformation:** Converting local → world coordinates
```
World Position = Object Position + Rotate(Local Position)
```

---

<a name="vectors"></a>
## 3. Vectors - The Foundation

### What is a Vector?

A vector is a **direction and magnitude** in 3D space.

```cpp
struct Vec3 {
    float x, y, z;
};

Vec3 position = {1.0f, 2.0f, 3.0f};     // A point in space
Vec3 velocity = {0.5f, 0.0f, -0.2f};    // Movement direction and speed
Vec3 forward = {0.0f, 0.0f, -1.0f};     // Direction headset is facing
```

### Visualizing Vectors

```
    (x=2, y=3)
      *
     /|
    / |
   /  | y=3
  /   |
 /____|
   x=2

Magnitude (length) = sqrt(2² + 3²) = sqrt(13) ≈ 3.6
```

### Types of Vectors

**Position Vector:** A point in space
```cpp
Vec3 headsetPos = {1.5f, 1.8f, 0.5f};  // Headset location
```

**Direction Vector:** Which way something is pointing (usually normalized)
```cpp
Vec3 forward = {0.0f, 0.0f, -1.0f};    // Looking forward
```

**Velocity Vector:** Speed and direction of movement
```cpp
Vec3 velocity = {0.1f, 0.0f, 0.0f};    // Moving right at 0.1 units/frame
```

---

<a name="vector-operations"></a>
## 4. Vector Operations

### Vector Addition (+)

**Use case:** Applying offsets, combining movements

```
v1 = (1, 2, 3)
v2 = (4, 5, 6)
v1 + v2 = (1+4, 2+5, 3+6) = (5, 7, 9)
```

**Basic Example:**
```cpp
Vec3 position = {5.0f, 2.0f, 0.0f};
Vec3 offset = {0.1f, 0.0f, 0.0f};
Vec3 newPosition = position + offset;  // Move 0.1 units to the right
```

**Real-World OptiTrack/VR Applications:**

1. **Applying motion/velocity to position** (Every frame in VR)
   ```cpp
   // Update headset position based on velocity
   Vec3 currentPos = headset.getPosition();
   Vec3 velocity = {0.01f, 0.0f, 0.0f};  // Moving right
   Vec3 newPos = currentPos + velocity;
   headset.setPosition(newPos);
   ```

2. **Offset calculations for mounted devices**
   ```cpp
   // Camera mounted 2 meters above the tracking origin
   Vec3 trackingOrigin = {0.0f, 0.0f, 0.0f};
   Vec3 cameraOffset = {0.0f, 2.0f, 0.0f};
   Vec3 cameraPosition = trackingOrigin + cameraOffset;
   ```

3. **Combining multiple movements**
   ```cpp
   // Patient wearing headset walks while head moves
   Vec3 bodyMovement = {0.5f, 0.0f, 0.2f};   // Walking
   Vec3 headMovement = {0.0f, 0.05f, 0.0f};  // Slight head bob
   Vec3 totalMovement = bodyMovement + headMovement;
   Vec3 newHeadsetPos = currentPos + totalMovement;
   ```

4. **Marker position relative to rigid body center**
   ```cpp
   // VR controller has 4 markers. Calculate world position of each marker
   Vec3 controllerCenter = {1.0f, 1.2f, 0.5f};
   Vec3 marker1LocalOffset = {0.05f, 0.02f, 0.0f};
   Vec3 marker1WorldPos = controllerCenter + marker1LocalOffset;
   ```

**Formula:**
```cpp
Vec3 operator+(const Vec3& other) const {
    return {x + other.x, y + other.y, z + other.z};
}
```

### Vector Subtraction (-)

**Use case:** Finding direction between two points

```
v1 = (5, 3, 2)
v2 = (2, 1, 1)
v1 - v2 = (3, 2, 1)  // Direction from v2 to v1
```

**Basic Example:**
```cpp
Vec3 headPos = {2.0f, 1.5f, 0.0f};
Vec3 controllerPos = {1.0f, 1.2f, 0.5f};
Vec3 direction = headPos - controllerPos;  // Vector pointing from controller to head
```

**Real-World OptiTrack/VR Applications:**

1. **Calculate direction to look at a target**
   ```cpp
   // VR headset needs to face a surgical target
   Vec3 headsetPos = {1.0f, 1.8f, 2.0f};
   Vec3 targetPos = {2.5f, 1.5f, 0.0f};
   Vec3 directionToTarget = (targetPos - headsetPos).normalize();
   // Now headset knows which way to rotate to face target
   ```

2. **Distance calculation between tracked objects**
   ```cpp
   // How far apart are the left and right controllers?
   Vec3 leftController = {-0.3f, 1.2f, 0.5f};
   Vec3 rightController = {0.3f, 1.2f, 0.5f};
   Vec3 diff = rightController - leftController;
   float distance = diff.magnitude();  // 0.6 meters apart
   ```

3. **Velocity calculation** (Critical for medical tracking precision)
   ```cpp
   // Track how fast surgeon's hand is moving
   Vec3 currentPos = tool.getPosition();
   Vec3 previousPos = tool.getPreviousPosition();
   float deltaTime = 1.0f / 120.0f;  // 120 FPS tracking
   Vec3 velocity = (currentPos - previousPos) / deltaTime;
   float speed = velocity.magnitude();

   if (speed > 0.5f) {
       // Warning: Hand moving too fast for precision surgery
   }
   ```

4. **Detecting collision/proximity**
   ```cpp
   // Is the surgical tool getting too close to a restricted zone?
   Vec3 toolTip = {1.2f, 0.8f, 0.3f};
   Vec3 dangerZone = {1.5f, 0.8f, 0.3f};
   Vec3 separation = toolTip - dangerZone;
   float distance = separation.magnitude();

   if (distance < 0.1f) {  // Less than 10cm
       // Alert: Too close to danger zone!
   }
   ```

5. **Relative positioning for UI elements**
   ```cpp
   // Place a virtual menu 0.5m in front of headset
   Vec3 headsetPos = headset.getPosition();
   Vec3 forward = headset.forward();
   Vec3 menuPos = headsetPos + (forward * 0.5f);
   // But first we needed: forward = rotate({0,0,-1}) using headset rotation
   ```

**Formula:**
```cpp
Vec3 operator-(const Vec3& other) const {
    return {x - other.x, y - other.y, z - other.z};
}
```

### Scalar Multiplication (*)

**Use case:** Scaling vectors (change magnitude, keep direction)

```
v = (2, 3, 1)
v * 2 = (4, 6, 2)  // Twice as long, same direction
```

**Basic Example:**
```cpp
Vec3 velocity = {1.0f, 0.0f, 0.0f};
Vec3 fastVelocity = velocity * 3.0f;  // 3x speed
```

**Real-World OptiTrack/VR Applications:**

1. **Moving along a direction at specific speed**
   ```cpp
   // Move headset 0.5 meters forward
   Vec3 forward = headset.forward();  // Normalized direction
   Vec3 movement = forward * 0.5f;    // Scale to desired distance
   Vec3 newPos = headset.getPosition() + movement;
   ```

2. **Adjusting velocity/speed**
   ```cpp
   // Slow down a fast-moving tracked object for smoother visualization
   Vec3 rawVelocity = calculateVelocity();
   float smoothingFactor = 0.7f;  // 70% of original speed
   Vec3 smoothedVelocity = rawVelocity * smoothingFactor;
   ```

3. **Interpolation (lerp) between positions**
   ```cpp
   // Smooth camera movement from startPos to endPos
   Vec3 startPos = {0.0f, 1.0f, 0.0f};
   Vec3 endPos = {5.0f, 1.0f, 0.0f};
   float t = 0.3f;  // 30% of the way
   Vec3 currentPos = startPos + ((endPos - startPos) * t);
   // Result: (1.5, 1.0, 0.0) - 30% from start to end
   ```

4. **Applying forces/accelerations**
   ```cpp
   // Simulate gravity on a virtual object in VR
   Vec3 gravity = {0.0f, -9.8f, 0.0f};  // m/s²
   float deltaTime = 1.0f / 120.0f;     // Frame time
   Vec3 velocityChange = gravity * deltaTime;
   velocity = velocity + velocityChange;
   ```

5. **Offset positioning at variable distances**
   ```cpp
   // Place UI elements at user-preferred distance
   Vec3 forward = headset.forward();
   float userPreferredDistance = 0.6f;  // 60cm from face
   Vec3 uiPosition = headset.getPosition() + (forward * userPreferredDistance);
   ```

**Formula:**
```cpp
Vec3 operator*(float scalar) const {
    return {x * scalar, y * scalar, z * scalar};
}
```

### Magnitude (Length)

**Use case:** How far? How fast? Distance from origin.

```
v = (3, 4, 0)
|v| = sqrt(3² + 4² + 0²) = sqrt(9 + 16) = sqrt(25) = 5
```

**Example:**
```cpp
Vec3 velocity = {3.0f, 4.0f, 0.0f};
float speed = velocity.magnitude();  // = 5.0
```

**Formula:**
```cpp
float magnitude() const {
    return sqrt(x*x + y*y + z*z);
}
```

### Normalization (Unit Vector)

**Use case:** Direction without magnitude (length = 1)

```
v = (3, 4, 0)
|v| = 5
normalized = (3/5, 4/5, 0/5) = (0.6, 0.8, 0)
|normalized| = 1
```

**Example:**
```cpp
Vec3 direction = {3.0f, 4.0f, 0.0f};
Vec3 unitDirection = direction.normalize();  // (0.6, 0.8, 0) - same direction, length 1
```

**Formula:**
```cpp
Vec3 normalize() const {
    float mag = magnitude();
    if (mag == 0) return {0, 0, 0};  // Avoid division by zero
    return {x / mag, y / mag, z / mag};
}
```

**Why normalize?**
- Rotations require unit vectors
- Comparing directions (ignoring magnitude)
- Consistent movement speed

### Dot Product (·)

**Use case:** Angle between vectors, projection, checking if perpendicular

```
v1 = (1, 0, 0)
v2 = (0, 1, 0)
v1 · v2 = (1×0) + (0×1) + (0×0) = 0  (perpendicular!)
```

**Formula:**
```cpp
float dot(const Vec3& other) const {
    return x*other.x + y*other.y + z*other.z;
}
```

**Relationship to angle:**
```
v1 · v2 = |v1| × |v2| × cos(θ)

For unit vectors:
v1 · v2 = cos(θ)
```

**Interpretation:**
- `dot = 1`: Same direction (0° angle)
- `dot = 0`: Perpendicular (90° angle)
- `dot = -1`: Opposite direction (180° angle)
- `dot > 0`: Less than 90° apart
- `dot < 0`: More than 90° apart

**Example:**
```cpp
Vec3 forward = {0, 0, -1};
Vec3 toTarget = {1, 0, -1};
float alignment = forward.normalize().dot(toTarget.normalize());
// If alignment > 0.7, target is in front of you
```

### Cross Product (×)

**Use case:** Find perpendicular vector, calculate rotation axis

```
v1 = (1, 0, 0)  // X-axis
v2 = (0, 1, 0)  // Y-axis
v1 × v2 = (0, 0, 1)  // Z-axis (perpendicular to both)
```

**Formula:**
```cpp
Vec3 cross(const Vec3& other) const {
    return {
        y * other.z - z * other.y,  // x component
        z * other.x - x * other.z,  // y component
        x * other.y - y * other.x   // z component
    };
}
```

**Properties:**
- **Order matters:** `v1 × v2 = -(v2 × v1)`
- **Right-hand rule:** Curl fingers from v1 to v2, thumb points to result
- **Result is perpendicular** to both input vectors
- **Magnitude = |v1| × |v2| × sin(θ)** (area of parallelogram)

**Example:**
```cpp
Vec3 up = {0, 1, 0};
Vec3 forward = {0, 0, -1};
Vec3 right = up.cross(forward);  // (1, 0, 0)
```

---

<a name="quaternions"></a>
## 5. Quaternions - The Right Way to Rotate

### The Problem with Euler Angles

**Euler angles:** Rotation as three angles (pitch, yaw, roll)

```cpp
struct EulerAngles {
    float pitch;  // Rotation around X (nodding)
    float yaw;    // Rotation around Y (shaking head "no")
    float roll;   // Rotation around Z (tilting head)
};
```

**Problems:**
1. **Gimbal Lock:** At certain angles, you lose a degree of freedom
   - Pitch to 90° → roll and yaw become the same axis!
2. **Interpolation:** Can't smoothly blend between rotations
3. **Order dependent:** XYZ rotation ≠ ZYX rotation

**Why VR cares:** When you look up (pitch 90°), the headset can't distinguish roll from yaw = BAD tracking!

### What is a Quaternion?

A quaternion is a **4D number** that represents rotation:

```cpp
struct Quaternion {
    float x, y, z;  // Vector part (rotation axis)
    float w;        // Scalar part (rotation amount)
};
```

**Think of it as:** "Rotate by angle θ around axis (x, y, z)"

### Axis-Angle Representation

**Most intuitive way to think about rotation:**
- **Axis:** Direction vector (normalized)
- **Angle:** How much to rotate around that axis

**Example:**
```
Rotate 90° around Y-axis:
  axis = (0, 1, 0)
  angle = π/2 (90 degrees in radians)
```

**Converting to Quaternion:**
```cpp
Quaternion(Vec3 axis, float angleRadians) {
    Vec3 normalizedAxis = axis.normalize();
    float halfAngle = angleRadians / 2.0f;
    float sinHalf = sin(halfAngle);

    x = normalizedAxis.x * sinHalf;
    y = normalizedAxis.y * sinHalf;
    z = normalizedAxis.z * sinHalf;
    w = cos(halfAngle);
}
```

**Why half angle?** Math reasons (quaternion double-cover property) - just remember to use `angle/2`.

### Quaternion Multiplication (Combining Rotations)

**Use case:** Apply multiple rotations

```cpp
Quaternion rotate90Y(Vec3{0,1,0}, M_PI/2);   // Rotate 90° around Y
Quaternion rotate45Z(Vec3{0,0,1}, M_PI/4);   // Then rotate 45° around Z
Quaternion combined = rotate45Z * rotate90Y; // Combined rotation
```

**Important:** **Order matters!** `q1 * q2 ≠ q2 * q1`

**Formula:** (You don't need to memorize, just implement from TODO)
```cpp
Quaternion operator*(const Quaternion& q) const {
    Quaternion result;
    result.w = w*q.w - x*q.x - y*q.y - z*q.z;
    result.x = w*q.x + x*q.w + y*q.z - z*q.y;
    result.y = w*q.y - x*q.z + y*q.w + z*q.x;
    result.z = w*q.z + x*q.y - y*q.x + z*q.w;
    return result;
}
```

### Rotating a Vector with a Quaternion

**Use case:** Apply headset rotation to a forward vector

```cpp
Vec3 localForward = {0, 0, -1};
Quaternion headsetRotation = ...;
Vec3 worldForward = headsetRotation.rotate(localForward);
```

**Simplified formula:** (Efficient version)
```cpp
Vec3 rotate(const Vec3& v) const {
    Vec3 qvec = {x, y, z};
    Vec3 uv = qvec.cross(v);
    Vec3 uuv = qvec.cross(uv);
    return v + (uv * (2.0f * w)) + (uuv * 2.0f);
}
```

### Identity Quaternion (No Rotation)

```cpp
Quaternion identity = {0, 0, 0, 1};  // No rotation
```

### Why Quaternions are Better

| Feature | Euler Angles | Quaternions |
|---------|-------------|-------------|
| Gimbal lock | ✗ Yes | ✓ No |
| Smooth interpolation | ✗ Hard | ✓ Easy (SLERP) |
| Combining rotations | ✗ Complex | ✓ Simple (multiply) |
| Memory | 3 floats | 4 floats |
| Human-readable | ✓ Yes | ✗ No |

**For VR/OptiTrack:** Always use quaternions for internal representation.

---

<a name="transforms"></a>
## 6. Transforms - Position + Rotation

### What is a Transform?

A **transform** describes an object's state in 3D space:

```cpp
class Transform {
    Vec3 position;        // Where is it?
    Quaternion rotation;  // Which way is it facing?
};
```

(Sometimes also includes `scale`, but OptiTrack rigid bodies don't scale)

### Local to World Transformation

**Problem:** You have a marker's position relative to a headset. Where is it in the room?

```cpp
Vec3 transformPoint(const Vec3& localPoint) const {
    // 1. Rotate the local point by the object's rotation
    Vec3 rotated = rotation.rotate(localPoint);

    // 2. Add the object's position
    return rotated + position;
}
```

**Example:**
```
Headset Transform:
  position = (2, 1.5, 3)
  rotation = 90° around Y

Local marker position: (0.1, 0, 0)  // 0.1m to the right of headset center

World position = rotate(0.1, 0, 0) + (2, 1.5, 3)
               = (0, 0, -0.1) + (2, 1.5, 3)    // After 90° Y rotation, right becomes -forward
               = (2, 1.5, 2.9)
```

### Direction Vectors

Objects have **local** direction vectors:

```cpp
Vec3 localForward = {0, 0, -1};  // -Z is forward (OpenGL convention)
Vec3 localUp = {0, 1, 0};        // +Y is up
Vec3 localRight = {1, 0, 0};     // +X is right
```

**Getting world-space directions:**
```cpp
Vec3 forward() const {
    return rotation.rotate({0, 0, -1});
}

Vec3 up() const {
    return rotation.rotate({0, 1, 0});
}

Vec3 right() const {
    return rotation.rotate({1, 0, 0});
}
```

**Use case:**
```cpp
Transform headset = ...;
Vec3 lookDirection = headset.forward();  // Which way is the user looking?

// Move 0.5 units in the direction the headset is facing
Vec3 newPos = headset.getPosition() + (lookDirection * 0.5f);
```

---

<a name="matrices"></a>
## 6.5. Matrices - Combining Transformations

### Why Matrices Matter

While Vec3 + Quaternion works conceptually, **real systems use matrices** because:
- GPU optimized for matrix operations
- Single operation combines rotation + translation
- Essential for camera calibration
- Standard in OptiTrack SDK

### 3x3 Matrix - Rotation Only

```cpp
float mat[3][3] = {
    {r00, r01, r02},  // First row
    {r10, r11, r12},  // Second row
    {r20, r21, r22}   // Third row
};
```

**Use cases:**
- Rotation transformations
- Camera intrinsic parameters
- Coordinate system changes

**Rotation around Y-axis (90°):**
```cpp
Mat3 rotY = {
    { 0, 0, 1},  // cos(90°)=0, sin(90°)=1
    { 0, 1, 0},
    {-1, 0, 0}
};

Vec3 point = {1, 0, 0};
Vec3 rotated = rotY * point;  // Result: (0, 0, -1)
```

### 4x4 Matrix - THE Transform Matrix

**This is what OptiTrack SDK actually uses!**

```cpp
float transform[4][4] = {
    {r00, r01, r02, tx},  // Rotation + X translation
    {r10, r11, r12, ty},  // Rotation + Y translation
    {r20, r21, r22, tz},  // Rotation + Z translation
    {  0,   0,   0,  1}   // Homogeneous coordinate
};
```

**Structure breakdown:**
- **Upper-left 3x3**: Rotation (same as Mat3)
- **Right column (tx, ty, tz)**: Translation
- **Bottom row**: Always [0, 0, 0, 1] for standard transforms

### Homogeneous Coordinates

**To use 4x4 matrices, vectors become 4D:**

```cpp
// Position (point in space)
Vec4 position = {x, y, z, 1};  // w=1

// Direction (no position, just direction)
Vec4 direction = {x, y, z, 0};  // w=0
```

**Why w=1 vs w=0?**
- `w=1`: Affected by translation (it's a point)
- `w=0`: NOT affected by translation (it's a direction)

**Example:**
```cpp
Mat4 transform = /* rotation + translation (5, 2, 0) */;

Vec4 point = {1, 0, 0, 1};      // w=1
Vec4 transformedPoint = transform * point;  // Gets rotated AND translated

Vec4 direction = {1, 0, 0, 0};  // w=0
Vec4 transformedDir = transform * direction;  // Only rotated, NO translation
```

### Matrix Multiplication - Combining Transforms

**Key insight:** Multiply matrices to combine transformations

```cpp
// Headset transform
Mat4 headsetTransform = ...;

// Marker offset (local to headset)
Mat4 markerOffset = Mat4::translation({0.1, 0.05, 0});

// Combined: marker position in world space
Mat4 markerWorld = headsetTransform * markerOffset;

// Transform a point
Vec3 localPoint = {0, 0, 0};
Vec3 worldPoint = markerWorld.transformPoint(localPoint);
```

**Order matters!** `A * B ≠ B * A`

```cpp
// Rotate THEN translate
Mat4 result1 = translation * rotation;

// Translate THEN rotate (different result!)
Mat4 result2 = rotation * translation;
```

### Real OptiTrack SDK Example

```cpp
// From NatNet SDK
sRigidBodyData* rigidBody;

// OptiTrack provides a 4x4 matrix (flattened to 1D array)
float transform[16];  // Actually 4x4, stored as 1D
rigidBody->GetTransformMatrix(transform);

// Use it to transform markers
for (int i = 0; i < numMarkers; i++) {
    Vec3 markerLocal = markerPositions[i];
    Vec3 markerWorld = transformPoint(transform, markerLocal);
}
```

### When to Use What?

| Representation | Storage | Use Case |
|---------------|---------|----------|
| Vec3 + Quaternion | 7 floats | Conceptual understanding, storage |
| 3x3 Matrix | 9 floats | Rotation only, camera calibration |
| 4x4 Matrix | 16 floats | GPU operations, combining transforms |

**In practice:**
- **Store:** Quaternion (compact)
- **Compute:** 4x4 Matrix (fast)
- **Convert:** Between them as needed

### Common Matrix Operations

**Identity matrix (no transformation):**
```cpp
Mat4 identity = {
    {1, 0, 0, 0},
    {0, 1, 0, 0},
    {0, 0, 1, 0},
    {0, 0, 0, 1}
};
```

**Translation matrix:**
```cpp
Mat4 translation(Vec3 t) {
    return {
        {1, 0, 0, t.x},
        {0, 1, 0, t.y},
        {0, 0, 1, t.z},
        {0, 0, 0,   1}
    };
}
```

**Rotation matrix around Y-axis:**
```cpp
Mat4 rotationY(float angle) {
    float c = cos(angle);
    float s = sin(angle);
    return {
        { c, 0, s, 0},
        { 0, 1, 0, 0},
        {-s, 0, c, 0},
        { 0, 0, 0, 1}
    };
}
```

### Practical Applications

**1. Camera Calibration**
```cpp
// Intrinsic matrix (3x3) - maps 3D → 2D
Mat3 K = {
    {fx,  0, cx},  // Focal length & principal point
    { 0, fy, cy},
    { 0,  0,  1}
};

// Project 3D point to 2D image
Vec3 point3D = {x, y, z};
Vec3 projected = K * (point3D / point3D.z);  // Perspective division
Vec2 pixel = {projected.x, projected.y};
```

**2. Multi-camera setup**
```cpp
// Each OptiTrack camera has position + rotation
Mat4 camera1Transform = ...;
Mat4 camera2Transform = ...;

// Convert point from camera1 space to camera2 space
Mat4 camera1ToWorld = camera1Transform;
Mat4 worldToCamera2 = inverse(camera2Transform);
Mat4 camera1ToCamera2 = worldToCamera2 * camera1ToWorld;
```

**3. Hierarchical transforms (parent-child)**
```cpp
// Headset transform
Mat4 headsetTransform = ...;

// Eye offset (children of headset)
Mat4 leftEyeOffset = Mat4::translation({-0.03, 0, 0});
Mat4 rightEyeOffset = Mat4::translation({0.03, 0, 0});

// World positions of eyes
Mat4 leftEyeWorld = headsetTransform * leftEyeOffset;
Mat4 rightEyeWorld = headsetTransform * rightEyeOffset;
```

---

<a name="real-world-applications"></a>
## 7. Real-World Applications

### OptiTrack Tracking Pipeline

```
1. Cameras detect marker positions (2D in each camera)
   ↓
2. Triangulation → 3D marker positions in world space
   ↓
3. Marker clustering → Identify which markers belong to which rigid body
   ↓
4. Rigid body pose estimation → Calculate position + rotation (quaternion)
   ↓
5. Send to VR application → Update headset/controller transforms
```

### Common Calculations

**Distance between headset and controller:**
```cpp
Vec3 headPos = headset.getPosition();
Vec3 controllerPos = controller.getPosition();
Vec3 diff = headPos - controllerPos;
float distance = diff.magnitude();
```

**Is target in front of headset?**
```cpp
Vec3 toTarget = targetPos - headsetPos;
Vec3 forward = headset.forward();
float alignment = toTarget.normalize().dot(forward);
if (alignment > 0.7) {  // cos(45°) ≈ 0.7
    // Target is in front
}
```

**Calculate velocity:**
```cpp
Vec3 currentPos = headset.getPosition();
Vec3 previousPos = ...;
float deltaTime = 1.0f / 120.0f;  // 120 FPS
Vec3 velocity = (currentPos - previousPos) / deltaTime;
```

**Smooth rotation interpolation (SLERP):**
```cpp
// Blend between two rotations smoothly
Quaternion slerp(Quaternion q1, Quaternion q2, float t) {
    // t = 0 → q1, t = 1 → q2, t = 0.5 → halfway
    // (Implementation complex, but concept simple)
}
```

---

## 7.5. Understanding the Tests - Practical Walkthrough

This section explains what each test in `session3_motion_tracking.cpp` does **practically** to help you gain real-world understanding.

### Test 1: Vector Operations - Building Blocks

**What it tests:** All fundamental vector operations you'll use in motion tracking.

#### Test 1a: Vector Addition
```cpp
Vec3 v1 = {1.0f, 2.0f, 3.0f};
Vec3 v2 = {4.0f, 5.0f, 6.0f};
Vec3 sum = v1 + v2;
// Expected: (5, 7, 9)
```

**Practical meaning:**
```
v1 = Current headset position
v2 = Movement offset (velocity * time)
sum = New headset position after movement

Real OptiTrack scenario:
- Headset at (1, 2, 3)
- User moves (+4, +5, +6) in one frame
- New position: (5, 7, 9)
```

**Visual:**
```
    v2 (4,5,6)
      /
     /
    *-----v1+v2 (5,7,9)
   /
  /
 * v1 (1,2,3)
```

#### Test 1b: Magnitude
```cpp
Vec3 v = {3.0f, 4.0f, 0.0f};
float mag = v.magnitude();
// Expected: 5.0
```

**Practical meaning:**
```
3-4-5 right triangle!
  *
  |\
4 | \  5 (magnitude)
  |  \
  *---*
    3

Real scenario:
- Velocity vector is (3, 4, 0) m/s
- Magnitude = 5.0 m/s = actual speed
- Used to check: "Is surgeon's hand moving too fast?"
```

#### Test 1c: Normalization
```cpp
Vec3 v = {3.0f, 4.0f, 0.0f};
Vec3 norm = v.normalize();
// Expected: (0.6, 0.8, 0.0)
```

**Practical meaning:**
```
Take direction, remove magnitude:
v = (3, 4, 0), magnitude = 5
normalized = (3/5, 4/5, 0/5) = (0.6, 0.8, 0)

New magnitude = sqrt(0.6² + 0.8²) = 1.0

Real scenario:
- You know velocity is (3, 4, 0)
- You only care about DIRECTION user is moving
- Normalize to get pure direction: (0.6, 0.8, 0)
- Now you can say "move 2 meters in that direction": direction * 2.0
```

#### Test 1d: Dot Product
```cpp
Vec3 v1 = {1.0f, 0.0f, 0.0f};  // X-axis (right)
Vec3 v2 = {0.0f, 1.0f, 0.0f};  // Y-axis (up)
float dot = v1.dot(v2);
// Expected: 0.0 (perpendicular!)
```

**Practical meaning:**
```
X-axis (right) and Y-axis (up) are perpendicular.
Dot product = 0 confirms they're 90° apart.

Visual (top view):
     Y (up)
     |
     |
     +------ X (right)

dot(X, Y) = 0 → perpendicular

Real scenario - "Is target in front of me?":
Vec3 forward = headset.forward();
Vec3 toTarget = (target - headset.position).normalize();
float dot = forward.dot(toTarget);

if (dot > 0.7)  → target is in front (< 45° away)
if (dot ≈ 0)    → target is to the side (≈ 90° away)
if (dot < 0)    → target is behind (> 90° away)
```

#### Test 1e: Cross Product
```cpp
Vec3 v1 = {1.0f, 0.0f, 0.0f};  // X-axis
Vec3 v2 = {0.0f, 1.0f, 0.0f};  // Y-axis
Vec3 cross = v1.cross(v2);
// Expected: (0, 0, 1) - Z-axis!
```

**Practical meaning:**
```
Cross product finds perpendicular vector.
X × Y = Z (right-hand rule)

Visual:
      Y (up)
      |
      |
      +------ X (right)
     /
    /
   Z (result - forward)

Right-hand rule:
1. Point fingers along X (right)
2. Curl them toward Y (up)
3. Thumb points to Z (forward/out of screen)

Real scenario - "Find camera's right vector":
Vec3 worldUp = {0, 1, 0};
Vec3 cameraForward = camera.forward();
Vec3 cameraRight = worldUp.cross(cameraForward);
// Now you have all 3 axes of camera!
```

### Test 2: Quaternion Rotation - The Core of Tracking

**What it tests:** Rotating a vector using a quaternion (what OptiTrack does every frame).

```cpp
// Rotate 90° around Y-axis
Quaternion rot(Vec3{0.0f, 1.0f, 0.0f}, M_PI/2);
Vec3 point = {1.0f, 0.0f, 0.0f};
Vec3 rotated = rot.rotate(point);
// Expected: (0, 0, -1)
```

**Practical meaning:**
```
Top view (looking down Y-axis):

Before rotation:           After 90° rotation:
       Z                          Z
       |                          |
       |                          |
       +------ X                  +------ X
      point (1,0,0)            rotated (0,0,-1)

Point at (1, 0, 0) = "to the right"
Rotate 90° around Y = "turn left"
Result: (0, 0, -1) = "now pointing forward"
```

**Step-by-step visualization:**
```
1. Start: point = (1, 0, 0)

   Top view:
        ↑ Z
        |
   -----+----→ X
        |  *point
        |

2. Rotate 90° around Y (counterclockwise from above)

3. End: rotated = (0, 0, -1)

   Top view:
        ↑ Z
        *rotated
   -----+----→ X
        |
        |
```

**Real OptiTrack scenario:**
```cpp
// Headset is rotated 90° to the left (around Y)
Quaternion headsetRotation = ...;  // 90° around Y

// In the headset's local space, "forward" is (0, 0, -1)
Vec3 localForward = {0.0f, 0.0f, -1.0f};

// What direction is the user actually looking in world space?
Vec3 worldForward = headsetRotation.rotate(localForward);

// If headset is rotated 90° left, user is looking along +X axis
```

**Why this matters:**
- OptiTrack gives you headset rotation as quaternion
- You need to know which direction user is looking
- You rotate the local "forward" direction to get world direction
- This happens 120+ times per second in VR!

### Test 3: Transform Operations - Putting It All Together

**What it tests:** How OptiTrack represents rigid bodies (position + rotation).

#### Test 3a: transformPoint - Local to World
```cpp
Transform headset(Vec3{2.0f, 1.5f, 3.0f}, Quaternion(Vec3{0,1,0}, M_PI/2));
Vec3 localMarker = {0.1f, 0.0f, 0.0f};
Vec3 worldMarker = headset.transformPoint(localMarker);
// Expected: (2.0, 1.5, 2.9)
```

**Practical meaning:**
```
Complete OptiTrack scenario:

1. Headset is in the room at position (2, 1.5, 3)
2. Headset is rotated 90° around Y-axis (facing left)
3. Marker is attached 0.1m to the RIGHT of headset center (local coordinates)
4. Where is the marker in world space?

Step-by-step:
┌─────────────────────────────────────────────────┐
│ STEP 1: Local marker position                  │
│         (0.1, 0, 0) = "0.1m to the right"      │
└─────────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────────┐
│ STEP 2: Rotate by headset's rotation           │
│         90° around Y turns "right" into "back"  │
│         (0.1, 0, 0) → (0, 0, -0.1)             │
└─────────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────────┐
│ STEP 3: Add headset's position                 │
│         (0, 0, -0.1) + (2, 1.5, 3)             │
│         = (2.0, 1.5, 2.9)                      │
└─────────────────────────────────────────────────┘

Visual (top view):
       Z
       |
   [H] + headset at (2, 1.5, 3), facing left (90° rotated)
       |
       +------ X
       |
       |
     marker at (2.0, 1.5, 2.9) - behind headset in world space
```

**Why this matters:**
```
OptiTrack tracks rigid bodies by their markers.
- Each rigid body has a center position
- Each marker has a FIXED offset from center (in local space)
- To render markers or check tracking quality, you need world positions
- This transform is how you get them!

Real code flow:
for (int i = 0; i < rigidBody->nMarkers; i++) {
    Vec3 localOffset = rigidBody->markerLocalPositions[i];
    Vec3 worldPos = rigidBody.transformPoint(localOffset);
    renderMarker(worldPos);
}
```

#### Test 3b: forward() - Which Way Is It Facing?
```cpp
Transform headset(Vec3{0,0,0}, Quaternion(Vec3{0,1,0}, M_PI/2));
Vec3 forwardDir = headset.forward();
// Expected: (1, 0, 0) - pointing right in world space!
```

**Practical meaning:**
```
Headset rotated 90° around Y-axis.
Default forward is (0, 0, -1).
After 90° rotation, forward becomes (1, 0, 0).

Visual (top view):
       Z
       |
Default forward      After 90° rotation:
       ↑
       |                  [H]----→ now forward is RIGHT
   [H] |
       |
       +------ X

Real scenario:
// Place a virtual menu 0.5m in front of user
Vec3 headsetPos = headset.getPosition();
Vec3 lookDir = headset.forward();
Vec3 menuPos = headsetPos + (lookDir * 0.5f);
displayMenu(menuPos);
```

#### Test 3c: up() - Which Way Is Up For This Object?
```cpp
Transform rotatedCamera(Vec3{0,0,0}, Quaternion(Vec3{1,0,0}, M_PI));
Vec3 upDir = rotatedCamera.up();
// Expected: (0, -1, 0) - pointing DOWN!
```

**Practical meaning:**
```
Camera flipped upside-down (180° around X-axis).
Default up is (0, 1, 0).
After 180° flip, up becomes (0, -1, 0).

Visual (side view):
  Normal camera:          Flipped camera:
       Y (up)                  Y
       ↑                       |
       |                       |
   [C]-+--→ Z (forward)    [C]-+--→ Z
                               |
                               ↓ (0,-1,0) "up" is now down!

Real scenario - detecting if camera is mounted upside-down:
Vec3 cameraUp = camera.up();
Vec3 worldUp = {0, 1, 0};
float dot = cameraUp.dot(worldUp);

if (dot < -0.9) {
    // Camera is upside-down! Flip the image.
}
```

### Putting It All Together - Complete OptiTrack Flow

Here's how all these operations work together in a real VR application:

```cpp
// FRAME 1: OptiTrack sends data
sRigidBodyData* headsetData = /* from OptiTrack SDK */;

// 1. Extract position and rotation (quaternion)
Vec3 position = {headsetData->x, headsetData->y, headsetData->z};
Quaternion rotation = {headsetData->qx, headsetData->qy,
                       headsetData->qz, headsetData->qw};

// 2. Create transform
Transform headset(position, rotation);

// 3. Calculate where user is looking (for rendering, interaction)
Vec3 lookDirection = headset.forward();

// 4. Check if user is looking at target
Vec3 toTarget = (targetPos - headset.getPosition()).normalize();
float alignment = lookDirection.dot(toTarget);
if (alignment > 0.9) {  // Looking almost directly at it
    highlightTarget();
}

// 5. Calculate velocity (is user moving?)
Vec3 velocity = (position - previousPosition) / deltaTime;
float speed = velocity.magnitude();
if (speed > 2.0f) {  // Moving fast
    enableMotionBlur();
}

// 6. Transform hand controller position to world space
Vec3 localControllerOffset = {0.3f, -0.2f, 0.4f};
Vec3 worldControllerPos = headset.transformPoint(localControllerOffset);

// 7. Check distance to virtual object
Vec3 toObject = worldControllerPos - virtualObjectPos;
float distance = toObject.magnitude();
if (distance < 0.1f) {  // Within 10cm
    grabObject();
}
```

**Every test you completed is a step in this pipeline!**

---

<a name="interview-questions"></a>
## 8. Common Interview Questions

### Conceptual Questions

**Q: What's the difference between a position and a direction vector?**
- Position: A point in space (origin matters)
- Direction: A direction and magnitude (origin doesn't matter, often normalized)

**Q: Why use quaternions instead of Euler angles?**
- No gimbal lock
- Smooth interpolation
- Easy to combine rotations
- Better for real-time systems

**Q: What does the dot product tell you?**
- How aligned two vectors are
- If result > 0: less than 90° apart
- If result = 0: perpendicular
- If result < 0: more than 90° apart

**Q: What does the cross product give you?**
- A vector perpendicular to both input vectors
- Direction follows right-hand rule
- Used to find rotation axes

**Q: How do you convert from local space to world space?**
1. Rotate the local point by the object's rotation
2. Add the object's position

### Practical Questions

**Q: Given two points, how do you find the direction from A to B?**
```cpp
Vec3 direction = (B - A).normalize();
```

**Q: How do you check if a point is in front of the camera?**
```cpp
Vec3 toPoint = (point - cameraPos).normalize();
Vec3 forward = camera.forward();
float dot = toPoint.dot(forward);
if (dot > 0) { /* in front */ }
```

**Q: How do you rotate a vector 90° around the Y-axis?**
```cpp
Quaternion rot(Vec3{0,1,0}, M_PI/2);
Vec3 rotated = rot.rotate(originalVec);
```

---

## Quick Reference Formulas

```cpp
// Vector Operations
magnitude = sqrt(x² + y² + z²)
normalize = v / magnitude
dot(v1, v2) = v1.x*v2.x + v1.y*v2.y + v1.z*v2.z
cross(v1, v2) = (v1.y*v2.z - v1.z*v2.y,
                 v1.z*v2.x - v1.x*v2.z,
                 v1.x*v2.y - v1.y*v2.x)

// Quaternion from axis-angle
half = angle / 2
q.x = axis.x * sin(half)
q.y = axis.y * sin(half)
q.z = axis.z * sin(half)
q.w = cos(half)

// Transform point
worldPos = rotation.rotate(localPos) + position
```

---

## Key Takeaways

✓ **Vectors** represent positions, directions, velocities in 3D
✓ **Dot product** measures alignment (cos of angle)
✓ **Cross product** finds perpendicular vectors
✓ **Normalize** to get direction without magnitude
✓ **Quaternions** avoid gimbal lock (critical for VR!)
✓ **Transforms** combine position + rotation
✓ **Local → World** transformation: rotate then translate

**You're now equipped to tackle Session 3 exercises!** 🚀
