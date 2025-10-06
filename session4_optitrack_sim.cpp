/*
 * SESSION 4: OptiTrack Motion Tracking Simulation
 *
 * This simulates a complete OptiTrack motion capture system:
 * - Rigid bodies with markers
 * - Pose estimation from marker positions
 * - Tracking quality metrics
 * - Frame-by-frame updates
 *
 * This prepares you for working with the real NatNet SDK!
 */

#include <iostream>
#include <cmath>
#include <string>
#include <vector>

const float M_PI = 3.14159265358979323846f;

// ============================================================================
// MATH FOUNDATION (from Session 3)
// ============================================================================

struct Vec3 {
    float x, y, z;

    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& other) const {
        return {x + other.x, y + other.y, z + other.z};
    }

    Vec3 operator-(const Vec3& other) const {
        return {x - other.x, y - other.y, z - other.z};
    }

    Vec3 operator*(float scalar) const {
        return {x * scalar, y * scalar, z * scalar};
    }

    Vec3 operator/(float scalar) const {
        return {x / scalar, y / scalar, z / scalar};
    }

    float magnitude() const {
        return sqrt(x*x + y*y + z*z);
    }

    Vec3 normalize() const {
        float mag = magnitude();
        if (mag == 0) return {0, 0, 0};
        return {x / mag, y / mag, z / mag};
    }

    void print(const std::string& label) const {
        printf("%s: (%.3f, %.3f, %.3f)\n", label.c_str(), x, y, z);
    }
};

struct Quaternion {
    float x, y, z, w;

    Quaternion() : x(0), y(0), z(0), w(1) {}
    Quaternion(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}

    // From axis-angle
    Quaternion(Vec3 axis, float angleRadians) {
        Vec3 normalizedAxis = axis.normalize();
        float half_angle = angleRadians / 2.0f;
        float sin_half = sin(half_angle);
        x = normalizedAxis.x * sin_half;
        y = normalizedAxis.y * sin_half;
        z = normalizedAxis.z * sin_half;
        w = cos(half_angle);
    }

    Vec3 rotate(const Vec3& v) const {
        Vec3 qvec = {x, y, z};
        Vec3 uv = cross(qvec, v);
        Vec3 uuv = cross(qvec, uv);
        return v + (uv * (2.0f * w)) + (uuv * 2.0f);
    }

    static Vec3 cross(const Vec3& a, const Vec3& b) {
        return {
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        };
    }
};

struct Transform {
    Vec3 position;
    Quaternion rotation;

    Transform() : position(), rotation() {}
    Transform(Vec3 pos, Quaternion rot) : position(pos), rotation(rot) {}

    Vec3 transformPoint(const Vec3& localPoint) const {
        return rotation.rotate(localPoint) + position;
    }
};

// ============================================================================
// OPTITRACK SIMULATION
// ============================================================================

const int MAX_MARKERS = 8;

// TODO 1: Implement RigidBody class
class RigidBody {
public:
    int id;
    std::string name;
    Transform transform;

    // Marker configuration (local space)
    int numMarkers;
    Vec3 markerOffsets[MAX_MARKERS];  // Fixed positions relative to rigid body center

    // Tracking state
    bool isTracked;
    int visibleMarkers;
    float meanError;

    // TODO 1a: Constructor
    RigidBody(int bodyId, const std::string& bodyName, int markerCount)
        : id(bodyId), name(bodyName), numMarkers(markerCount),
          isTracked(false), visibleMarkers(0), meanError(0.0f) {
        // Initialize transform to origin, no rotation
        // YOUR CODE HERE
    }

    // TODO 1b: Set marker offsets
    void setMarkerOffset(int markerIndex, Vec3 offset) {
        // YOUR CODE HERE
        // Set markerOffsets[markerIndex] = offset
        // Validate markerIndex is within bounds (0 to numMarkers-1)
    }

    // TODO 1c: Get world position of a marker
    Vec3 getMarkerWorldPosition(int markerIndex) const {
        // YOUR CODE HERE
        // Transform marker's local offset to world space
        // Use: transform.transformPoint(markerOffsets[markerIndex])
    }

    // TODO 1d: Calculate centroid of markers in world space
    Vec3 getMarkerCentroid() const {
        // YOUR CODE HERE
        // 1. Sum all marker world positions
        // 2. Divide by numMarkers
        // Hint: Use getMarkerWorldPosition() for each marker
    }

    void print() const {
        printf("RigidBody '%s' (ID %d):\n", name.c_str(), id);
        transform.position.print("  Position");
        printf("  Tracked: %s\n", isTracked ? "YES" : "NO");
        printf("  Visible markers: %d/%d\n", visibleMarkers, numMarkers);
        printf("  Mean error: %.3f mm\n", meanError);
    }
};

// TODO 2: Implement pose estimation
class PoseEstimator {
public:
    // TODO 2a: Compute centroid (average position)
    static Vec3 computeCentroid(const Vec3 positions[], int count) {
        // YOUR CODE HERE
        // Sum all positions and divide by count
    }

    // TODO 2b: Calculate mean error
    // This measures tracking quality: how far are the actual markers from expected positions?
    static float calculateMeanError(const RigidBody& rb, const Vec3 actualMarkerPositions[]) {
        // YOUR CODE HERE
        // For each marker:
        //   1. Get expected position: rb.getMarkerWorldPosition(i)
        //   2. Get actual position: actualMarkerPositions[i]
        //   3. Calculate distance between them
        //   4. Sum all distances
        // Return: total distance / numMarkers
    }

    // TODO 2c: Simplified pose estimation
    // In real OptiTrack, this uses complex algorithms (SVD/Kabsch)
    // We'll use a simplified version: just find position from centroid
    static void estimatePose(RigidBody& rb, const Vec3 actualMarkerPositions[], int count) {
        // YOUR CODE HERE
        // Simplified approach:
        // 1. Calculate centroid of actual marker positions
        // 2. Set rigid body position to that centroid
        // 3. (For this exercise, we'll keep rotation unchanged - real OptiTrack would calculate it)
        // 4. Set isTracked = true if count >= 3
        // 5. Set visibleMarkers = count
        // 6. Calculate mean error using calculateMeanError()
    }
};

// TODO 3: Implement tracking simulation
class TrackingSystem {
private:
    std::vector<RigidBody> rigidBodies;
    int frameNumber;

public:
    TrackingSystem() : frameNumber(0) {}

    // TODO 3a: Add rigid body to system
    void addRigidBody(const RigidBody& rb) {
        // YOUR CODE HERE
        // Add to rigidBodies vector
    }

    // TODO 3b: Find rigid body by ID
    RigidBody* getRigidBody(int id) {
        // YOUR CODE HERE
        // Search rigidBodies vector for matching id
        // Return pointer to rigid body, or nullptr if not found
    }

    // TODO 3c: Process a frame
    // This simulates receiving marker data from OptiTrack cameras
    void processFrame(int rigidBodyId, const Vec3 markerPositions[], int markerCount) {
        // YOUR CODE HERE
        // 1. Find rigid body by ID
        // 2. If found, estimate its pose using PoseEstimator::estimatePose()
        // 3. Increment frameNumber
    }

    // TODO 3d: Print system status
    void printStatus() const {
        // YOUR CODE HERE
        // Print frame number
        // Print status of each rigid body (use rb.print())
    }

    int getFrameNumber() const { return frameNumber; }
};

// ============================================================================
// MAIN - Simulation
// ============================================================================

int main() {
    std::cout << "=== SESSION 4: OptiTrack Motion Tracking Simulation ===" << std::endl;
    std::cout << std::endl;

    // -------------------------------------------------------------------------
    // Test 1: Create a rigid body (VR Headset)
    // -------------------------------------------------------------------------
    std::cout << "--- Test 1: Creating Headset Rigid Body ---" << std::endl;

    RigidBody headset(1, "VR_Headset", 4);

    // Set marker offsets (fixed positions in local space)
    // Pattern: Square configuration
    headset.setMarkerOffset(0, Vec3( 0.05f,  0.03f,  0.02f));  // Front-right-top
    headset.setMarkerOffset(1, Vec3(-0.05f,  0.03f,  0.02f));  // Front-left-top
    headset.setMarkerOffset(2, Vec3( 0.05f,  0.03f, -0.02f));  // Back-right-top
    headset.setMarkerOffset(3, Vec3(-0.05f,  0.03f, -0.02f));  // Back-left-top

    // Set initial pose (headset at origin, no rotation)
    headset.transform = Transform(Vec3(0, 0, 0), Quaternion());

    std::cout << "Headset created with " << headset.numMarkers << " markers" << std::endl;
    std::cout << "Marker offsets (local space):" << std::endl;
    for (int i = 0; i < headset.numMarkers; i++) {
        printf("  Marker %d: (%.3f, %.3f, %.3f)\n",
               i, headset.markerOffsets[i].x, headset.markerOffsets[i].y, headset.markerOffsets[i].z);
    }
    std::cout << std::endl;

    // -------------------------------------------------------------------------
    // Test 2: Transform markers to world space
    // -------------------------------------------------------------------------
    std::cout << "--- Test 2: Marker World Positions ---" << std::endl;

    // Move headset to (1.5, 1.8, 0.5) and rotate 45° around Y
    headset.transform = Transform(
        Vec3(1.5f, 1.8f, 0.5f),
        Quaternion(Vec3(0, 1, 0), M_PI / 4.0f)  // 45° around Y
    );

    std::cout << "Headset moved to (1.5, 1.8, 0.5) and rotated 45° around Y" << std::endl;
    std::cout << "Marker positions (world space):" << std::endl;
    for (int i = 0; i < headset.numMarkers; i++) {
        Vec3 worldPos = headset.getMarkerWorldPosition(i);
        printf("  Marker %d: (%.3f, %.3f, %.3f)\n", i, worldPos.x, worldPos.y, worldPos.z);
    }

    Vec3 centroid = headset.getMarkerCentroid();
    centroid.print("Marker centroid");
    std::cout << "Expected centroid: (1.5, 1.8, 0.5) - matches headset position" << std::endl;
    std::cout << std::endl;

    // -------------------------------------------------------------------------
    // Test 3: Pose estimation
    // -------------------------------------------------------------------------
    std::cout << "--- Test 3: Pose Estimation ---" << std::endl;

    // Simulate OptiTrack detecting markers
    // (In reality, these come from camera triangulation)
    Vec3 detectedMarkers[4] = {
        Vec3(1.56f, 1.83f, 0.52f),  // Slightly offset from expected
        Vec3(1.44f, 1.83f, 0.51f),
        Vec3(1.57f, 1.83f, 0.48f),
        Vec3(1.43f, 1.83f, 0.49f)
    };

    std::cout << "Detected markers (simulated camera data):" << std::endl;
    for (int i = 0; i < 4; i++) {
        printf("  Marker %d: (%.3f, %.3f, %.3f)\n",
               i, detectedMarkers[i].x, detectedMarkers[i].y, detectedMarkers[i].z);
    }
    std::cout << std::endl;

    // Estimate pose
    RigidBody headsetEstimated(1, "VR_Headset", 4);
    for (int i = 0; i < 4; i++) {
        headsetEstimated.setMarkerOffset(i, headset.markerOffsets[i]);
    }

    PoseEstimator::estimatePose(headsetEstimated, detectedMarkers, 4);

    std::cout << "Estimated pose:" << std::endl;
    headsetEstimated.print();
    std::cout << std::endl;

    // -------------------------------------------------------------------------
    // Test 4: Tracking quality
    // -------------------------------------------------------------------------
    std::cout << "--- Test 4: Tracking Quality ---" << std::endl;

    // Scenario 1: All markers visible, low error
    std::cout << "Scenario 1: Perfect tracking (all markers visible)" << std::endl;
    Vec3 perfectMarkers[4] = {
        headset.getMarkerWorldPosition(0),
        headset.getMarkerWorldPosition(1),
        headset.getMarkerWorldPosition(2),
        headset.getMarkerWorldPosition(3)
    };
    float error1 = PoseEstimator::calculateMeanError(headset, perfectMarkers);
    printf("Mean error: %.3f mm (should be ~0)\n", error1);
    std::cout << std::endl;

    // Scenario 2: Some measurement noise
    std::cout << "Scenario 2: Typical tracking (small noise)" << std::endl;
    Vec3 noisyMarkers[4] = {
        headset.getMarkerWorldPosition(0) + Vec3(0.001f, -0.0005f, 0.0008f),  // 1mm noise
        headset.getMarkerWorldPosition(1) + Vec3(-0.0008f, 0.0012f, -0.0006f),
        headset.getMarkerWorldPosition(2) + Vec3(0.0006f, 0.0009f, 0.0011f),
        headset.getMarkerWorldPosition(3) + Vec3(-0.0011f, -0.0007f, 0.0005f)
    };
    float error2 = PoseEstimator::calculateMeanError(headset, noisyMarkers);
    printf("Mean error: %.3f mm (typical: 0.5-1.0mm)\n", error2);
    std::cout << std::endl;

    // -------------------------------------------------------------------------
    // Test 5: Tracking system
    // -------------------------------------------------------------------------
    std::cout << "--- Test 5: Complete Tracking System ---" << std::endl;

    TrackingSystem system;

    // Add headset
    RigidBody headset2(1, "VR_Headset", 4);
    for (int i = 0; i < 4; i++) {
        headset2.setMarkerOffset(i, headset.markerOffsets[i]);
    }
    system.addRigidBody(headset2);

    // Add left controller
    RigidBody leftController(2, "Left_Controller", 3);
    leftController.setMarkerOffset(0, Vec3( 0.03f,  0.01f,  0.0f));
    leftController.setMarkerOffset(1, Vec3(-0.03f,  0.01f,  0.0f));
    leftController.setMarkerOffset(2, Vec3( 0.0f,  -0.02f,  0.0f));
    system.addRigidBody(leftController);

    std::cout << "Tracking system initialized with 2 rigid bodies" << std::endl;
    std::cout << std::endl;

    // Simulate frame 1
    std::cout << "FRAME 1:" << std::endl;
    Vec3 frame1_headset[4] = {
        Vec3(1.56f, 1.83f, 0.52f),
        Vec3(1.44f, 1.83f, 0.51f),
        Vec3(1.57f, 1.83f, 0.48f),
        Vec3(1.43f, 1.83f, 0.49f)
    };
    system.processFrame(1, frame1_headset, 4);

    Vec3 frame1_controller[3] = {
        Vec3(0.8f, 1.2f, 0.6f),
        Vec3(0.74f, 1.2f, 0.6f),
        Vec3(0.77f, 1.18f, 0.6f)
    };
    system.processFrame(2, frame1_controller, 3);

    system.printStatus();
    std::cout << std::endl;

    // Simulate frame 2 (headset moved)
    std::cout << "FRAME 2 (headset moved):" << std::endl;
    Vec3 frame2_headset[4] = {
        Vec3(1.66f, 1.85f, 0.52f),  // Moved 10cm right, 2cm up
        Vec3(1.54f, 1.85f, 0.51f),
        Vec3(1.67f, 1.85f, 0.48f),
        Vec3(1.53f, 1.85f, 0.49f)
    };
    system.processFrame(1, frame2_headset, 4);
    system.processFrame(2, frame1_controller, 3);  // Controller unchanged

    system.printStatus();
    std::cout << std::endl;

    // -------------------------------------------------------------------------
    // Test 6: Occlusion scenario
    // -------------------------------------------------------------------------
    std::cout << "--- Test 6: Occlusion Handling ---" << std::endl;

    std::cout << "Scenario: Only 2 markers visible (not enough to track)" << std::endl;
    Vec3 occludedMarkers[2] = {
        Vec3(1.56f, 1.83f, 0.52f),
        Vec3(1.44f, 1.83f, 0.51f)
    };

    RigidBody occludedHeadset(1, "VR_Headset", 4);
    for (int i = 0; i < 4; i++) {
        occludedHeadset.setMarkerOffset(i, headset.markerOffsets[i]);
    }
    PoseEstimator::estimatePose(occludedHeadset, occludedMarkers, 2);

    occludedHeadset.print();
    if (!occludedHeadset.isTracked) {
        std::cout << "WARNING: Not tracked - need at least 3 markers!" << std::endl;
    }
    std::cout << std::endl;

    std::cout << "=== END OF SESSION 4 ===" << std::endl;
    std::cout << "\nCompile with:" << std::endl;
    std::cout << "  clang++ -std=c++11 -o session4 session4_optitrack_sim.cpp" << std::endl;
    std::cout << "Run with:" << std::endl;
    std::cout << "  ./session4" << std::endl;

    return 0;
}