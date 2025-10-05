/*
 * SESSION 2: OOP Basics for Motion Tracking
 *
 * In OptiTrack/VR, you'll see classes representing:
 * - RigidBody (VR headset, controller with multiple markers)
 * - TrackingCamera
 * - CalibrationData
 *
 * This session covers the OOP essentials you NEED for the interview.
 */

#include <iostream>
#include <cmath>
#include <string>

// From Session 1 - we'll build on this
struct Point3D {
    float x, y, z;
};

// TODO 1: Complete the RigidBody class
// A rigid body is an object tracked by multiple markers (e.g., VR headset)
class RigidBody {
private:
    // Private members - only accessible within this class
    std::string name;
    Point3D position;
    int id;
    bool isTracked;

public:
    // TODO 1a: Constructor - initializes a new RigidBody
    // Syntax: ClassName(parameters) : member1(value1), member2(value2) { }
    RigidBody(std::string bodyName, int bodyId) {
        // YOUR CODE HERE
        // Initialize: name, id, position to {0,0,0}, isTracked to false
        name = bodyName;
        id = bodyId;
        position = {0.0f, 0.0f, 0.0f};
        isTracked = false;
        
        // The more idioatic way:
        // :name(bodyName), id(bodyId), position({0.0f, 0.0f, 0.0f}), isTracked(false) {

    }

    // TODO 1b: Getter method - returns the name
    std::string getName() const {
        return name;
    }

    // TODO 1c: Setter method - updates the position
    void updatePosition(float x, float y, float z) {
        // YOUR CODE HERE
        // Update position.x, position.y, position.z
        // Set isTracked to true (we got new data!)
        position.x = x;
        position.y = y;
        position.z = z;
        isTracked = true;
    }

    // TODO 1d: Method that calculates distance from origin (0,0,0)
    float distanceFromOrigin() const {
        // YOUR CODE HERE
        // Use the distance formula from Session 1
        // Distance from origin = sqrt(x^2 + y^2 + z^2)
        return sqrt(position.x * position.x + position.y * position.y + position.z * position.z);
    }

    // TODO 1e: Print method - displays rigid body info
    void printInfo() const {

        std::cout << "RigidBody " << name << "(ID: " << id << "): Position (" << position.x << ", "<< position.y << ", " << position.z << ") - Tracked: " << (isTracked ? "Yes" : "No") << std::endl;

       // YOUR CODE HERE
        // Print: "RigidBody [name] (ID: id): Position (x, y, z) - Tracked: yes/no"
        // Use isTracked to print "yes" or "no"
    }

    // Getter for position (already complete - example for you)
    Point3D getPosition() const {
        return position;
    }

    // Check if currently tracked
    bool getIsTracked() const {
        return isTracked;
    }
};

// TODO 2: Complete the TrackingCamera class
class TrackingCamera {
private:
    int cameraId;
    Point3D position;
    bool isActive;
    int framesProcessed;

public:
    // TODO 2a: Constructor
    TrackingCamera(int id, float x, float y, float z) {
        // YOUR CODE HERE
        // Initialize cameraId, position, isActive=true, framesProcessed=0
        cameraId = id;
        position = {x,y,z};
        isActive = true;
        framesProcessed = 0;
    }

    // TODO 2b: Method to process a frame (increments counter)
    void processFrame() {
        // YOUR CODE HERE
        // Increment framesProcessed by 1
        // Only if isActive is true
        if (isActive)
        {
            framesProcessed++;
        }
        
    }

    // TODO 2c: Calculate distance to a rigid body
    float distanceTo(const RigidBody& body) const {
        // YOUR CODE HERE
        // Get body's position
        // Calculate distance between camera position and body position
        // Hint: Use the distance formula from Session 1
        float dx = position.x - body.getPosition().x;
        float dy = position.y - body.getPosition().y;
        float dz = position.z - body.getPosition().z;
        return sqrt(dx*dx + dy*dy + dz*dz);
    }

    // TODO 2d: Print camera info
    void printInfo() const {
        // YOUR CODE HERE
        // Print: "Camera ID: [id] at (x, y, z) - Frames: [count] - Active: yes/no"
        std::cout << "Camera ID: " << cameraId << " at (" << position.x << ", " << position.y << ", " << position.z << ") - Frames: " << framesProcessed << " - Active: " << (isActive ? "Yes" : "No") << std::endl;
    }
};

int main() {
    std::cout << "=== SESSION 2: OOP Basics for Motion Tracking ===" << std::endl;
    std::cout << std::endl;

    // Test TODO 1: RigidBody class
    std::cout << "--- Test 1: RigidBody Creation and Methods ---" << std::endl;
    RigidBody headset("VR_Headset", 1);
    headset.printInfo();

    headset.updatePosition(1.5f, 1.8f, 0.5f);
    headset.printInfo();

    std::cout << "Distance from origin: " << headset.distanceFromOrigin() << std::endl;
    std::cout << "Expected: ~2.4" << std::endl;
    std::cout << std::endl;

    // Test TODO 2: TrackingCamera class
    std::cout << "--- Test 2: TrackingCamera ---" << std::endl;
    TrackingCamera camera(1, 0.0f, 3.0f, 0.0f);
    camera.printInfo();

    // Simulate processing frames
    for (int i = 0; i < 120; i++) {  // 120 FPS for 1 second
        camera.processFrame();
    }
    camera.printInfo();

    float distance = camera.distanceTo(headset);
    std::cout << "Distance from camera to headset: " << distance << std::endl;
    std::cout << "Expected: ~1.7" << std::endl;
    std::cout << std::endl;

    // Test multiple rigid bodies
    std::cout << "--- Test 3: Multiple Rigid Bodies ---" << std::endl;
    RigidBody leftController("Left_Controller", 2);
    RigidBody rightController("Right_Controller", 3);

    leftController.updatePosition(-0.3f, 1.2f, 0.4f);
    rightController.updatePosition(0.3f, 1.2f, 0.4f);

    leftController.printInfo();
    rightController.printInfo();

    std::cout << "\n=== END OF SESSION 2 ===" << std::endl;
    std::cout << "\nCompile with:" << std::endl;
    std::cout << "  clang++ -o session2 session2_oop_basics.cpp" << std::endl;
    std::cout << "Run with:" << std::endl;
    std::cout << "  ./session2" << std::endl;

    return 0;
}
