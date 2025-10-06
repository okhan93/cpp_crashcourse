/*
 * SESSION 3: Motion Tracking Fundamentals
 *
 * In OptiTrack/VR, you're constantly working with:
 * - 3D positions and orientations
 * - Vector math (distance, dot product, cross product)
 * - Rotations (Euler angles and quaternions)
 * - Coordinate transformations
 *
 * This is CORE knowledge for VR interviews!
 */

#include <iostream>
#include <cmath>
#include <string>

// ============================================================================
// PART 1: Vector Math (Building on Point3D)
// ============================================================================

struct Vec3 {
    float x, y, z;

    // TODO 1a: Vector addition
    // Used for: Moving objects by offsets
    Vec3 operator+(const Vec3& other) const {
        // YOUR CODE HERE
        // Return a new Vec3 with (x+other.x, y+other.y, z+other.z)
        return {x+other.x, y+other.y, z+other.z};
    }

    // TODO 1b: Vector subtraction
    // Used for: Finding direction between two points
    Vec3 operator-(const Vec3& other) const {
        // YOUR CODE HERE
        // Return a new Vec3 with (x-other.x, y-other.y, z-other.z)
        return {x-other.x, y-other.y, z-other.z};
    }

    // TODO 1c: Scalar multiplication
    // Used for: Scaling velocities, normalizing vectors
    Vec3 operator*(float scalar) const {
        // YOUR CODE HERE
        // Return a new Vec3 with (x*scalar, y*scalar, z*scalar)
        return {x*scalar, y*scalar, z*scalar};
    }

    // TODO 1d: Magnitude (length) of vector
    // Used for: Distance calculations, normalization
    float magnitude() const {
        // YOUR CODE HERE
        // Return sqrt(x*x + y*y + z*z)
        return sqrt(x*x+y*y+z*z);
    }

    // TODO 1e: Normalize (make length = 1)
    // Used for: Direction vectors, rotations
    Vec3 normalize() const {
        // YOUR CODE HERE
        // Get magnitude, then return Vec3(x/mag, y/mag, z/mag)
        // Handle mag == 0 case by returning (0,0,0)
        float mag = magnitude();
        if (mag==0){
            return {0,0,0};
        }else {
        return {x/mag,y/mag,z/mag};
        }

    }

    // TODO 1f: Dot product
    // Used for: Calculating angles between vectors, projections
    float dot(const Vec3& other) const {
        // YOUR CODE HERE
        // Return x*other.x + y*other.y + z*other.z
        return x*other.x + y*other.y + z*other.z;
    }

    // TODO 1g: Cross product
    // Used for: Finding perpendicular vectors, calculating rotations
    Vec3 cross(const Vec3& other) const {
        // YOUR CODE HERE
        // Formula:
        // result.x = y*other.z - z*other.y
        // result.y = z*other.x - x*other.z
        // result.z = x*other.y - y*other.x
     return {y*other.z - z*other.y,z*other.x - x*other.z,x*other.y - y*other.x};
        
        



    }

    void print(const std::string& label) const {
        std::cout << label << ": (" << x << ", " << y << ", " << z << ")" << std::endl;
    }
};

// ============================================================================
// PART 2: Quaternions (Rotation Representation)
// ============================================================================

struct Quaternion {
    float x, y, z, w;  // w is the scalar part

    // TODO 2a: Constructor from axis-angle
    // This is how OptiTrack often represents rotations
    Quaternion(Vec3 axis, float angleRadians) {
        // YOUR CODE HERE
        // Normalize the axis first
        // half_angle = angleRadians / 2
        // sin_half = sin(half_angle)
        // x = axis.x * sin_half
        // y = axis.y * sin_half
        // z = axis.z * sin_half
        // w = cos(half_angle)
        Vec3 normalizedAxis = axis.normalize();
        float half_angle = angleRadians/2;
        float sin_half = sin(half_angle);

        x = normalizedAxis.x * sin_half;
        y = normalizedAxis.y * sin_half;
        z = normalizedAxis.z * sin_half;
        this->w = cos(half_angle);

    }

    // Default constructor (no rotation)
    Quaternion() : x(0), y(0), z(0), w(1) {}

    // TODO 2b: Quaternion multiplication (combining rotations)
    // Used for: Applying multiple rotations in sequence
    Quaternion operator*(const Quaternion& q) const {
        // YOUR CODE HERE
        // This is the quaternion multiplication formula
        // Quaternion result;
        // result.w = w*q.w - x*q.x - y*q.y - z*q.z;
        // result.x = w*q.x + x*q.w + y*q.z - z*q.y;
        // result.y = w*q.y - x*q.z + y*q.w + z*q.x;
        // result.z = w*q.z + x*q.y - y*q.x + z*q.w;
        // return result;
        Quaternion result;
        result.w = w*q.w - x*q.x - y*q.y - z*q.z;
        result.x = w*q.x + x*q.w + y*q.z - z*q.y;
        result.y = w*q.y - x*q.z + y*q.w + z*q.x;
        result.z = w*q.z + x*q.y - y*q.x + z*q.w;
        return result;

    }

    // TODO 2c: Rotate a vector by this quaternion
    // Used for: Transforming positions/directions
    Vec3 rotate(const Vec3& v) const {
        // YOUR CODE HERE
        // Simplified formula (more efficient than full quaternion multiplication):
        // 1. Extract vector part of quaternion: qvec = {x, y, z}
        // 2. Calculate: uv = qvec.cross(v)
        // 3. Calculate: uuv = qvec.cross(uv)
        // 4. Return: v + (uv * (2.0f * w)) + (uuv * 2.0f)
        Vec3 qvec = {x,y,z};
        Vec3 uv = qvec.cross(v);
        Vec3 uuv = qvec.cross(uv);

        return v+(uv*(2.0f * w))+(uuv*2.0f);
    }

    void print(const std::string& label) const {
        std::cout << label << ": (" << x << ", " << y << ", " << z << ", " << w << ")" << std::endl;
    }
};

// ============================================================================
// PART 3: Rigid Body Transform
// ============================================================================

class Transform {
private:
    Vec3 position;
    Quaternion rotation;

public:
    Transform() : position({0,0,0}), rotation() {}

    Transform(Vec3 pos, Quaternion rot) : position(pos), rotation(rot) {}

    // TODO 3a: Transform a point from local to world space
    // Used for: Converting marker positions to world coordinates
    Vec3 transformPoint(const Vec3& localPoint) const {
        // YOUR CODE HERE
        // 1. Rotate the local point by the rotation
        // 2. Add the position
        // return rotation.rotate(localPoint) + position;
        return rotation.rotate(localPoint)+position;
    }

    // TODO 3b: Get forward direction vector
    // Used for: Knowing which way a VR headset is facing
    Vec3 forward() const {
        // YOUR CODE HERE
        // Local forward is (0, 0, -1) in OpenGL/OptiTrack convention
        // Rotate it by the current rotation
        Vec3 local_forward = {0,0,-1};
        return rotation.rotate(local_forward);
    }

    // TODO 3c: Get up direction vector
    Vec3 up() const {
        // YOUR CODE HERE
        // Local up is (0, 1, 0)
        // Rotate it by the current rotation
        Vec3 local_up= {0,1,0};
        return rotation.rotate(local_up);
    }

    // TODO 3d: Get right direction vector
    Vec3 right() const {
        // YOUR CODE HERE
        // Local right is (1, 0, 0)
        // Rotate it by the current rotation
        Vec3 local_right = {1,0,0};
        return rotation.rotate(local_right);
    }

    void print(const std::string& label) const {
        position.print(label + " Position");
        rotation.print(label + " Rotation");
    }

    Vec3 getPosition() const { return position; }
    Quaternion getRotation() const { return rotation; }
};

// ============================================================================
// MAIN - Tests
// ============================================================================

int main() {
    std::cout << "=== SESSION 3: Motion Tracking Fundamentals ===" << std::endl;
    std::cout << std::endl;

    // Test 1: Vector operations
    std::cout << "--- Test 1: Vector Math ---" << std::endl;
    Vec3 v1 = {1.0f, 0.0f, 0.0f};
    Vec3 v2 = {0.0f, 1.0f, 0.0f};

    Vec3 sum = v1 + v2;
    sum.print("v1 + v2");
    std::cout << "Expected: (1, 1, 0)" << std::endl;

    Vec3 v3 = {3.0f, 4.0f, 0.0f};
    std::cout << "Magnitude of (3,4,0): " << v3.magnitude() << std::endl;
    std::cout << "Expected: 5" << std::endl;

    Vec3 normalized = v3.normalize();
    normalized.print("Normalized (3,4,0)");
    std::cout << "Expected: (0.6, 0.8, 0)" << std::endl;

    float dotProduct = v1.dot(v2);
    std::cout << "Dot product of (1,0,0) and (0,1,0): " << dotProduct << std::endl;
    std::cout << "Expected: 0 (perpendicular vectors)" << std::endl;

    Vec3 crossProduct = v1.cross(v2);
    crossProduct.print("Cross product of (1,0,0) and (0,1,0)");
    std::cout << "Expected: (0, 0, 1)" << std::endl;
    std::cout << std::endl;

    // Test 2: Quaternions
    std::cout << "--- Test 2: Quaternions ---" << std::endl;
    Vec3 axis = {0.0f, 1.0f, 0.0f};  // Y-axis
    float angle = M_PI / 2.0f;  // 90 degrees
    Quaternion rot(axis, angle);
    rot.print("90° rotation around Y-axis");

    Vec3 point = {1.0f, 0.0f, 0.0f};
    Vec3 rotated = rot.rotate(point);
    rotated.print("Point (1,0,0) rotated 90° around Y");
    std::cout << "Expected: (0, 0, -1)" << std::endl;
    std::cout << std::endl;

    // Test 3: Transform
    std::cout << "--- Test 3: Rigid Body Transform ---" << std::endl;
    Vec3 pos = {5.0f, 2.0f, 0.0f};
    Transform transform(pos, rot);
    transform.print("Headset Transform");

    Vec3 forwardDir = transform.forward();
    forwardDir.print("Forward direction");
    std::cout << "Expected: Shows which way headset is facing" << std::endl;
    std::cout << std::endl;

    std::cout << "=== END OF SESSION 3 ===" << std::endl;
    std::cout << "\nCompile with:" << std::endl;
    std::cout << "  clang++ -o session3 session3_motion_tracking.cpp" << std::endl;
    std::cout << "Run with:" << std::endl;
    std::cout << "  ./session3" << std::endl;

    return 0;
}
