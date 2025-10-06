/*
 * SESSION 3.5: Matrix Math for Transforms
 *
 * Matrices are fundamental in motion tracking:
 * - 3x3 matrices for rotations
 * - 4x4 matrices for combined rotation + translation
 * - Camera calibration matrices
 * - Coordinate system transformations
 *
 * This bridges Vec3/Quaternions with real OptiTrack SDK patterns
 */

#include <iostream>
#include <cmath>
#include <string>

// From Session 3
struct Vec3 {
    float x, y, z;

    Vec3 operator+(const Vec3& other) const {
        return {x + other.x, y + other.y, z + other.z};
    }

    Vec3 operator*(float scalar) const {
        return {x * scalar, y * scalar, z * scalar};
    }

    void print(const std::string& label) const {
        std::cout << label << ": (" << x << ", " << y << ", " << z << ")" << std::endl;
    }
};

// Homogeneous coordinates (for 4x4 matrix operations)
struct Vec4 {
    float x, y, z, w;

    Vec4(float x_, float y_, float z_, float w_ = 1.0f) : x(x_), y(y_), z(z_), w(w_) {}
    Vec4(Vec3 v, float w_ = 1.0f) : x(v.x), y(v.y), z(v.z), w(w_) {}

    Vec3 toVec3() const {
        if (w != 0.0f) {
            return {x / w, y / w, z / w};
        }
        return {x, y, z};
    }

    void print(const std::string& label) const {
        std::cout << label << ": (" << x << ", " << y << ", " << z << ", " << w << ")" << std::endl;
    }
};

// ============================================================================
// PART 1: 3x3 Matrix (Rotation/Scale)
// ============================================================================

class Mat3 {
public:
    float m[3][3];  // Row-major: m[row][col]

    // TODO 1a: Constructor - Initialize to identity matrix
    Mat3() {
        // YOUR CODE HERE
        // Identity matrix:
        // [1, 0, 0]
        // [0, 1, 0]
        // [0, 0, 1]
    }

    // Constructor from values
    Mat3(float m00, float m01, float m02,
         float m10, float m11, float m12,
         float m20, float m21, float m22) {
        m[0][0] = m00; m[0][1] = m01; m[0][2] = m02;
        m[1][0] = m10; m[1][1] = m11; m[1][2] = m12;
        m[2][0] = m20; m[2][1] = m21; m[2][2] = m22;
    }

    // TODO 1b: Matrix-vector multiplication (apply transformation)
    Vec3 operator*(const Vec3& v) const {
        // YOUR CODE HERE
        // result.x = m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z
        // result.y = m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z
        // result.z = m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z
    }

    // TODO 1c: Matrix-matrix multiplication (combine transformations)
    Mat3 operator*(const Mat3& other) const {
        // YOUR CODE HERE
        // Standard matrix multiplication formula
        // result[i][j] = sum of m[i][k] * other[k][j] for k=0,1,2
    }

    // TODO 1d: Create rotation matrix around X axis
    static Mat3 rotationX(float angleRadians) {
        // YOUR CODE HERE
        // Rotation around X-axis:
        // [1,    0,           0        ]
        // [0, cos(θ),    -sin(θ)]
        // [0, sin(θ),     cos(θ)]
    }

    // TODO 1e: Create rotation matrix around Y axis
    static Mat3 rotationY(float angleRadians) {
        // YOUR CODE HERE
        // Rotation around Y-axis:
        // [ cos(θ), 0, sin(θ)]
        // [    0,        1,    0       ]
        // [-sin(θ), 0, cos(θ)]
    }

    // TODO 1f: Create rotation matrix around Z axis
    static Mat3 rotationZ(float angleRadians) {
        // YOUR CODE HERE
        // Rotation around Z-axis:
        // [cos(θ), -sin(θ), 0]
        // [sin(θ),  cos(θ), 0]
        // [   0,           0,        1]
    }

    void print(const std::string& label) const {
        std::cout << label << ":" << std::endl;
        for (int i = 0; i < 3; i++) {
            std::cout << "  [" << m[i][0] << ", " << m[i][1] << ", " << m[i][2] << "]" << std::endl;
        }
    }
};

// ============================================================================
// PART 2: 4x4 Matrix (Transform = Rotation + Translation)
// ============================================================================

class Mat4 {
public:
    float m[4][4];  // Row-major: m[row][col]

    // TODO 2a: Constructor - Initialize to identity
    Mat4() {
        // YOUR CODE HERE
        // Identity matrix:
        // [1, 0, 0, 0]
        // [0, 1, 0, 0]
        // [0, 0, 1, 0]
        // [0, 0, 0, 1]
    }

    // TODO 2b: Create translation matrix
    static Mat4 translation(Vec3 t) {
        // YOUR CODE HERE
        // Translation matrix:
        // [1, 0, 0, tx]
        // [0, 1, 0, ty]
        // [0, 0, 1, tz]
        // [0, 0, 0,  1]
    }

    // TODO 2c: Create rotation + translation matrix
    static Mat4 transform(Mat3 rotation, Vec3 translation) {
        // YOUR CODE HERE
        // Combined matrix:
        // [r00, r01, r02, tx]
        // [r10, r11, r12, ty]
        // [r20, r21, r22, tz]
        // [  0,   0,   0,  1]
    }

    // TODO 2d: Matrix-vector multiplication
    Vec4 operator*(const Vec4& v) const {
        // YOUR CODE HERE
        // result.x = m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z + m[0][3]*v.w
        // result.y = m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z + m[1][3]*v.w
        // result.z = m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z + m[2][3]*v.w
        // result.w = m[3][0]*v.x + m[3][1]*v.y + m[3][2]*v.z + m[3][3]*v.w
    }

    // Convenience: Transform a Vec3 point (assumes w=1)
    Vec3 transformPoint(const Vec3& v) const {
        Vec4 result = (*this) * Vec4(v, 1.0f);
        return result.toVec3();
    }

    // Convenience: Transform a Vec3 direction (assumes w=0, no translation)
    Vec3 transformDirection(const Vec3& v) const {
        Vec4 result = (*this) * Vec4(v, 0.0f);
        return result.toVec3();
    }

    // TODO 2e: Matrix-matrix multiplication
    Mat4 operator*(const Mat4& other) const {
        // YOUR CODE HERE
        // Standard 4x4 matrix multiplication
        // result[i][j] = sum of m[i][k] * other[k][j] for k=0,1,2,3
    }

    void print(const std::string& label) const {
        std::cout << label << ":" << std::endl;
        for (int i = 0; i < 4; i++) {
            std::cout << "  [" << m[i][0] << ", " << m[i][1] << ", "
                      << m[i][2] << ", " << m[i][3] << "]" << std::endl;
        }
    }
};

// ============================================================================
// MAIN - Tests
// ============================================================================

int main() {
    std::cout << "=== SESSION 3.5: Matrix Math for Transforms ===" << std::endl;
    std::cout << std::endl;

    // Test 1: 3x3 Rotation Matrix
    std::cout << "--- Test 1: 3x3 Rotation Around Y-Axis ---" << std::endl;
    Mat3 rotY = Mat3::rotationY(M_PI / 2.0f);  // 90 degrees
    rotY.print("90° Rotation around Y");

    Vec3 point = {1.0f, 0.0f, 0.0f};
    Vec3 rotated = rotY * point;
    point.print("Original point");
    rotated.print("After 90° Y rotation");
    std::cout << "Expected: (0, 0, -1) - X rotates to -Z" << std::endl;
    std::cout << std::endl;

    // Test 2: Combining rotations
    std::cout << "--- Test 2: Combining Rotations ---" << std::endl;
    Mat3 rotX = Mat3::rotationX(M_PI / 4.0f);  // 45° around X
    Mat3 rotZ = Mat3::rotationZ(M_PI / 4.0f);  // 45° around Z
    Mat3 combined = rotZ * rotX;  // Apply X first, then Z
    combined.print("Combined rotation (45° X then 45° Z)");
    std::cout << std::endl;

    // Test 3: 4x4 Transform Matrix
    std::cout << "--- Test 3: 4x4 Transform (Rotation + Translation) ---" << std::endl;
    Vec3 translation = {5.0f, 2.0f, 0.0f};
    Mat4 transform = Mat4::transform(rotY, translation);
    transform.print("Transform: 90° Y rotation + translate (5,2,0)");

    Vec3 localPoint = {1.0f, 0.0f, 0.0f};
    Vec3 worldPoint = transform.transformPoint(localPoint);
    localPoint.print("Local point");
    worldPoint.print("World point");
    std::cout << "Expected: (5, 2, -1) - rotated then translated" << std::endl;
    std::cout << std::endl;

    // Test 4: Real-world scenario
    std::cout << "--- Test 4: OptiTrack Rigid Body Transform ---" << std::endl;
    std::cout << "Scenario: VR controller with 4 markers" << std::endl;

    // Controller transform (position + rotation)
    Vec3 controllerPos = {1.0f, 1.2f, 0.5f};
    Mat3 controllerRot = Mat3::rotationY(M_PI / 6.0f);  // 30° rotation
    Mat4 controllerTransform = Mat4::transform(controllerRot, controllerPos);

    // Marker positions in controller's local space
    Vec3 marker1Local = {0.05f, 0.02f, 0.0f};
    Vec3 marker2Local = {-0.05f, 0.02f, 0.0f};

    // Calculate world positions
    Vec3 marker1World = controllerTransform.transformPoint(marker1Local);
    Vec3 marker2World = controllerTransform.transformPoint(marker2Local);

    marker1Local.print("Marker 1 (local)");
    marker1World.print("Marker 1 (world)");
    marker2Local.print("Marker 2 (local)");
    marker2World.print("Marker 2 (world)");
    std::cout << std::endl;

    std::cout << "=== END OF SESSION 3.5 ===" << std::endl;
    std::cout << "\nCompile with:" << std::endl;
    std::cout << "  clang++ -o session3_5 session3_5_matrices.cpp" << std::endl;
    std::cout << "Run with:" << std::endl;
    std::cout << "  ./session3_5" << std::endl;

    return 0;
}
