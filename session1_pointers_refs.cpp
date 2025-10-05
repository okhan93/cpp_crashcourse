/*
 * SESSION 1: C++ Essentials for Motion Tracking
 *
 * In motion tracking systems like OptiTrack, you'll constantly work with:
 * - Pointers (to access SDK data structures)
 * - References (to pass data efficiently)
 * - Structs (to represent markers, rigid bodies, etc.)
 *
 * This exercise covers the basics you MUST know.
 */

#include <iostream>
#include <cmath>

// A simple 3D point structure (like a marker position from OptiTrack)
struct Point3D {
    float x;
    float y;
    float z;
};

// TODO 1: Complete this function
// Calculate the distance between two 3D points
// Formula: distance = sqrt((x2-x1)^2 + (y2-y1)^2 + (z2-z1)^2)
// Hint: Use the sqrt() and pow() functions from <cmath>
float calculateDistance(const Point3D& p1, const Point3D& p2) {
    //separate the diffrences to make easier to read and debug
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    float dz = p2.z - p1.z;
    //calculate the distance and return directly withough a creating a new variable
    //use temp variables when the expression is too complex or when it is used multiple times and for debugging
    //using dx*dx instead of pow(dx,2) for better performance
    return sqrt(dx*dx + dy*dy + dz*dz); // Replace this
}

// TODO 2: Complete this function
// Update a point's position by adding an offset
// This function should MODIFY the original point (that's why we use a reference)
void updatePosition(Point3D& point, float dx, float dy, float dz) {
    point.x += dx; // Add dx to the x coordinate
    point.y += dy; // Add dy to the y coordinate
    point.z += dz; // Add dz to the z coordinate
}

// TODO 3: Complete this function
// Given an array of marker positions, find the average (centroid) position
// This is commonly used to find the center of a rigid body from its markers
Point3D calculateCentroid(Point3D* markers, int numMarkers) {
    Point3D centroid = {0.0f, 0.0f, 0.0f};

    // YOUR CODE HERE
    // Hint: Loop through all markers, sum their x, y, z values
    // Then divide by numMarkers
    for (int i = 0; i < numMarkers; ++i) {
        centroid.x += markers[i].x;
        centroid.y += markers[i].y;
        centroid.z += markers[i].z;
    }
    centroid.x /= numMarkers;
    centroid.y /= numMarkers;
    centroid.z /= numMarkers;

    return centroid;
}

// TODO 4: Complete this function
// Print a point in a readable format
void printPoint(const Point3D& p, const char* label) {
    // YOUR CODE HERE
    // Print something like: "Label: (x, y, z)"
    std::cout << label << ": (" << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;
}

int main() {
    std::cout << "=== SESSION 1: C++ Essentials for Motion Tracking ===" << std::endl;
    std::cout << std::endl;

    // Test TODO 1: Distance calculation
    std::cout << "--- Test 1: Distance Calculation ---" << std::endl;
    Point3D marker1 = {0.0f, 0.0f, 0.0f};
    Point3D marker2 = {3.0f, 4.0f, 0.0f};

    float distance = calculateDistance(marker1, marker2);
    std::cout << "Distance between marker1 and marker2: " << distance << std::endl;
    std::cout << "Expected: 5.0" << std::endl;
    std::cout << std::endl;

    // Test TODO 2: Update position
    std::cout << "--- Test 2: Update Position ---" << std::endl;
    Point3D headPosition = {1.0f, 1.5f, 0.5f};
    std::cout << "Before update: ";
    printPoint(headPosition, "Head");

    updatePosition(headPosition, 0.1f, -0.05f, 0.2f);
    std::cout << "After update (+0.1, -0.05, +0.2): ";
    printPoint(headPosition, "Head");
    std::cout << "Expected: Head: (1.1, 1.45, 0.7)" << std::endl;
    std::cout << std::endl;

    // Test TODO 3: Centroid calculation
    std::cout << "--- Test 3: Centroid of Rigid Body Markers ---" << std::endl;
    // Simulating 4 markers on a rigid body (like a VR headset)
    Point3D rigidBodyMarkers[4] = {
        {0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
        {1.0f, 1.0f, 0.0f},
        {0.0f, 1.0f, 0.0f}
    };

    Point3D centroid = calculateCentroid(rigidBodyMarkers, 4);
    printPoint(centroid, "Centroid");
    std::cout << "Expected: Centroid: (0.5, 0.5, 0)" << std::endl;
    std::cout << std::endl;

    std::cout << "=== END OF SESSION 1 ===" << std::endl;
    std::cout << "\nOnce you've completed all TODOs, compile with:" << std::endl;
    std::cout << "  clang++ -o session1 session1_pointers_refs.cpp" << std::endl;
    std::cout << "Then run with:" << std::endl;
    std::cout << "  ./session1" << std::endl;

    return 0;
}
