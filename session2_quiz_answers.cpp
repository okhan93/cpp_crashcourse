/*
 * SESSION 2: Knowledge Check Quiz - Your Answers
 *
 * Fill in your answers below for each question.
 * This file won't compile - it's just for writing your answers!
 */

#include <iostream>

// From previous sessions
struct Point3D {
    float x, y, z;
};

// ============================================================================
// QUESTION 1: Class Structure (Practical)
// ============================================================================
// Create a Marker class with:
// - Private: id (int), position (Point3D), isVisible (bool)
// - Constructor that takes: id, x, y, z
// - Getter for ID
// - Method to update visibility status

// YOUR ANSWER HERE:

class Marker{
private:
    int id;
    Point3D position;
    bool isVisible;

public:
    Marker(int markerId, float x, float y, float z)
        : id(markerId), position({x,y,z}), isVisible(false){

        }
    
    int getId() const {
        return id;
    }

    void updateVisibility(bool visible)
    {
       isVisible = visible;
    }




// ============================================================================
// QUESTION 2: const Methods (Theoretical)
// ============================================================================
// Given this code:
/*
class Marker {
private:
    int id;
    Point3D position;
    bool isVisible;

public:
    void setVisible(bool visible) const {
        isVisible = visible;
    }
};
*/

// Will this compile? Why or why not?
// YOUR ANSWER HERE (as a comment):
// You are modifying a variable but using a const.




// ============================================================================
// QUESTION 3: Constructor Initialization (Practical)
// ============================================================================
// Which of these constructors is problematic and why?

// Option A
class CameraA {
    int id;
public:
    CameraA(int cameraId) {
        id = cameraId;
    }
};

// Option B
class CameraB {
    int id;
public:
    CameraB(int id) {
        id = id;  // Sets member id to parameter id
    }
};

// Option C
class CameraC {
    int id;
public:
    CameraC(int id) : id(id) {
    }
};

// YOUR ANSWER HERE (as a comment):
// Which is problematic?
// Why?
// B because we dont know which id is referring to which. if we want to use the id in both instances we need to use the this-> notation to make it explicit




// ============================================================================
// QUESTION 4: The this Pointer (Theoretical)
// ============================================================================
// Explain in your own words: What is the `this` pointer and when would you
// explicitly use it in your code?

// YOUR ANSWER HERE (as a comment):
// this pointer is an explicit way to set the property of the class. we dont need to use it because it is implicit unless we are using the same variable name for the object property and the one we are assigning. then we need to differentiate and make it explicit.






// ============================================================================
// QUESTION 5: Real-World Scenario (Practical)
// ============================================================================
// Should the updatePosition method be const? Why or why not?

class Controller {
private:
    Point3D position;

public:
    void updatePosition(float x, float y, float z) /* const or no const? */ {
        position = {x, y, z};
    }
};

// YOUR ANSWER HERE (as a comment):
// Should it be const?
// Why or why not?
// Should not be ca const becuase we are changinf the property not jusr reading it



// ============================================================================
// When you're done, save this file and let me know!
// ============================================================================
