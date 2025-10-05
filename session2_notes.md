# Session 2: OOP Basics for Motion Tracking - Study Notes

## 1. Classes vs Structs

### What's the Difference?

**Struct (from Session 1):**
```cpp
struct Point3D {
    float x, y, z;  // Public by default
};
```

**Class:**
```cpp
class RigidBody {
private:           // Private by default
    std::string name;
    Point3D position;

public:
    void updatePosition(float x, float y, float z) { }
};
```

**Key Difference:**
- **Struct:** Members are `public` by default (anyone can access)
- **Class:** Members are `private` by default (only the class can access)

**In practice:** Use `struct` for simple data containers, use `class` when you want to control access to data.

## 2. Private vs Public (Encapsulation)

### Why Hide Data?

```cpp
class RigidBody {
private:
    // Hidden from outside - can only be accessed by methods in this class
    std::string name;
    Point3D position;
    bool isTracked;

public:
    // Accessible from anywhere
    std::string getName() const { return name; }
    void updatePosition(float x, float y, float z) {
        position = {x, y, z};
        isTracked = true;
    }
};
```

**Benefits:**
- **Control:** Can validate data before setting
- **Safety:** Can't accidentally break internal state
- **Flexibility:** Can change internal implementation without breaking code that uses the class

**Real example:**
```cpp
RigidBody headset("VR_Headset", 1);

// ✗ Can't do this - position is private
headset.position.x = 100;

// ✓ Must use public method
headset.updatePosition(100, 0, 0);  // Can add validation here!
```

## 3. Constructors

### What is a Constructor?

A **constructor** is a special method that runs when you create an object. It initializes the object's data.

```cpp
class RigidBody {
private:
    std::string name;
    int id;
    Point3D position;
    bool isTracked;

public:
    // Constructor - same name as class, no return type
    RigidBody(std::string bodyName, int bodyId) {
        name = bodyName;
        id = bodyId;
        position = {0.0f, 0.0f, 0.0f};
        isTracked = false;
    }
};

// Using the constructor:
RigidBody headset("VR_Headset", 1);  // Calls constructor
```

### Two Ways to Write Constructors

**Method 1: Assignment in body (beginner-friendly)**
```cpp
RigidBody(std::string bodyName, int bodyId) {
    name = bodyName;
    id = bodyId;
    position = {0.0f, 0.0f, 0.0f};
    isTracked = false;
}
```

**Method 2: Initializer list (more idiomatic C++)**
```cpp
RigidBody(std::string bodyName, int bodyId)
    : name(bodyName),
      id(bodyId),
      position({0.0f, 0.0f, 0.0f}),
      isTracked(false) {
    // Body can be empty - members already initialized
}
```

**Both work!** Initializer lists are slightly more efficient but both are fine for this use case.

## 4. Methods (Member Functions)

### What are Methods?

**Methods** are functions that belong to a class and operate on the class's data.

```cpp
class RigidBody {
private:
    Point3D position;
    bool isTracked;

public:
    // Method that modifies the object
    void updatePosition(float x, float y, float z) {
        position.x = x;
        position.y = y;
        position.z = z;
        isTracked = true;
    }

    // Method that just reads data
    float distanceFromOrigin() const {
        return sqrt(position.x * position.x +
                    position.y * position.y +
                    position.z * position.z);
    }
};
```

### Calling Methods

```cpp
RigidBody headset("VR_Headset", 1);

headset.updatePosition(1.5f, 1.8f, 0.5f);  // Calls method on headset object
float dist = headset.distanceFromOrigin();  // Calls another method
```

### How Methods Access Member Variables

**Key question:** How does a method know which object's data to access?

```cpp
class RigidBody {
private:
    Point3D position;  // Each object has its own position

public:
    void updatePosition(float x, float y, float z) {
        position.x = x;  // Which position? THIS object's position!
    }
};

// Creating multiple objects:
RigidBody headset("VR_Headset", 1);
RigidBody controller("Controller", 2);

headset.updatePosition(1.0f, 2.0f, 3.0f);
// Inside updatePosition, "position" refers to headset's position

controller.updatePosition(5.0f, 6.0f, 7.0f);
// Inside updatePosition, "position" refers to controller's position
```

**Each object has its own copy of member variables!**

### The Hidden `this` Pointer

Behind the scenes, every method receives a hidden pointer called `this`:

```cpp
// What you write:
void updatePosition(float x, float y, float z) {
    position.x = x;
}

// What the compiler actually does (conceptually):
void updatePosition(RigidBody* this, float x, float y, float z) {
    this->position.x = x;  // "this" points to the current object
}
```

**You can use `this` explicitly if you want:**

```cpp
void updatePosition(float x, float y, float z) {
    this->position.x = x;  // Explicit - same as below
    position.x = x;        // Implicit - compiler adds "this->" automatically
}

// Common use case - when parameter name matches member name:
void setPosition(Point3D position) {
    this->position = position;  // this->position is member, position is parameter
}
```

**Key takeaway:** Methods automatically know which object's data to access based on which object you called the method on!

## 5. The `const` Keyword with Methods

### `const` After Method Name

```cpp
std::string getName() const {  // <-- const here!
    return name;
}
```

**What it means:** "This method promises NOT to modify any member variables"

```cpp
class RigidBody {
private:
    std::string name;
    Point3D position;

public:
    // ✓ Can be const - just reads data
    std::string getName() const {
        return name;
    }

    // ✓ Can be const - calculates but doesn't modify
    float distanceFromOrigin() const {
        return sqrt(position.x * position.x + ...);
    }

    // ✗ Cannot be const - modifies position
    void updatePosition(float x, float y, float z) {
        position = {x, y, z};  // Modifying data!
    }
};
```

### Why Use `const` Methods?

1. **Compiler enforces it** - prevents accidental modifications
2. **Documentation** - tells readers "this is read-only"
3. **Works with const objects:**

```cpp
const RigidBody headset("VR_Headset", 1);
std::string n = headset.getName();  // ✓ Works - getName() is const
headset.updatePosition(1, 2, 3);    // ✗ Error - can't call non-const method on const object
```

## 6. Getter and Setter Methods

### Common Pattern

```cpp
class RigidBody {
private:
    std::string name;  // Private - can't access directly
    Point3D position;

public:
    // Getter - reads private data
    std::string getName() const {
        return name;
    }

    // Getter - returns copy
    Point3D getPosition() const {
        return position;
    }

    // Setter - modifies private data (with control!)
    void updatePosition(float x, float y, float z) {
        // Could add validation here:
        // if (x < 0) return;  // Reject invalid positions

        position = {x, y, z};
        isTracked = true;  // Automatically update tracking status
    }
};
```

**Why getters/setters instead of public members?**
- Can add validation
- Can update related data automatically
- Can change internal representation later without breaking code

## 7. Common Pitfall: Assignment vs Comparison

### Critical Bug to Avoid

```cpp
// ✗ WRONG - Assignment (=)
if (isActive = true) {  // Sets isActive to true, always executes!
    framesProcessed++;
}

// ✓ CORRECT - Comparison (==)
if (isActive == true) {  // Checks if isActive equals true
    framesProcessed++;
}

// ✓ BEST - Direct boolean check
if (isActive) {  // Most idiomatic for booleans
    framesProcessed++;
}
```

### The Difference

```cpp
=   // Assignment operator - SETS a value
==  // Comparison operator - CHECKS equality
```

**Common mistake:**
```cpp
if (x = 5) {  // ✗ Sets x to 5, then evaluates as true (always executes!)
    // This always runs!
}

if (x == 5) {  // ✓ Checks if x equals 5
    // Only runs if x is 5
}
```

**For booleans, just use the variable directly:**
```cpp
if (isActive == true)  // Works, but verbose
if (isActive)          // Better - more idiomatic

if (isActive == false) // Works, but verbose
if (!isActive)         // Better - ! means "not"
```

## 8. Ternary Operator (Conditional Operator)

### What is the Ternary Operator?

A shorthand for simple if-else statements:

```cpp
// Long form - if/else
std::string status;
if (isTracked) {
    status = "yes";
} else {
    status = "no";
}

// Short form - ternary operator
std::string status = isTracked ? "yes" : "no";
```

### Syntax

```cpp
condition ? value_if_true : value_if_false
```

### Common Use Cases

**Printing different messages:**
```cpp
void printInfo() const {
    std::cout << "Tracked: " << (isTracked ? "yes" : "no") << std::endl;
}

// Prints: "Tracked: yes" or "Tracked: no"
```

**Selecting values:**
```cpp
int max = (a > b) ? a : b;  // Get maximum of two numbers

float speed = isActive ? 1.0f : 0.0f;  // 1.0 if active, 0.0 if not
```

**Why use parentheses `(isTracked ? "yes" : "no")`?**

When using in `std::cout` chains, wrap in parentheses for clarity:
```cpp
// ✓ Clear - wrapped in parentheses
std::cout << (isTracked ? "yes" : "no") << std::endl;

// ✗ Can be confusing without them
std::cout << isTracked ? "yes" : "no" << std::endl;  // Operator precedence issues
```

### When to Use

- **Use ternary:** Simple, single value selection
- **Use if/else:** Complex logic, multiple statements

**Good use:**
```cpp
return isValid ? 1 : 0;
std::string msg = hasError ? "Error" : "OK";
```

**Bad use (too complex):**
```cpp
// Don't do this - hard to read!
int x = (a > b) ? (c > d ? 1 : 2) : (e > f ? 3 : 4);  // Nested ternary - confusing
```

## 8. C++ Strings: `std::string` vs `const char*`

### Two String Types

```cpp
// C-style string (Session 1)
const char* name = "VR_Headset";  // Pointer to character array

// C++ string (Session 2)
std::string name = "VR_Headset";  // String object
```

### `std::string` Advantages

```cpp
std::string name = "VR";
name = name + "_Headset";      // Easy concatenation
int len = name.length();       // Easy to get length
if (name == "VR_Headset") {}   // Easy comparison

// vs C-style (more complex):
const char* name = "VR";
// Can't easily concatenate, need strcmp() for comparison, etc.
```

### The `std::` Prefix

```cpp
std::string     // String from C++ standard library
std::cout       // Output stream from standard library
std::endl       // End line from standard library
```

**`std::`** means "standard library namespace"

### When to Use Each

- **`std::string`** - Most modern C++ code, easier to work with
- **`const char*`** - OptiTrack SDK often uses this for compatibility with C

**You'll see both in interviews**, so know they're different ways to handle text.

## 8. Object-Oriented Mindset

### Procedural (Session 1) vs OOP (Session 2)

**Procedural - Data and functions separate:**
```cpp
struct Point3D {
    float x, y, z;
};

float calculateDistance(const Point3D& p1, const Point3D& p2) {
    // Function is separate from data
}
```

**OOP - Data and functions together:**
```cpp
class RigidBody {
private:
    Point3D position;  // Data

public:
    float distanceFromOrigin() const {  // Function belongs to the class
        // Can directly access position
    }
};
```

### Real-World OptiTrack Example

```cpp
// OptiTrack SDK provides classes like:
class NatNetClient {
private:
    ServerDescription serverInfo;
    ConnectionType connectionType;

public:
    int Connect(char* serverIP);
    int Disconnect();
    int GetDataDescriptions(sDataDescriptions** pDataDescriptions);
};

// You use it like:
NatNetClient client;
client.Connect("127.0.0.1");
```

## 9. Key Takeaways

✓ **Classes bundle data + methods together**
✓ **`private` hides data, `public` exposes interface**
✓ **Constructors initialize objects**
✓ **`const` methods don't modify the object**
✓ **Getters/setters control access to private data**
✓ **`std::string` is easier than `const char*` for most tasks**
✓ **OOP is about organizing code around objects (like RigidBody, Camera)**

## 10. Common Interview Topics

- "What's the difference between a struct and a class?" → Default access: public vs private
- "Why use private members?" → Encapsulation, control, validation
- "What does `const` after a method mean?" → Method doesn't modify the object
- "What's a constructor?" → Special method that initializes an object when created
- "When would you use `std::string` vs `const char*`?" → std::string is easier, const char* for C compatibility

---

**Next:** Complete TODO 1c-1e and TODO 2 to practice these concepts!
