# Session 1: C++ Essentials for Motion Tracking - Study Notes

## 1. References (`&`) - Core Concept

### What is a Reference?
A reference is an **alias** to existing data - another name for the same memory location, NOT a copy.

```cpp
int x = 5;
int& ref = x;  // ref is another name for x
ref = 10;      // x is now 10 (same memory location)
```

### Why Use References?

**Performance:** Avoid expensive copying of large data structures
```cpp
struct Point3D {
    float x, y, z;
};

// BAD: Copies entire struct every time (slow, wasteful)
void process(Point3D point) { ... }

// GOOD: Works with original, no copy (fast)
void process(Point3D& point) { ... }

// BEST: Read-only access, no copy, compiler-enforced safety
void process(const Point3D& point) { ... }
```

### The `const` Keyword

Use `const` to indicate you won't modify the data:

```cpp
// Read-only: calculateDistance doesn't need to modify points
float calculateDistance(const Point3D& p1, const Point3D& p2) {
    // p1.x = 0;  // COMPILER ERROR! const prevents bugs
    float dx = p2.x - p1.x;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

// Needs to modify: updatePosition changes the point
void updatePosition(Point3D& point, float dx, float dy, float dz) {
    point.x += dx;  // Must modify - can't use const
    point.y += dy;
    point.z += dz;
}
```

### Decision Tree: When to Use What?

```
Need to modify the data?
├─ YES → Use `Type&`
└─ NO → Is it a large struct/object?
    ├─ YES → Use `const Type&`
    └─ NO (small: int, float, etc.) → Pass by value `Type`
```

### Common Misconception: References in Assignment

**References are for FUNCTION PARAMETERS, not regular assignment:**

```cpp
// ✓ CORRECT - References in function parameters
void modify(Point3D& p) {    // & makes it a reference parameter
    p.x = 100;
}

void calculate(const Point3D& p) {  // & for read-only reference
    float x = p.x;
}

// ✗ WRONG - Don't use & in regular assignment for simple types
float largest = markers[i].y;   // ✓ Correct - simple assignment
float& largest = markers[i].y;  // ✗ Wrong - unnecessary reference

// ✓ OK - Reference can be useful for complex repeated access
Point3D& current = markers[i];  // Avoid writing markers[i] many times
current.x += 10;
current.y += 20;
current.z += 30;
```

**Rule of thumb:**
- **Small types (int, float, char):** Just copy them with `=` - it's fast!
- **Large types (structs, objects):** Use `&` in function parameters to avoid copying
- **Regular variables:** Usually just use `=`, not `&`

## 2. Arrays and Pointers (`*`) - The Basics

### Critical Concept: Arrays Don't Know Their Own Size

**In C++, when you pass an array to a function, you MUST also pass its size.**

```cpp
// You create an array - YOU know it has 4 elements
Point3D markers[4] = {
    {0.0f, 0.0f, 0.0f},
    {1.0f, 0.0f, 0.0f},
    {1.0f, 1.0f, 0.0f},
    {0.0f, 1.0f, 0.0f}
};

// Function receives a POINTER to the array (doesn't know size!)
Point3D calculateCentroid(Point3D* markers, int numMarkers) {
    // markers is a pointer - can't tell how many elements
    // numMarkers tells us: "this array has 4 elements"
}

// When calling, YOU provide both the array and the count
Point3D centroid = calculateCentroid(markers, 4);
                                            // ^^ you tell it the size
```

### Why OptiTrack Works This Way

OptiTrack **creates** the array, so it already knows the size:

```cpp
// OptiTrack SDK provides BOTH array and count:
sFrameOfMocapData* frameData;
int markerCount = frameData->nMarkers;      // "I found 47 markers"
MarkerData* markers = frameData->Markers;   // Array of 47 markers

// You use both pieces of info:
for (int i = 0; i < markerCount; i++) {
    processMarker(markers[i]);
}
```

### Accessing Array Elements

Use `[]` to access elements by index:

```cpp
Point3D* markers;  // Pointer to array
int numMarkers = 4;

for (int i = 0; i < numMarkers; i++) {
    markers[i].x;  // Access x field of i-th marker
    markers[i].y;  // Access y field of i-th marker
    markers[i].z;  // Access z field of i-th marker
}
```

### Compound Assignment Operators

These are shortcuts for common operations:

```cpp
// Long form vs. compound form
x = x + 5;    →    x += 5;     // Add and assign
x = x - 3;    →    x -= 3;     // Subtract and assign
x = x * 2;    →    x *= 2;     // Multiply and assign
x = x / 4;    →    x /= 4;     // Divide and assign
```

**Real example from centroid calculation:**
```cpp
// Accumulating values
centroid.x += markers[i].x;  // Same as: centroid.x = centroid.x + markers[i].x

// Computing average
centroid.x /= numMarkers;    // Same as: centroid.x = centroid.x / numMarkers
```

### Loop Increment: `i++` vs `++i`

Both increment `i` by 1, but subtle difference:

```cpp
for (int i = 0; i < numMarkers; i++)   // Post-increment (common)
for (int i = 0; i < numMarkers; ++i)   // Pre-increment (slightly more efficient)
```

**For simple loops, either works fine.** Some C++ developers prefer `++i` as a best practice (marginally faster in some cases), but modern compilers optimize both to the same code.

### Typical Pattern: Sum and Average

Common in motion tracking - calculate average position:

```cpp
// Step 1: Initialize accumulator to zero
Point3D centroid = {0.0f, 0.0f, 0.0f};

// Step 2: Sum all values
for (int i = 0; i < numMarkers; i++) {
    centroid.x += markers[i].x;
    centroid.y += markers[i].y;
    centroid.z += markers[i].z;
}

// Step 3: Divide by count to get average
centroid.x /= numMarkers;
centroid.y /= numMarkers;
centroid.z /= numMarkers;
```

## 3. Accessing Struct Members

Use the dot operator (`.`) to access fields:

```cpp
Point3D marker = {1.0f, 2.0f, 3.0f};
float x = marker.x;  // Access x field
marker.y = 5.0f;     // Modify y field
```

## 4. Console Output in C++

### The `std::cout` Stream

C++ uses **stream operators** for output (very different from `print()` in Python or `printf()` in C):

```cpp
#include <iostream>  // Required for std::cout

// Chain multiple values with << operator
std::cout << "Hello" << " " << "World" << std::endl;

// Mix strings and variables
int x = 42;
std::cout << "The answer is: " << x << std::endl;

// Print struct fields
Point3D p = {1.0f, 2.0f, 3.0f};
std::cout << "Point: (" << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;
```

### Key Differences from Other Languages

| Language | Syntax |
|----------|--------|
| Python | `print(f"Point: ({p.x}, {p.y}, {p.z})")` |
| C | `printf("Point: (%f, %f, %f)\n", p.x, p.y, p.z);` |
| C++ | `std::cout << "Point: (" << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;` |

### Common Components

```cpp
std::cout          // "standard character output stream"
<<                 // Stream insertion operator (chains values)
std::endl          // End line (prints newline + flushes buffer)
"\n"               // Just newline (faster, doesn't flush)
```

### Example from TODO 4

```cpp
void printPoint(const Point3D& p, const char* label) {
    std::cout << label << ": ("
              << p.x << ", "
              << p.y << ", "
              << p.z << ")"
              << std::endl;
}

// Output: "Head: (1.5, 2.3, 0.8)"
```

### `const char*` - C-Style Strings

```cpp
const char* label = "Head";  // Pointer to constant character array
```

This is C++'s way of handling text strings (there's also `std::string`, but OptiTrack SDK often uses C-style strings for compatibility).

## 5. Compiling and Running C++ Programs

### The Two-Step Process

Unlike interpreted languages (Python, JavaScript), C++ requires **compilation** before running:

```bash
# Step 1: Compile - converts C++ code to executable
clang++ -o session1 session1_pointers_refs.cpp

# Step 2: Run - execute the program
./session1
```

### Understanding the Compile Command

```
clang++                              The C++ compiler (Apple's version on Mac)
   -o session1                       "output" flag - name the executable "session1"
             session1_pointers_refs.cpp    The source code file to compile
```

**What happens during compilation:**
1. Compiler reads your `.cpp` file
2. Checks for syntax errors
3. Converts C++ code to machine code
4. Creates an executable file named `session1`

**Without `-o session1`:** Compiler creates `a.out` (default name)

### Understanding the Run Command

```
./              Means "current directory"
  session1      The executable file to run
```

**Why the `./`?** For security, your shell doesn't automatically run programs from the current directory. You must explicitly say "run THIS program from THIS directory".

### Common Errors

**Compile-time errors (syntax issues):**
```
error: use of undeclared identifier 'std::count'
       did you mean 'std::cout'?
```
→ Fix the code, then compile again

**Link errors (missing libraries):**
```
undefined reference to 'sqrt'
```
→ Need to include proper headers or link libraries

**Runtime errors (crashes during execution):**
```
Segmentation fault (core dumped)
```
→ Program compiled successfully but crashes (usually pointer/array issues)

### Development Workflow

```bash
# 1. Write code
# 2. Compile
clang++ -o session1 session1_pointers_refs.cpp

# 3. If compilation fails, fix errors and go to step 2
# 4. If compilation succeeds, run
./session1

# 5. If output is wrong, fix code and go to step 2
```

## 6. Performance Tips for Real-Time Systems

### Squaring Numbers
```cpp
// SLOWER: Function call overhead
float result = pow(dx, 2);

// FASTER: Direct multiplication
float result = dx * dx;
```

**Why it matters:** OptiTrack runs at 120+ FPS. Every millisecond counts when processing thousands of markers.

### Returning Values
```cpp
// Unnecessary temporary variable
float calculateDistance(...) {
    float distance = sqrt(dx*dx + dy*dy + dz*dz);
    return distance;  // Extra variable serves no purpose
}

// Better: Direct return
float calculateDistance(...) {
    return sqrt(dx*dx + dy*dy + dz*dz);
}
```

**When to use temporary variables:**
- Debugging/printing intermediate values
- Complex expressions that benefit from being broken up
- Value is used multiple times in the function

### Breaking Down Complex Calculations
```cpp
// More readable and debuggable
float dx = p2.x - p1.x;
float dy = p2.y - p1.y;
float dz = p2.z - p1.z;
return sqrt(dx*dx + dy*dy + dz*dz);

// vs. everything in one line (harder to debug)
return sqrt((p2.x-p1.x)*(p2.x-p1.x) + (p2.y-p1.y)*(p2.y-p1.y) + (p2.z-p1.z)*(p2.z-p1.z));
```

## 4. Application to OptiTrack/VR

### Real-World Scenario

OptiTrack sends marker data at high frame rates (120+ Hz). You need to:
1. Process thousands of markers per frame
2. Apply transformations/calibrations
3. Calculate distances and centroids
4. Do all this WITHOUT copying data unnecessarily

```cpp
// OptiTrack gives you data like this:
sFrameOfMocapData* frameData;
int markerCount = frameData->nMarkers;
MarkerData* markers = frameData->Markers;

// Process efficiently with references (no copying):
void applyCalibration(MarkerData& marker) {
    marker.x += calibrationOffset.x;
    marker.y += calibrationOffset.y;
    marker.z += calibrationOffset.z;
}

// Loop through all markers:
for (int i = 0; i < markerCount; i++) {
    applyCalibration(markers[i]);  // Fast, modifies in place
}
```

## 5. Key Takeaways

✓ **References (`&`)** let you work with original data without copying
✓ **`const`** communicates intent and prevents accidental modifications
✓ **Arrays need size passed separately** - C++ arrays don't know their own size
✓ **Pointers (`*`)** are used for arrays - access elements with `array[i]`
✓ **Performance matters** in real-time tracking (avoid copies, use `x*x` not `pow(x,2)`)
✓ **Readable code** uses intermediate variables when it improves clarity
✓ Understanding references and pointers is CRITICAL for working with OptiTrack SDK

## 6. Common Interview Topics

- "Why use `const &` instead of just `&`?" → Safety, communicates intent, prevents bugs
- "When would you pass by value vs reference?" → Small types by value, large structs by reference
- "How does OptiTrack handle real-time data?" → Pointers and references for zero-copy processing
- "What's the difference between a reference and a pointer?" → (Coming in next session!)

---

**Next Session Preview:** Pointers (`*`), arrays, and how to work with OptiTrack's marker arrays
