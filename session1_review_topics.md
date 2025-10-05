# Session 1: Topics to Review

## ⚠️ Key Concept to Solidify: Arrays vs Single Objects

### The Core Confusion
**Question:** Why do we use references (`&`) for single structs but pointers (`*`) for arrays?

**The Answer:**

#### Single Struct Behavior
```cpp
// WITHOUT reference - creates a COPY
void modify(Point3D p) {
    p.x = 100;  // Modifies the copy, original unchanged
}

// WITH reference - works with original
void modify(Point3D& p) {
    p.x = 100;  // Modifies the original
}

Point3D point = {1.0f, 2.0f, 3.0f};
modify(point);  // Which version did we call?
```

#### Array Behavior
```cpp
// Arrays AUTOMATICALLY pass by reference (as pointers)
void modify(Point3D* markers, int count) {
    markers[0].x = 100;  // Modifies the ORIGINAL array
}

Point3D myMarkers[10];
modify(myMarkers, 10);  // No copying! Array name = pointer to first element
// myMarkers[0].x is now 100
```

### Why This Matters

**Arrays are ALREADY like references:**
- When you write `myMarkers`, it automatically becomes a pointer to the first element
- This is called "array decay" - arrays decay to pointers when passed to functions
- You CANNOT accidentally copy an array in C++ (unlike structs)

**Structs need explicit `&`:**
- By default, structs are copied when passed to functions
- You must use `&` to avoid the copy and work with the original

### Visual Comparison

```cpp
// SINGLE STRUCT
Point3D p;
modify(p);         // ✗ Copies the struct (bad)
modify(p&);        // ✗ Wrong syntax!
// Function needs: void modify(Point3D& p)

// ARRAY
Point3D arr[10];
modify(arr, 10);   // ✓ Already a pointer (good)
modify(&arr, 10);  // ✗ Wrong! Double pointer now
// Function needs: void modify(Point3D* arr, int count)
```

### The `const` Layer

```cpp
// Read-only access
void read(const Point3D& p)          // Single struct - can't modify
void read(const Point3D* arr, int n) // Array - can't modify

// Can modify
void write(Point3D& p)               // Single struct - can modify
void write(Point3D* arr, int n)      // Array - can modify
```

### Key Takeaway

| Type | Pass by value (copy) | Pass by reference (no copy) |
|------|---------------------|---------------------------|
| Single struct | `Point3D p` | `Point3D& p` or `const Point3D& p` |
| Array | Not possible in C++ | `Point3D* arr` or `const Point3D* arr` (automatic!) |

---

## Review Questions to Test Understanding

1. **Why can't you accidentally copy an array in C++?**
   - Answer: Arrays automatically decay to pointers when passed to functions

2. **What does "array decay" mean?**
   - Answer: Array name automatically converts to a pointer to the first element

3. **If arrays are already "reference-like", why do we use `*` instead of `&`?**
   - Answer: That's the C++ syntax for arrays. `Point3D&` means "reference to ONE Point3D", while `Point3D*` means "pointer to Point3D(s)" - can point to a single one or an array

4. **When would you use `const` with a pointer?**
   - Answer: When you want to read the data but promise not to modify it

---

## Practice Exercise

Complete this code:

```cpp
// Function that takes an array and doubles all x values
void doubleX(/* YOUR SIGNATURE HERE */) {
    // YOUR CODE HERE
}

// Function that calculates average x without modifying array
float averageX(/* YOUR SIGNATURE HERE */) {
    // YOUR CODE HERE
}
```

**Answers:**
```cpp
void doubleX(Point3D* markers, int count) {  // No const - we modify
    for (int i = 0; i < count; i++) {
        markers[i].x *= 2;
    }
}

float averageX(const Point3D* markers, int count) {  // const - read only
    float sum = 0.0f;
    for (int i = 0; i < count; i++) {
        sum += markers[i].x;
    }
    return sum / count;
}
```
