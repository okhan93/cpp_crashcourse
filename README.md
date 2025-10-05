# C++ Crash Course - OptiTrack/VR Interview Prep

A hands-on C++ learning project focused on motion tracking concepts for OptiTrack and VR system interviews.

## 🎯 Goal

Learn C++ fundamentals and 3D motion tracking concepts through interactive coding exercises, preparing for technical interviews in medical research/VR environments using OptiTrack systems.

## 📚 Session Overview

### ✅ Session 1: C++ Essentials for Motion Tracking
**Topics:** Pointers, references, arrays, structs
**Files:**
- `session1_pointers_refs.cpp` - Main exercises
- `session1_notes.md` - Study notes
- `session1_review_topics.md` - Deep dive on arrays vs references

**Key Concepts:**
- References (`&`) vs pointers (`*`)
- `const` for safety and intent
- Arrays don't know their own size
- Performance tips (use `x*x` not `pow(x,2)`)

### ✅ Session 2: OOP Basics
**Topics:** Classes, constructors, methods, encapsulation
**Files:**
- `session2_oop_basics.cpp` - Main exercises
- `session2_notes.md` - Study notes
- `session2_quiz_answers.cpp` - Knowledge check quiz

**Key Concepts:**
- Classes vs structs
- Private/public encapsulation
- Constructors (normal vs initializer list)
- `const` methods
- The hidden `this` pointer
- Ternary operator
- Assignment (`=`) vs comparison (`==`)

### 🚧 Session 3: Motion Tracking Fundamentals (IN PROGRESS)
**Topics:** Vector math, quaternions, transformations
**Files:**
- `session3_motion_tracking.cpp` - Main exercises (TODO: Complete vector operations)

**Key Concepts:**
- 3D vector operations (add, subtract, dot, cross)
- Quaternions for rotation (avoiding gimbal lock)
- Rigid body transforms
- Local vs world space

### 📋 Session 4: OptiTrack Simulation (PLANNED)
**Topics:** Simulating OptiTrack SDK patterns

### 📋 Session 5: VR Integration Concepts (PLANNED)
**Topics:** Applying concepts to real VR scenarios

## 🚀 Getting Started

### Prerequisites
- C++ compiler (clang++ on Mac, g++ on Linux, MSVC on Windows)
- Git
- Text editor or IDE

### Setup on a New Computer

1. **Clone the repository:**
   ```bash
   git clone https://github.com/okhan93/cpp_crashcourse.git
   cd cpp_crashcourse
   ```

2. **Verify you have a C++ compiler:**
   ```bash
   # On Mac/Linux:
   clang++ --version
   # or
   g++ --version

   # On Windows (Visual Studio):
   cl
   ```

3. **Review where you left off:**
   - Check session notes (`.md` files)
   - Look for `TODO` comments in `.cpp` files
   - Compile and run completed sessions to refresh

### Compiling and Running

Each session has its own executable:

```bash
# Session 1
clang++ -o session1 session1_pointers_refs.cpp
./session1

# Session 2
clang++ -o session2 session2_oop_basics.cpp
./session2

# Session 3
clang++ -o session3 session3_motion_tracking.cpp
./session3
```

**Note:** Executables are gitignored - you'll need to compile on each machine.

## 📖 Learning Workflow

1. **Read the session notes** (`.md` files) first
2. **Open the exercise file** (`.cpp` file)
3. **Complete TODOs** one at a time
4. **Compile and test** after each TODO
5. **Review notes** if you get stuck
6. **Complete the knowledge check quiz** (if available)

## 🔄 Continuing from Another Computer

Your progress is tracked in the code files themselves (completed TODOs). To continue:

1. **Pull latest changes:**
   ```bash
   git pull origin main
   ```

2. **Find where you left off:**
   - Search for `// YOUR CODE HERE` in `.cpp` files
   - Check for incomplete `TODO` comments

3. **Continue coding!**

4. **When done, commit and push:**
   ```bash
   git add .
   git commit -m "Completed Session X TODOs"
   git push origin main
   ```

## 💡 Current Status

**Last completed:** Session 2 (OOP Basics)
**Currently working on:** Session 3, TODO 1 (Vector operations)
**Next up:** Complete vector math, then quaternions

## 📝 Study Notes Quick Reference

- **References vs Pointers:** References are for function parameters to avoid copies, pointers for arrays
- **`const` methods:** Methods that don't modify the object
- **Performance:** Prefer `x*x` over `pow(x,2)`, use references for large structs
- **The `this` pointer:** Use when parameter names match member names
- **Operator overloading:** Defining what `+`, `-`, `*` mean for custom types

## 🎓 Interview Prep Tips

- Understand **why** OptiTrack uses certain patterns (performance, real-time constraints)
- Be able to explain **const correctness**
- Know the difference between **Euler angles and quaternions**
- Understand **coordinate transformations** (local to world space)
- Practice explaining **vector dot/cross products** and their use cases

## 🤝 Contributing

This is a personal learning project. Feel free to fork and adapt for your own learning!

## 📄 License

Educational use - feel free to learn and share!

---

**Repository:** https://github.com/okhan93/cpp_crashcourse
**Learning Partner:** Claude (Anthropic)
