# H1_2  — Task integration for MuJoCo MPC


https://github.com/user-attachments/assets/d5dcf97c-d1ba-455c-a610-d1f77b5e2012


This repository provides the **`h1_2_walk`** task to be run inside
[DeepMind’s MuJoCo MPC](https://github.com/google-deepmind/mujoco_mpc).

Follow the steps below **in order**. After step 5 you can build and run the
MuJoCo MPC app and select the `h1_2_walk` task.

---

## 0) Prerequisites

1. Install MuJoCo MPC by **following the official instructions**:  
   https://github.com/google-deepmind/mujoco_mpc  
   Make sure the base project builds and runs before adding this task.

---

## 1) Copy the task folder into MuJoCo MPC

Copy this repository’s `h1_2` folder into the MuJoCo MPC tasks directory:

---

## 2) Update CMake in `mjpc/tasks`

Open:
```
/path/to/mujoco_mpc/mjpc/tasks/CMakeLists.txt
```

Append the following block (exactly as below), which ensures the XML and meshes
for the task are copied into the build tree:

```cmake
# h1_2_walk
  COMMAND ${CMAKE_COMMAND} -E make_directory
          ${CMAKE_CURRENT_BINARY_DIR}/h1_2
  COMMAND ${CMAKE_COMMAND} -E copy
          ${CMAKE_CURRENT_SOURCE_DIR}/h1_2/h1_2_walk.xml
          ${CMAKE_CURRENT_BINARY_DIR}/h1_2/h1_2_walk.xml
  COMMAND ${CMAKE_COMMAND} -E copy_directory
          ${CMAKE_CURRENT_SOURCE_DIR}/h1_2/meshes
          ${CMAKE_CURRENT_BINARY_DIR}/h1_2/meshes
```
## 3) Register the task in `tasks.cc`

Open:
```
/path/to/mujoco_mpc/mjpc/tasks/tasks.cc
```

- Add the include near the other task includes:
```cpp
#include "mjpc/tasks/h1_2/walk/h1_2_walk.h"
#include "mjpc/tasks/h1_2/obstacle/h1_2_obstacle.h"
#include "mjpc/tasks/h1_2/integrated/h1_2_integrated.h" 
```

- Add the task to the list where tasks are registered/returned:
```cpp
      std::make_shared<H1_2_walk>(),  
      std::make_shared<H1_2_obstacle_Task>(), 
      std::make_shared<H1_2_integrated_Task>(), 
```

(Place it alongside the other `std::make_shared<...>()` entries.)

---

## 4) Add sources to the main `mjpc/CMakeLists.txt`

Open:
```
/path/to/mujoco_mpc/mjpc/CMakeLists.txt
```

Add these files to the appropriate source lists so they are compiled:

```
  tasks/h1_2/walk/h1_2_walk.cc   # da me
  tasks/h1_2/walk/h1_2_walk.h    # da me
  tasks/h1_2/obstacle/h1_2_obstacle.cc   # da me
  tasks/h1_2/obstacle/h1_2_obstacle.h    # da me
  tasks/h1_2/integrated/h1_2_integrated.cc   # da me
  tasks/h1_2/integrated/h1_2_integrated.h    # da me
tasks/h1_2_functions.cc
tasks/h1_2_functions.h
```


## 5) Re-configure build and Run MuJoCo MPC

Launch the MuJoCo MPC binary (GUI or CLI as provided by the project).
Then **select the `h1_2_walk` task** from the task list (or via the project’s
`--task` flag).

---

## ⚠️ Important Note — Core File Modifications

This task introduces several **non-destructive additions** to the original MuJoCo MPC source files.  
No existing functionality was removed; all modifications only **extend** the framework.

These additions were required to:
- Enable **obstacle detection and visualization** in the integrated task  
- Allow **recording and saving monitoring data**
- **Automatically start the planner** when the task is activated  
  (needed for experiments when the GUI is disabled)

### Modified Source Files
The following MuJoCo MPC files contain added logic:
- `simulate.cc`
- `main.cc`
- `agent.cc`

### Configuration in `simulate.cc`
In particular, these options can be configured inside `simulate.cc`
around **lines 625–628**:

```cpp
bool record_joint = true;
bool record_data = true;
bool integrated_view = true;
bool record_point_cloud = true;


