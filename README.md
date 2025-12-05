# H1_2 Walk2 — Task integration for MuJoCo MPC

This repository provides the **`h1_2_walk2`** task to be run inside
[DeepMind’s MuJoCo MPC](https://github.com/google-deepmind/mujoco_mpc).

Follow the steps below **in order**. After step 5 you can build and run the
MuJoCo MPC app and select the `h1_2_walk2` task.

---

## 0) Prerequisites

1. Install MuJoCo MPC by **following the official instructions**:  
   https://github.com/google-deepmind/mujoco_mpc  
   Make sure the base project builds and runs before adding this task.

---

## 1) Copy the task folder into MuJoCo MPC

Copy this repository’s `h1_2_walk` folder into the MuJoCo MPC tasks directory:

---

## 2) Update CMake in `mjpc/tasks`

Open:
```
/path/to/mujoco_mpc/mjpc/tasks/CMakeLists.txt
```

Append the following block (exactly as below), which ensures the XML and meshes
for the task are copied into the build tree:

```cmake
# h1_2_walk2
COMMAND ${CMAKE_COMMAND} -E make_directory
        ${CMAKE_CURRENT_BINARY_DIR}/h1_2_walk2
COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_CURRENT_SOURCE_DIR}/h1_2_walk2/h1_2.xml
        ${CMAKE_CURRENT_BINARY_DIR}/h1_2_walk2/h1_2.xml
COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/h1_2_walk2/meshes
        ${CMAKE_CURRENT_BINARY_DIR}/h1_2_walk2/meshes

## 3) Register the task in `tasks.cc`

Open:
```
/path/to/mujoco_mpc/mjpc/tasks/tasks.cc
```

- Add the include near the other task includes:
```cpp
#include "mjpc/tasks/h1_2_walk2/h1_2_walk.h"
```

- Add the task to the list where tasks are registered/returned:
```cpp
std::make_shared<H1_2_walk_Task2>(),
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
tasks/h1_2_walk2/h1_2_walk.cc
tasks/h1_2_walk2/h1_2_walk.h
tasks/h1_2_walk2/h1_2_functions.cc
tasks/h1_2_walk2/h1_2_functions.h
```


## 5) Re-configure build and Run MuJoCo MPC

Launch the MuJoCo MPC binary (GUI or CLI as provided by the project).
Then **select the `h1_2_walk2` task** from the task list (or via the project’s
`--task` flag).


