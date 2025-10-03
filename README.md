# H1_2 Walk2 — Task integration for MuJoCo MPC

This repository provides the **`h1_2_walk2`** task to be run inside
[DeepMind’s MuJoCo MPC](https://github.com/google-deepmind/mujoco_mpc).

Follow the steps below **in order**. After step 6 you can build and run the
MuJoCo MPC app and select the `h1_2_walk2` task.

---

## 0) Prerequisites

1. Install MuJoCo MPC by **following the official instructions**:  
   https://github.com/google-deepmind/mujoco_mpc  
   Make sure the base project builds and runs before adding this task.
2. You need a working C++ toolchain, CMake and MuJoCo as required by that repo.

---

## 1) Copy the task folder into MuJoCo MPC

Copy this repository’s `h1_2_walk2` folder into the MuJoCo MPC tasks directory:

```bash
cp -r h1_2_walk2 /path/to/mujoco_mpc/mjpc/tasks/
```

After this, you should have:
```
/path/to/mujoco_mpc/mjpc/tasks/h1_2_walk2
├── h1_2.xml
├── h1_2_walk.cc
├── h1_2_walk.h
├── h1_2_functions.cc
├── h1_2_functions.h
└── meshes/
```

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
```

> If your task uses additional assets, add similar `copy` lines for them.

---

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

(Keep the same style used for the other tasks in that file—e.g., add `.cc` to the
sources list and `.h` to the headers list if the project separates them.)

---

## 5) Re-configure and build MuJoCo MPC

From your MuJoCo MPC build directory:

```bash
cd /path/to/mujoco_mpc/build
cmake ..          # reconfigure so the new task is picked up
make -j$(nproc)   # or: cmake --build . --parallel
```

If CMake reports missing files, re-check you added the paths in steps 2–4.

---

## 6) Run and select the task

Launch the MuJoCo MPC binary (GUI or CLI as provided by the project).
Then **select the `h1_2_walk2` task** from the task list (or via the project’s
`--task` flag if supported).

That’s it — the `h1_2_walk2` task should now run inside MuJoCo MPC.

---

## Troubleshooting

- **Headers not found / linker errors**: re-check step **4** (all four files from
  `h1_2_walk2` must be added to `mjpc/CMakeLists.txt`).
- **Assets not found at runtime**: re-check step **2** (the copy commands must
  copy `h1_2.xml` and the entire `meshes/` directory into the build tree).
- **Task not visible in UI**: re-check step **3** (include + registration line).
