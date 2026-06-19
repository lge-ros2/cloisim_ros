# cloisim_ros Copilot Instructions

## Overview

**cloisim_ros** is a collection of ROS 2 (Jazzy) packages that provide virtual sensor and actuator nodes for [CLOiSim](https://github.com/lge-ros2/cloisim), a Unity3D-based multi-robot simulator. Communication between CLOiSim and these ROS nodes happens over **ZMQ** (data/info) and **WebSocket** (service port discovery), using **Protobuf** for message serialization.

## General Rules

- Prefer narrow, local fixes over broad refactors.
- Keep C++17 compatibility and existing warning settings.
- Preserve package boundaries and ROS-facing interfaces unless the task requires a contract change.
- Do not mix behavior changes with unrelated cleanup.
- For Harness Engineering work, git commit messages must be written in English only.
- Harness Engineering commit message format:
  - first line: `type(scope): summary` or `type: summary`
  - following lines: indented bullet list with `- ` items describing concrete changes and effects
- Do not write Harness Engineering commit messages in Korean.

### Harness Engineering commit message examples

```text
fix(cloth): improve resting stability on multi-support contacts
    - include contact threshold in ClothGrabberPlugin cache key to avoid stale group activation results
    - reuse cached collider bounds in ClothGrabber to reduce repeated hot-path contact work
    - skip BurstCloth collider updates when collider state is unchanged to avoid needless wake-ups
    - make sleep detection tolerate persistent low-energy resting contact
    - strengthen resting-contact tangential snap and static-friction dead-zone in BurstCloth
    - filter scene collider candidates more conservatively in ClothPlugin to reduce irrelevant contact noise
```

```text
perf: optimize sensor processing loops and reduce lock contention

    - Sonar: optimize raycasting by caching persistent NativeArrays for commands and hits to eliminate per-frame allocations and reduce GC pressure in FixedUpdate
    - Contact: refactor collision processing to group contact points by unique collision pairs and remove redundant resizing and allocations in the loop
    - JointState: minimize lock duration in the TX thread by buffering physics samples and moving interpolation work outside the critical section
```

## Build and test context

- ROS 2 distro: Jazzy
- Typical workspace root for builds: `~/cloisim_ws`
- Common build target: `cloisim_ros_bringup`

## Additional guidance

Use `.github/instructions/*.instructions.md` for targeted coding rules, `.github/prompts/*.prompt.md` for reusable prompts, and `.github/skills/**/SKILL.md` for structured skill documents.
