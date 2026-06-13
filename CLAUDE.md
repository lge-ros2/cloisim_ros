# cloisim_ros Claude Code Guidance

## Scope

This file exists so Claude Code has a stable, explicit project instruction entrypoint.

## Repository role

`cloisim_ros` is a ROS 2 Jazzy workspace that bridges CLOiSim simulator devices to ROS 2 nodes over ZMQ and WebSocket.

## Working rules

- Prefer narrow, local fixes over broad refactors.
- Keep C++ compatible with the repository's current standard and warning settings.
- Preserve existing ROS 2 package boundaries.
- Do not mix behavior changes with unrelated cleanup.
- When running ROS 2 build or test commands, use the intended workspace root for this repository setup.

## Harness Engineering commit messages

For Harness Engineering work, git commit messages must be written in English only.

Use this format:
- first line: `type(scope): summary` or `type: summary`
- following lines: indented bullet list with `- ` items describing concrete changes and effects

Do not write Harness Engineering commit messages in Korean.

### Example 1

```text
fix(cloth): improve resting stability on multi-support contacts
    - include contact threshold in ClothGrabberPlugin cache key to avoid stale group activation results
    - reuse cached collider bounds in ClothGrabber to reduce repeated hot-path contact work
    - skip BurstCloth collider updates when collider state is unchanged to avoid needless wake-ups
    - make sleep detection tolerate persistent low-energy resting contact
    - strengthen resting-contact tangential snap and static-friction dead-zone in BurstCloth
    - filter scene collider candidates more conservatively in ClothPlugin to reduce irrelevant contact noise
```

### Example 2

```text
perf: optimize sensor processing loops and reduce lock contention

    - Sonar: optimize raycasting by caching persistent NativeArrays for commands and hits to eliminate per-frame allocations and reduce GC pressure in FixedUpdate
    - Contact: refactor collision processing to group contact points by unique collision pairs and remove redundant resizing and allocations in the loop
    - JointState: minimize lock duration in the TX thread by buffering physics samples and moving interpolation work outside the critical section
```

## Copilot-specific instruction sources

For Copilot-targeted instruction loading, prefer the files under `.github/`:
- `.github/copilot-instructions.md`
- `.github/instructions/*.instructions.md`
- `.github/prompts/*.prompt.md`
- `.github/skills/**/SKILL.md`
