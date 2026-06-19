---
description: Harness Engineering commit message policy for cloisim_ros.
applyTo: "**"
---

# Harness Engineering Commit Message Policy

- For Harness Engineering work, git commit messages must be written in English only.
- Use this format:
  - first line: `type(scope): summary` or `type: summary`
  - following lines: indented bullet list with `- ` items describing concrete changes and effects
- Keep the subject concise and technical.
- Do not write Harness Engineering commit messages in Korean.

## Example 1

```text
fix(cloth): improve resting stability on multi-support contacts
    - include contact threshold in ClothGrabberPlugin cache key to avoid stale group activation results
    - reuse cached collider bounds in ClothGrabber to reduce repeated hot-path contact work
    - skip BurstCloth collider updates when collider state is unchanged to avoid needless wake-ups
    - make sleep detection tolerate persistent low-energy resting contact
    - strengthen resting-contact tangential snap and static-friction dead-zone in BurstCloth
    - filter scene collider candidates more conservatively in ClothPlugin to reduce irrelevant contact noise
```

## Example 2

```text
perf: optimize sensor processing loops and reduce lock contention

    - Sonar: optimize raycasting by caching persistent NativeArrays for commands and hits to eliminate per-frame allocations and reduce GC pressure in FixedUpdate
    - Contact: refactor collision processing to group contact points by unique collision pairs and remove redundant resizing and allocations in the loop
    - JointState: minimize lock duration in the TX thread by buffering physics samples and moving interpolation work outside the critical section
```
