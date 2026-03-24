# Stuart control overhaul

This project was updated to shift from raw dead-reckoning-only motion toward a cell-by-cell navigation stack.

## What changed
- Added safer low-level motion control in `src/motors.cpp`
  - true zero command handling
  - separate left/right breakaway PWM support
  - action timeout and stall timeout
  - kick-start boost when target RPM is nonzero but wheel RPM stays near zero
  - low-level loop runs only while an action is active
- Added missing gyro compatibility functions in `src/heading.cpp`
- Added `maze_nav` high-level navigator
  - discrete maze pose: `(row, col, heading)`
  - queue of primitive commands: forward 1 cell / left / right / turnaround / observe
  - wall observation from ToF sensors
  - discovered maze storage
  - plan-to-center BFS over discovered maze with unknown edges treated as traversable
  - pose + maze JSON endpoints for offboard polling
- Updated `src/main.cpp`
  - navigator-driven command dispatch
  - restart now resets pose/map instead of launching a hard-coded demo
- Updated `src/server.cpp`
  - command sequence endpoint `/exec?seq=F,R,F`
  - plan endpoint `/plan_center`
  - map endpoint `/maze`
  - state endpoint `/state`
  - observe endpoint `/observe`
  - reset pose endpoint `/reset_pose`

## Command syntax
Use `/exec?seq=...` or the UI input.

Supported tokens:
- `F` or `F3` -> move forward 1 or 3 cells (always queued as 1-cell primitives)
- `R` or `R180` -> turn right 90 or custom positive degrees
- `L` or `L90` -> turn left 90
- `U` or `B` -> turn around 180
- `O` -> observe current cell without moving

Examples:
- `F,R,F,L,F`
- `F3,R,F2`
- `O,F,R,F,O`

## Offboard telemetry
- `/pose` -> current discrete pose
- `/state` -> pose, queue depth, busy state
- `/maze` -> discovered maze walls/visited cells

## Important assumptions
- Cell length is `CELL_SIZE_CM = 18.0`
- Start pose comes from `START_ROW`, `START_COL`, `START_HEADING` in `include/config.h`
- Center goal is any of `(3,3), (3,4), (4,3), (4,4)` using 0-indexed coordinates
- ToF thresholds in `config.h` are first-pass values and may still need tuning on hardware

## First test procedure
1. Boot and verify reset reason + sensor init logs.
2. In the UI, confirm `/state` reports the expected start pose.
3. Run `O` to capture the starting cell walls.
4. Run `F` once and verify one-cell motion.
5. Run `R,F,L,F`.
6. Use `Plan To Center` only after single-cell primitives behave correctly.

