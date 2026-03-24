#pragma once
#include <Arduino.h>

// ═══════════════════════════════════════════════════════════════════
//  Maze instruction executor
//
//  Accepts a command string like "F,R,F,L,F,F" and executes each
//  primitive sequentially using the motors:: cascade PID system.
//
//  Supported tokens:
//    F   or F3  — move forward 1 or N cells (18cm each)
//    R   or R90 — turn right 90° (or custom degrees)
//    L   or L90 — turn left 90°
//    U   or B   — turn around 180°
//    W   or W500— wait 200ms (or custom ms)
//
//  Example: "F3,R,F2,L,F,U,F4"
// ═══════════════════════════════════════════════════════════════════

void motionInit();

// Parse and queue an instruction string. Returns true if parsed OK.
bool motionExecute(const String& instructions);

// Call every loop iteration to advance the state machine.
void motionUpdate();

// True if currently executing a sequence.
bool motionIsBusy();

// Abort current sequence and stop motors.
void motionAbort();

// Number of commands remaining in queue (including current).
int motionQueueRemaining();
