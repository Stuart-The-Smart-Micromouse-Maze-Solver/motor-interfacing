#pragma once
#include <Arduino.h>
#include <functional>
#include <vector>
#include <utility>

// ═══════════════════════════════════════════════════════════════════
//  MazeSolverHandlers
//
//  Fill these in before calling mazeSolverInit().
//  They replace all API.h calls from the simulation, letting the
//  solver work with your real motion and sensor stack.
//
//  Motion handlers must be non-blocking: they kick off a move and
//  return immediately. mazeSolverUpdate() checks motionIsBusy()
//  before advancing to the next step.
// ═══════════════════════════════════════════════════════════════════
struct MazeSolverHandlers {
    // ── Motion (non-blocking — must return before move completes) ──
    std::function<void()>        moveForward;   // move 1 cell forward
    std::function<void()>        turnLeft;      // turn 90° left in place
    std::function<void()>        turnRight;     // turn 90° right in place

    // ── Wall sensing (called while stationary after each move) ────
    std::function<bool()>        wallFront;
    std::function<bool()>        wallLeft;
    std::function<bool()>        wallRight;

    // ── Maze dimensions ───────────────────────────────────────────
    std::function<int()>         mazeWidth;
    std::function<int()>         mazeHeight;

    // ── Optional: simulator display helpers (safe to leave null) ──
    std::function<void(int x, int y, const char* text)>  setText;
    std::function<void(int x, int y, char color)>        setColor;
    std::function<void(int x, int y, char wallDir)>      setWall;   // 'n','e','s','w'
    std::function<void(const char* msg)>                 log;
};

// ── Lifecycle ─────────────────────────────────────────────────────
void mazeSolverInit(const MazeSolverHandlers& handlers);

// Call once per main-loop iteration (alongside motors::tick and motionUpdate).
// Returns true while the solver still has work to do.
bool mazeSolverUpdate();

// True while traversal or speed-run is in progress.
bool mazeSolverBusy();

// Abort and reset all internal state.
void mazeSolverAbort();

// ── Optional query helpers ────────────────────────────────────────
// How many cells have been visited so far.
int  mazeSolverVisitedCount();