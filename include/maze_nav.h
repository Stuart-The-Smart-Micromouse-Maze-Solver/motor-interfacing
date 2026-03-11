#pragma once
#include <Arduino.h>
#include "config.h"

namespace maze_nav {

enum Heading : uint8_t { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

enum CommandType : uint8_t { CMD_NONE = 0, CMD_FORWARD_1, CMD_TURN_LEFT, CMD_TURN_RIGHT, CMD_TURN_AROUND, CMD_OBSERVE };

struct Pose {
  int row;
  int col;
  Heading heading;
};

void init();
void resetToStart();
void clearQueue();

bool enqueueForward(int cells);
bool enqueueTurn(float deg);
bool enqueueSequence(const String& seq);
bool planToCenter();

void tick();
void observeCurrentCell();

bool isBusy();
Pose getPose();

String poseJson();
String stateJson();
String mazeJson();

const char* headingName(Heading h);

} // namespace maze_nav
