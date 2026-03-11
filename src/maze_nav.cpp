#include "maze_nav.h"

#include <Arduino.h>
#include <vector>

#include "motors.h"
#include "distance.h"
#include "encoders.h"

namespace maze_nav {

namespace {
constexpr uint8_t WALL_N = 1 << 0;
constexpr uint8_t WALL_E = 1 << 1;
constexpr uint8_t WALL_S = 1 << 2;
constexpr uint8_t WALL_W = 1 << 3;

struct CellInfo {
  uint8_t knownMask = 0;
  uint8_t wallMask = 0;
  bool visited = false;
};

struct Command {
  CommandType type = CMD_NONE;
};

struct InFlight {
  bool active = false;
  CommandType type = CMD_NONE;
};

CellInfo g_cells[MAZE_SIZE][MAZE_SIZE];
Pose g_pose{START_ROW, START_COL, static_cast<Heading>(START_HEADING)};
Command g_queue[NAV_QUEUE_LEN];
int g_qHead = 0;
int g_qTail = 0;
int g_qCount = 0;
InFlight g_inflight;
bool g_autoPlanToCenter = false;
uint32_t g_lastObserveMs = 0;

static int dRow(Heading h) {
  switch (h) {
    case NORTH: return -1;
    case EAST:  return 0;
    case SOUTH: return 1;
    case WEST:  return 0;
  }
  return 0;
}

static int dCol(Heading h) {
  switch (h) {
    case NORTH: return 0;
    case EAST:  return 1;
    case SOUTH: return 0;
    case WEST:  return -1;
  }
  return 0;
}

static uint8_t wallBit(Heading h) {
  switch (h) {
    case NORTH: return WALL_N;
    case EAST:  return WALL_E;
    case SOUTH: return WALL_S;
    case WEST:  return WALL_W;
  }
  return 0;
}

static Heading opposite(Heading h) {
  return static_cast<Heading>((static_cast<int>(h) + 2) % 4);
}

static Heading turnLeftOf(Heading h) {
  return static_cast<Heading>((static_cast<int>(h) + 3) % 4);
}

static Heading turnRightOf(Heading h) {
  return static_cast<Heading>((static_cast<int>(h) + 1) % 4);
}

static bool inBounds(int row, int col) {
  return row >= 0 && row < MAZE_SIZE && col >= 0 && col < MAZE_SIZE;
}

static void markBoundaryWalls() {
  for (int r = 0; r < MAZE_SIZE; ++r) {
    for (int c = 0; c < MAZE_SIZE; ++c) {
      if (r == 0) {
        g_cells[r][c].knownMask |= WALL_N;
        g_cells[r][c].wallMask |= WALL_N;
      }
      if (r == MAZE_SIZE - 1) {
        g_cells[r][c].knownMask |= WALL_S;
        g_cells[r][c].wallMask |= WALL_S;
      }
      if (c == 0) {
        g_cells[r][c].knownMask |= WALL_W;
        g_cells[r][c].wallMask |= WALL_W;
      }
      if (c == MAZE_SIZE - 1) {
        g_cells[r][c].knownMask |= WALL_E;
        g_cells[r][c].wallMask |= WALL_E;
      }
    }
  }
}

static void setWallKnown(int row, int col, Heading dir, bool hasWall) {
  if (!inBounds(row, col)) return;

  uint8_t bit = wallBit(dir);
  g_cells[row][col].knownMask |= bit;
  if (hasWall) g_cells[row][col].wallMask |= bit;
  else         g_cells[row][col].wallMask &= ~bit;

  int nr = row + dRow(dir);
  int nc = col + dCol(dir);
  if (!inBounds(nr, nc)) return;

  uint8_t obit = wallBit(opposite(dir));
  g_cells[nr][nc].knownMask |= obit;
  if (hasWall) g_cells[nr][nc].wallMask |= obit;
  else         g_cells[nr][nc].wallMask &= ~obit;
}

static bool enqueue(CommandType type) {
  if (g_qCount >= NAV_QUEUE_LEN) return false;
  g_queue[g_qTail].type = type;
  g_qTail = (g_qTail + 1) % NAV_QUEUE_LEN;
  ++g_qCount;
  return true;
}

static bool dequeue(Command &cmd) {
  if (g_qCount <= 0) return false;
  cmd = g_queue[g_qHead];
  g_qHead = (g_qHead + 1) % NAV_QUEUE_LEN;
  --g_qCount;
  return true;
}

static bool hasKnownWall(int row, int col, Heading dir, bool &known, bool &wall) {
  if (!inBounds(row, col)) {
    known = true;
    wall = true;
    return true;
  }
  uint8_t bit = wallBit(dir);
  known = (g_cells[row][col].knownMask & bit) != 0;
  wall = (g_cells[row][col].wallMask & bit) != 0;
  return known;
}

static void advancePoseOneCell() {
  int prevRow = g_pose.row;
  int prevCol = g_pose.col;
  int nr = g_pose.row + dRow(g_pose.heading);
  int nc = g_pose.col + dCol(g_pose.heading);
  if (!inBounds(nr, nc)) return;
  setWallKnown(prevRow, prevCol, g_pose.heading, false);
  g_pose.row = nr;
  g_pose.col = nc;
  g_cells[g_pose.row][g_pose.col].visited = true;
}

static bool immediateFrontBlocked() {
  bool known = false;
  bool wall = false;
  hasKnownWall(g_pose.row, g_pose.col, g_pose.heading, known, wall);
  if (known && wall) return true;

  int frontMM = getDistanceFront();
  if (frontMM > 0 && frontMM < FRONT_BLOCK_THRESHOLD_MM) {
    setWallKnown(g_pose.row, g_pose.col, g_pose.heading, true);
    return true;
  }
  return false;
}

static bool passable(int row, int col, Heading dir, bool allowUnknown) {
  if (!inBounds(row, col)) return false;
  int nr = row + dRow(dir);
  int nc = col + dCol(dir);
  if (!inBounds(nr, nc)) return false;

  bool known = false;
  bool wall = false;
  hasKnownWall(row, col, dir, known, wall);
  if (!known) return allowUnknown;
  return !wall;
}

static bool isGoalCell(int row, int col) {
  return (row == 3 || row == 4) && (col == 3 || col == 4);
}

static void appendTurnCommands(Heading from, Heading to) {
  int delta = (static_cast<int>(to) - static_cast<int>(from) + 4) % 4;
  if (delta == 0) return;
  if (delta == 1) enqueue(CMD_TURN_RIGHT);
  else if (delta == 3) enqueue(CMD_TURN_LEFT);
  else {
    enqueue(CMD_TURN_AROUND);
  }
}

static bool replanToCenterInternal() {
  struct ParentNode {
    int prevR = -1;
    int prevC = -1;
    Heading via = NORTH;
    bool seen = false;
  };

  ParentNode parent[MAZE_SIZE][MAZE_SIZE];
  std::vector<std::pair<int,int>> q;
  q.reserve(MAZE_SIZE * MAZE_SIZE);

  q.push_back({g_pose.row, g_pose.col});
  parent[g_pose.row][g_pose.col].seen = true;

  size_t qi = 0;
  int goalR = -1, goalC = -1;
  while (qi < q.size()) {
    auto rc = q[qi++];
    int r = rc.first;
    int c = rc.second;
    if (isGoalCell(r, c)) {
      goalR = r;
      goalC = c;
      break;
    }
    for (int d = 0; d < 4; ++d) {
      Heading dir = static_cast<Heading>(d);
      if (!passable(r, c, dir, true)) continue;
      int nr = r + dRow(dir);
      int nc = c + dCol(dir);
      if (parent[nr][nc].seen) continue;
      parent[nr][nc].seen = true;
      parent[nr][nc].prevR = r;
      parent[nr][nc].prevC = c;
      parent[nr][nc].via = dir;
      q.push_back({nr, nc});
    }
  }

  if (goalR < 0) return false;

  std::vector<Heading> steps;
  int cr = goalR, cc = goalC;
  while (!(cr == g_pose.row && cc == g_pose.col)) {
    ParentNode &pn = parent[cr][cc];
    steps.push_back(pn.via);
    int pr = pn.prevR;
    int pc = pn.prevC;
    cr = pr;
    cc = pc;
  }

  g_qHead = g_qTail = g_qCount = 0;
  Heading heading = g_pose.heading;
  for (auto it = steps.rbegin(); it != steps.rend(); ++it) {
    appendTurnCommands(heading, *it);
    heading = *it;
    enqueue(CMD_FORWARD_1);
  }
  return true;
}

} // namespace

const char* headingName(Heading h) {
  switch (h) {
    case NORTH: return "N";
    case EAST:  return "E";
    case SOUTH: return "S";
    case WEST:  return "W";
  }
  return "?";
}

void init() {
  resetToStart();
}

void resetToStart() {
  clearQueue();
  g_inflight = {};
  g_autoPlanToCenter = false;
  g_pose = {START_ROW, START_COL, static_cast<Heading>(START_HEADING)};
  for (int r = 0; r < MAZE_SIZE; ++r) {
    for (int c = 0; c < MAZE_SIZE; ++c) {
      g_cells[r][c] = {};
    }
  }
  markBoundaryWalls();
  g_cells[g_pose.row][g_pose.col].visited = true;
  g_lastObserveMs = 0;
}

void clearQueue() {
  g_qHead = g_qTail = g_qCount = 0;
  g_inflight = {};
  g_autoPlanToCenter = false;
}

bool enqueueForward(int cells) {
  if (cells == 0) return true;
  int steps = abs(cells);
  if (cells < 0) return false;
  for (int i = 0; i < steps; ++i) {
    if (!enqueue(CMD_FORWARD_1)) return false;
  }
  return true;
}

bool enqueueTurn(float deg) {
  int ideg = static_cast<int>(deg);
  if (ideg == 0) return true;
  if (ideg == 180 || ideg == -180) return enqueue(CMD_TURN_AROUND);
  if (ideg > 0) {
    int n = (ideg + 45) / 90;
    for (int i = 0; i < n; ++i) if (!enqueue(CMD_TURN_RIGHT)) return false;
    return true;
  }
  int n = ((-ideg) + 45) / 90;
  for (int i = 0; i < n; ++i) if (!enqueue(CMD_TURN_LEFT)) return false;
  return true;
}

bool enqueueSequence(const String& seq) {
  String s = seq;
  s.trim();
  if (s.length() == 0) return true;

  int i = 0;
  while (i < s.length()) {
    while (i < s.length() && (s[i] == ' ' || s[i] == ',' || s[i] == ';')) ++i;
    if (i >= s.length()) break;

    char op = toupper(s[i++]);
    int sign = 1;
    if (i < s.length() && s[i] == '-') { sign = -1; ++i; }
    int value = 0;
    bool hasValue = false;
    while (i < s.length() && isDigit(static_cast<unsigned char>(s[i]))) {
      value = value * 10 + (s[i] - '0');
      hasValue = true;
      ++i;
    }
    value *= sign;

    switch (op) {
      case 'F':
      case 'M':
        if (!enqueueForward(hasValue ? value : 1)) return false;
        break;
      case 'R':
        if (!enqueueTurn(hasValue ? value : 90)) return false;
        break;
      case 'L':
        if (!enqueueTurn(hasValue ? value : -90)) return false;
        break;
      case 'U':
      case 'B':
        if (!enqueue(CMD_TURN_AROUND)) return false;
        break;
      case 'O':
        if (!enqueue(CMD_OBSERVE)) return false;
        break;
      default:
        return false;
    }
  }
  return true;
}

bool planToCenter() {
  observeCurrentCell();
  g_autoPlanToCenter = true;
  return replanToCenterInternal();
}

void observeCurrentCell() {
  if (!inBounds(g_pose.row, g_pose.col)) return;
  g_cells[g_pose.row][g_pose.col].visited = true;

  auto applyReading = [&](Heading dir, int mm, int maxMm) {
    if (mm <= 0 || mm > maxMm) return;
    bool hasWall = mm < WALL_PRESENT_THRESHOLD_MM;
    setWallKnown(g_pose.row, g_pose.col, dir, hasWall);
  };

  applyReading(turnLeftOf(g_pose.heading), getDistanceLeft(), SIDE_OBSERVE_MAX_MM);
  applyReading(g_pose.heading, getDistanceFront(), FRONT_OBSERVE_MAX_MM);
  applyReading(turnRightOf(g_pose.heading), getDistanceRight(), SIDE_OBSERVE_MAX_MM);

  g_lastObserveMs = millis();
}

bool isBusy() {
  return g_inflight.active || g_qCount > 0 || motors::isInAction;
}

Pose getPose() {
  return g_pose;
}

void tick() {
  if (g_inflight.active && !motors::isInAction) {
    switch (g_inflight.type) {
      case CMD_FORWARD_1:
        advancePoseOneCell();
        observeCurrentCell();
        break;
      case CMD_TURN_LEFT:
        g_pose.heading = turnLeftOf(g_pose.heading);
        observeCurrentCell();
        break;
      case CMD_TURN_RIGHT:
        g_pose.heading = turnRightOf(g_pose.heading);
        observeCurrentCell();
        break;
      case CMD_TURN_AROUND:
        g_pose.heading = opposite(g_pose.heading);
        observeCurrentCell();
        break;
      default:
        break;
    }
    g_inflight = {};

    if (g_autoPlanToCenter && isGoalCell(g_pose.row, g_pose.col)) {
      clearQueue();
      g_autoPlanToCenter = false;
    }
  }

  if (motors::isInAction || g_inflight.active) return;

  Command cmd;
  if (!dequeue(cmd)) {
    return;
  }

  if (cmd.type == CMD_OBSERVE) {
    observeCurrentCell();
    return;
  }

  if (cmd.type == CMD_FORWARD_1) {
    observeCurrentCell();
    if (immediateFrontBlocked()) {
      if (g_autoPlanToCenter) {
        replanToCenterInternal();
      }
      return;
    }
    motors::setTargetPosition(CELL_SIZE_CM);
    g_inflight.active = true;
    g_inflight.type = cmd.type;
    return;
  }

  if (cmd.type == CMD_TURN_LEFT) {
    motors::setTargetRotation(-90.0f);
    g_inflight.active = true;
    g_inflight.type = cmd.type;
    return;
  }

  if (cmd.type == CMD_TURN_RIGHT) {
    motors::setTargetRotation(90.0f);
    g_inflight.active = true;
    g_inflight.type = cmd.type;
    return;
  }

  if (cmd.type == CMD_TURN_AROUND) {
    motors::setTargetRotation(180.0f);
    g_inflight.active = true;
    g_inflight.type = cmd.type;
    return;
  }
}

String poseJson() {
  String out = "{";
  out += "\"row\":" + String(g_pose.row) + ",";
  out += "\"col\":" + String(g_pose.col) + ",";
  out += "\"heading\":\"" + String(headingName(g_pose.heading)) + "\",";
  out += "\"heading_idx\":" + String((int)g_pose.heading) + "}";
  return out;
}

String stateJson() {
  String out = "{";
  out += "\"pose\":" + poseJson() + ",";
  out += "\"queue\":" + String(g_qCount) + ",";
  out += "\"busy\":" + String(isBusy() ? "true" : "false") + ",";
  out += "\"motors_busy\":" + String(motors::isInAction ? "true" : "false") + ",";
  out += "\"last_observe_ms\":" + String(g_lastObserveMs) + ",";
  out += "\"goal_center\":[[3,3],[3,4],[4,3],[4,4]]";
  out += "}";
  return out;
}

String mazeJson() {
  String out = "{";
  out += "\"size\":" + String(MAZE_SIZE) + ",";
  out += "\"pose\":" + poseJson() + ",";
  out += "\"cells\":[";
  for (int r = 0; r < MAZE_SIZE; ++r) {
    if (r) out += ",";
    out += "[";
    for (int c = 0; c < MAZE_SIZE; ++c) {
      if (c) out += ",";
      out += "{";
      out += "\"known\":" + String(g_cells[r][c].knownMask) + ",";
      out += "\"walls\":" + String(g_cells[r][c].wallMask) + ",";
      out += "\"visited\":" + String(g_cells[r][c].visited ? "true" : "false");
      out += "}";
    }
    out += "]";
  }
  out += "]}";
  return out;
}

} // namespace maze_nav
