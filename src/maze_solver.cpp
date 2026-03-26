// maze_solver.cpp
//
// Port of the simulation's main.cpp into a tick-driven state machine.
// No blocking calls — every motion command is issued once, then the
// function returns.  mazeSolverUpdate() is called each main-loop
// iteration; it checks motionIsBusy() before advancing state.

#include "maze_solver.h"
#include "motion.h"       // motionExecute, motionIsBusy, motionAbort
#include <map>
#include <set>
#include <queue>
#include <vector>
#include <tuple>
#include <algorithm>
#include <climits>

// ─────────────────────────────────────────────────────────────────
//  Direction helpers
// ─────────────────────────────────────────────────────────────────
enum Direction { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

static const int DX[]         = { 0,  1,  0, -1 };
static const int DY[]         = { 1,  0, -1,  0 };
static const char DIR_CHAR[] = { 'n','e','s','w' };

// ─────────────────────────────────────────────────────────────────
//  Internal state
// ─────────────────────────────────────────────────────────────────

static int       robotX   = 0;
static int       robotY   = 0;
static Direction robotDir = NORTH;

static int mazeW = 16;
static int mazeH = 16;

static std::map<std::pair<int,int>, std::vector<std::pair<int,int>>> graph;
static std::set<std::pair<int,int>>             visitedCells;
static std::vector<std::pair<int,int>>           backtackStack; 

static MazeSolverHandlers H;

enum class Phase {
    IDLE,
    TRAVERSE_SCAN,
    TRAVERSE_PATH_STEP,
    TRAVERSE_ARRIVING,
    RETURN_PATH_STEP,
    RETURN_ARRIVING,
    SPEEDRUN_PATH_STEP,
    SPEEDRUN_ARRIVING,
    DONE,
    ABORTED,
};

static Phase phase = Phase::IDLE;

static std::vector<Direction> activePath;
static size_t                activePathIdx = 0;

enum class StepState { IDLE, TURNING, MOVING, WAITING_MOTION };
static StepState stepState = StepState::IDLE;

// ─────────────────────────────────────────────────────────────────
//  Internal helpers
// ─────────────────────────────────────────────────────────────────

static void solverLog(const String& msg) {
    if (H.log) H.log(msg.c_str());
}

static void safeSetText(int x, int y, const char* t) {
    if (H.setText) H.setText(x, y, t);
}
static void safeSetColor(int x, int y, char c) {
    if (H.setColor) H.setColor(x, y, c);
}
static void safeSetWall(int x, int y, char d) {
    if (H.setWall) H.setWall(x, y, d);
}

static void ensureNode(std::pair<int,int> p) {
    if (graph.find(p) == graph.end())
        graph[p] = std::vector<std::pair<int,int>>();
}

static void setWallEntry(int x, int y, int dir, bool isWall) {
    int nx = x + DX[dir];
    int ny = y + DY[dir];
    if (nx < 0 || nx >= mazeW || ny < 0 || ny >= mazeH) return;

    std::pair<int,int> from = {x, y};
    std::pair<int,int> to   = {nx, ny};
    ensureNode(from);
    ensureNode(to);

    if (!isWall) {
        auto& fv = graph[from];
        if (std::find(fv.begin(), fv.end(), to) == fv.end())
            fv.push_back(to);
        auto& tv = graph[to];
        if (std::find(tv.begin(), tv.end(), from) == tv.end())
            tv.push_back(from);
    }
}

static bool hasWall(int x, int y, int dir) {
    int nx = x + DX[dir];
    int ny = y + DY[dir];
    if (nx < 0 || nx >= mazeW || ny < 0 || ny >= mazeH) return true;

    if (x == robotX && y == robotY) {
        int rel = (dir - robotDir + 4) % 4;
        if (rel == 0) return H.wallFront ? H.wallFront() : true;
        if (rel == 1) return H.wallRight ? H.wallRight() : true;
        if (rel == 3) return H.wallLeft  ? H.wallLeft()  : true;
        return false;
    }

    auto it = graph.find({x, y});
    if (it == graph.end()) return false; 
    const auto& nbrs = it->second;
    return std::find(nbrs.begin(), nbrs.end(), std::make_pair(nx, ny)) == nbrs.end();
}

static void scanAndStoreWalls() {
    bool front = H.wallFront ? H.wallFront() : false;
    bool left  = H.wallLeft  ? H.wallLeft()  : false;
    bool right = H.wallRight ? H.wallRight() : false;

    int frontDir = robotDir;
    int leftDir  = (robotDir + 3) % 4;
    int rightDir = (robotDir + 1) % 4;

    setWallEntry(robotX, robotY, frontDir, front);
    setWallEntry(robotX, robotY, leftDir,  left);
    setWallEntry(robotX, robotY, rightDir, right);

    auto mirrorWall = [&](int dir, bool wall) {
        if (!wall) return;
        safeSetWall(robotX, robotY, DIR_CHAR[dir]);
        int nx = robotX + DX[dir];
        int ny = robotY + DY[dir];
        if (nx >= 0 && nx < mazeW && ny >= 0 && ny < mazeH)
            setWallEntry(nx, ny, (dir + 2) % 4, true);
    };
    mirrorWall(frontDir, front);
    mirrorWall(leftDir,  left);
    mirrorWall(rightDir, right);
}

static void pushUnvisitedNeighbors() {
    for (int i = 0; i < 4; i++) {
        if (!hasWall(robotX, robotY, i)) {
            int nx = robotX + DX[i];
            int ny = robotY + DY[i];
            std::pair<int,int> np = {nx, ny};
            if (visitedCells.find(np) == visitedCells.end())
                backtackStack.push_back(np);
        }
    }
}

// ── Path-finding ──────────────────────────────────────────────────

static std::vector<Direction> getPath(std::pair<int,int> start, std::pair<int,int> end,
                                  const std::set<std::pair<int,int>>& allowed)
{
    std::queue<std::tuple<std::pair<int,int>, std::vector<Direction>>> q;
    q.push(std::make_tuple(start, std::vector<Direction>()));
    std::set<std::pair<int,int>> seen;
    seen.insert(start);

    while (!q.empty()) {
        auto [cur, path] = q.front(); q.pop();
        if (cur == end) return path;

        for (int i = 0; i < 4; i++) {
            int nx = cur.first  + DX[i];
            int ny = cur.second + DY[i];
            if (nx < 0 || nx >= mazeW || ny < 0 || ny >= mazeH) continue;
            std::pair<int,int> np = {nx, ny};
            if (seen.count(np)) continue;
            if (!allowed.count(np)) continue;
            if (hasWall(cur.first, cur.second, i)) continue;
            seen.insert(np);
            std::vector<Direction> np2 = path;
            np2.push_back((Direction)i);
            q.push(std::make_tuple(np, np2));
        }
    }
    return {};
}

static std::vector<std::pair<int,int>> dijkstra(std::pair<int,int> start, std::pair<int,int> end) {
    if (!graph.count(start) || !graph.count(end)) return {};

    std::map<std::pair<int,int>, int>             dist;
    std::map<std::pair<int,int>, std::pair<int,int>>   prev;

    for (auto& [node, _] : graph) dist[node] = INT_MAX;
    dist[start] = 0;

    using PQEntry = std::pair<int, std::pair<int,int>>;
    std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<PQEntry>> pq;
    pq.push(std::make_pair(0, start));

    while (!pq.empty()) {
        auto [d, cur] = pq.top(); pq.pop();
        if (d > dist[cur]) continue;
        if (cur == end) {
            std::vector<std::pair<int,int>> path;
            std::pair<int,int> n = end;
            while (prev.count(n)) { path.push_back(n); n = prev[n]; }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }
        for (auto& nb : graph[cur]) {
            if (!dist.count(nb)) dist[nb] = INT_MAX;
            int nd = d + 1;
            if (nd < dist[nb]) {
                dist[nb] = nd;
                prev[nb] = cur;
                pq.push(std::make_pair(nd, nb));
            }
        }
    }
    return {};
}

static std::vector<Direction> pathToDirections(const std::vector<std::pair<int,int>>& path) {
    std::vector<Direction> dirs;
    for (size_t i = 0; i + 1 < path.size(); i++) {
        int dx = path[i+1].first  - path[i].first;
        int dy = path[i+1].second - path[i].second;
        for (int d = 0; d < 4; d++) {
            if (DX[d] == dx && DY[d] == dy) {
                dirs.push_back((Direction)d);
                break;
            }
        }
    }
    return dirs;
}

static String buildMotionString(const std::vector<Direction>& dirs) {
    if (dirs.empty()) return "";
    String s = "";

    size_t i = 0;
    while (i < dirs.size()) {
        Direction cur = dirs[i];
        int count = 0;
        while (i < dirs.size() && dirs[i] == cur) { count++; i++; }

        int diff = ((int)cur - (int)robotDir + 4) % 4;
        if      (diff == 1) s += "R,";
        else if (diff == 3) s += "L,";
        else if (diff == 2) s += "R,R,";

        robotDir = cur;
        s += "F";
        if (count > 1) s += String(count);
        s += ",";
    }

    if (s.endsWith(",")) s.remove(s.length() - 1);
    return s;
}

static void applyPathToRobotPose(const std::vector<Direction>& path) {
    for (Direction d : path) {
        robotX += DX[(int)d];
        robotY += DY[(int)d];
    }
}

static void executeActivePath(Phase arrivingPhase) {
    if (activePath.empty()) {
        phase = arrivingPhase;
        return;
    }

    String cmd = buildMotionString(activePath);
    solverLog("Motion: " + cmd);

    if (!motionExecute(cmd)) {
        solverLog("[Solver] motionExecute failed — aborting.");
        phase = Phase::ABORTED;
        return;
    }

    phase = arrivingPhase;
    activePathIdx = 0;
}

// ─────────────────────────────────────────────────────────────────
//  Public API
// ─────────────────────────────────────────────────────────────────

void mazeSolverInit(const MazeSolverHandlers& handlers) {
    H = handlers;
    mazeW = H.mazeWidth  ? H.mazeWidth()  : 16;
    mazeH = H.mazeHeight ? H.mazeHeight() : 16;

    graph.clear();
    visitedCells.clear();
    backtackStack.clear();
    activePath.clear();

    robotX   = 0;
    robotY   = 0;
    robotDir = NORTH;

    phase     = Phase::IDLE;
    stepState = StepState::IDLE;
}

bool mazeSolverBusy() {
    return phase != Phase::IDLE && phase != Phase::DONE && phase != Phase::ABORTED;
}

void mazeSolverAbort() {
    motionAbort();
    phase     = Phase::ABORTED;
    stepState = StepState::IDLE;
    activePath.clear();
    backtackStack.clear();
}

int mazeSolverVisitedCount() {
    return (int)visitedCells.size();
}

bool mazeSolverUpdate() {
    switch (phase) {
    case Phase::IDLE:
        robotX = 0; robotY = 0; robotDir = NORTH;
        visitedCells.insert({0, 0});
        safeSetText(0, 0, "V");
        phase = Phase::TRAVERSE_SCAN;
        return true;

    case Phase::TRAVERSE_SCAN: {
        scanAndStoreWalls();
        pushUnvisitedNeighbors();

        while (!backtackStack.empty() && visitedCells.count(backtackStack.back()))
            backtackStack.pop_back();

        if (backtackStack.empty()) {
            solverLog("Exploration done. Returning home.");
            std::pair<int,int> home = {0, 0};
            activePath = getPath({robotX, robotY}, home, visitedCells);
            executeActivePath(Phase::RETURN_ARRIVING);
            return true;
        }

        std::pair<int,int> target = backtackStack.back();
        backtackStack.pop_back();

        std::set<std::pair<int,int>> allowed = visitedCells;
        allowed.insert(target);

        activePath = getPath({robotX, robotY}, target, allowed);
        if (activePath.empty()) return true;

        executeActivePath(Phase::TRAVERSE_ARRIVING);
        return true;
    }

    case Phase::TRAVERSE_ARRIVING:
        if (motionIsBusy()) return true;
        applyPathToRobotPose(activePath);
        activePath.clear();
        visitedCells.insert({robotX, robotY});
        safeSetText(robotX, robotY, "V");
        phase = Phase::TRAVERSE_SCAN;
        return true;

    case Phase::RETURN_ARRIVING:
        if (motionIsBusy()) return true;
        applyPathToRobotPose(activePath);
        activePath.clear();
        {
            std::pair<int,int> start  = {0, 0};
            std::pair<int,int> center = {mazeW / 2, mazeH / 2};
            std::vector<std::pair<int,int>> sp = dijkstra(start, center);
            if (sp.empty()) {
                phase = Phase::DONE;
                return false;
            }
            for (auto& c : sp) {
                safeSetText(c.first, c.second, "P");
                safeSetColor(c.first, c.second, 'G');
            }
            activePath = pathToDirections(sp);
            executeActivePath(Phase::SPEEDRUN_ARRIVING);
        }
        return true;

    case Phase::SPEEDRUN_ARRIVING:
        if (motionIsBusy()) return true;
        applyPathToRobotPose(activePath);
        activePath.clear();
        phase = Phase::DONE;
        return false;

    default:
        return false;
    }
}