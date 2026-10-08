#include "Agent.h"
#include <algorithm>
#include <climits>
#include <cstring>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include "World.h"

using namespace std;

// Grid distance between two cells on this hex layout
static int hexDistance(Point2D first, Point2D second) {
  auto cubeX = [](Point2D point) { return point.x - (point.y - (point.y & 1)) / 2; };
  int deltaX = cubeX(first) - cubeX(second);
  int deltaZ = first.y - second.y;
  int deltaY = -deltaX - deltaZ;
  return max(abs(deltaX), max(abs(deltaY), abs(deltaZ)));
}

// SearchBoard

Agent::SearchBoard::SearchBoard(const CatWorld& world)
    : side(world.getWorldSideSize()),
      half(side / 2),
      walls(world.worldState().begin(), world.worldState().end()),
      edgeCells(side * side, 0),
      neighborTable(side * side) {
  cat = toCell(world.getCat());
  for (int cell = 0; cell < cellCount(); cell++) {
    Point2D point = toPoint(cell);
    edgeCells[cell] = world.catWinsOnSpace(point);
    if (edgeCells[cell]) edgeCellList.push_back(cell);
    vector<Point2D> around = CatWorld::neighbors(point);  // same neighbor order as the game uses
    for (int direction = 0; direction < 6; direction++) {
      neighborTable[cell][direction] = world.isValidPosition(around[direction]) ? toCell(around[direction]) : OFF_BOARD;
      flatNeighbors.push_back(neighborTable[cell][direction]);
    }
  }
  scratchTimesReached.resize(cellCount());
  scratchFrontier.resize(cellCount());
  scratchSteps.resize(cellCount());
}

bool Agent::SearchBoard::hasOpenNeighbor(int cell) const {
  for (int neighbor : neighborsOf(cell))
    if (isOpen(neighbor)) return true;
  return false;
}

vector<int> Agent::SearchBoard::openNeighborsOf(int cell) const {
  vector<int> open;
  for (int neighbor : neighborsOf(cell))
    if (isOpen(neighbor)) open.push_back(neighbor);
  return open;
}

vector<int> Agent::SearchBoard::stepsFrom(const vector<int>& startCells, int neighborsNeeded, int maxSteps) const {
  // This scan runs millions of times per move, so it works directly on plain arrays.
  // In an unoptimized build (which the tournament uses), every std::vector [] or small
  // helper like isOpen() is a real function call, and avoiding them in this one loop
  // makes the whole search several times faster. The visiting order is unchanged.
  int cellTotal = cellCount();
  vector<int> steps(cellTotal);
  int* stepsTo = steps.data();
  for (int cell = 0; cell < cellTotal; cell++) stepsTo[cell] = NOT_REACHABLE;

  int* timesReached = scratchTimesReached.data();
  memset(timesReached, 0, cellTotal * sizeof(int));
  int* frontier = scratchFrontier.data();  // used as a queue: read front to back while appending
  int frontierEnd = 0;
  const unsigned char* isWall = walls.data();
  const int* neighbors = flatNeighbors.data();

  for (int start : startCells) {
    stepsTo[start] = 0;
    frontier[frontierEnd++] = start;
  }
  for (int next = 0; next < frontierEnd; next++) {
    int current = frontier[next];
    if (stepsTo[current] >= maxSteps) continue;  // far enough: don't spread further from here
    const int* around = neighbors + current * 6;
    for (int direction = 0; direction < 6; direction++) {
      int neighbor = around[direction];
      if (neighbor == OFF_BOARD || isWall[neighbor] || stepsTo[neighbor] != NOT_REACHABLE) continue;
      timesReached[neighbor]++;
      if (timesReached[neighbor] < neighborsNeeded) continue;
      stepsTo[neighbor] = stepsTo[current] + 1;
      frontier[frontierEnd++] = neighbor;
    }
  }
  return steps;
}

int Agent::SearchBoard::fewestEscapeStepsAround(int cell, int neighborsNeeded) const {
  // The scan spreads out from the edge in order of distance, so the first of the cell's
  // neighbors it reaches has the fewest steps. Same plain-array style as stepsFrom().
  const int* targets = flatNeighbors.data() + cell * 6;
  int cellTotal = cellCount();
  int* stepsTo = scratchSteps.data();
  for (int index = 0; index < cellTotal; index++) stepsTo[index] = NOT_REACHABLE;
  int* timesReached = scratchTimesReached.data();
  memset(timesReached, 0, cellTotal * sizeof(int));
  int* frontier = scratchFrontier.data();
  int frontierEnd = 0;
  const unsigned char* isWall = walls.data();
  const int* neighbors = flatNeighbors.data();
  auto isTarget = [&](int candidate) {
    for (int direction = 0; direction < 6; direction++)
      if (targets[direction] == candidate) return true;
    return false;
  };

  const int* edges = edgeCellList.data();
  int edgeTotal = (int)edgeCellList.size();
  for (int index = 0; index < edgeTotal; index++) {
    int edge = edges[index];
    if (isWall[edge]) continue;
    if (isTarget(edge)) return 0;  // an open edge cell right next to `cell`
    stepsTo[edge] = 0;
    frontier[frontierEnd++] = edge;
  }
  for (int next = 0; next < frontierEnd; next++) {
    int current = frontier[next];
    const int* around = neighbors + current * 6;
    for (int direction = 0; direction < 6; direction++) {
      int neighbor = around[direction];
      if (neighbor == OFF_BOARD || isWall[neighbor] || stepsTo[neighbor] != NOT_REACHABLE) continue;
      timesReached[neighbor]++;
      if (timesReached[neighbor] < neighborsNeeded) continue;
      stepsTo[neighbor] = stepsTo[current] + 1;
      if (isTarget(neighbor)) return stepsTo[neighbor];
      frontier[frontierEnd++] = neighbor;
    }
  }
  return NOT_REACHABLE;  // none of the neighbors can reach the edge
}

vector<int> Agent::SearchBoard::openEdgeCells() const {
  vector<int> edges;
  for (int cell : edgeCellList)
    if (isOpen(cell)) edges.push_back(cell);
  return edges;
}

int Agent::SearchBoard::roomAround(int cell) const {
  vector<int> steps = stepsFrom({cell}, 1);
  return (int)count_if(steps.begin(), steps.end(), [](int stepCount) { return stepCount != NOT_REACHABLE; });
}



// Simple rules

bool Agent::SpotRating::isBetterThan(const SpotRating& other) const {
  if (isEdge != other.isEdge) return isEdge;
  if (canReachEdge != other.canReachEdge) return canReachEdge;
  if (!canReachEdge) return room > other.room;  // sealed in: more room means surviving longer
  if (guaranteedSteps != other.guaranteedSteps) return guaranteedSteps < other.guaranteedSteps;
  if (shortestSteps != other.shortestSteps) return shortestSteps < other.shortestSteps;
  // Against a fence builder, the fence grows where its walls already are, so head away from them.
  if (distanceFromFence != other.distanceFromFence) return distanceFromFence > other.distanceFromFence;
  return openNeighborCount > other.openNeighborCount;
}

Agent::CatStep Agent::bestStepForCat(const SearchBoard& board) {
  vector<int> guaranteed = board.guaranteedEscapeSteps();
  vector<int> shortest = board.shortestEscapeSteps();
  vector<int> wallsOnEdge;
  for (int cell = 0; cell < board.cellCount(); cell++)
    if (board.isEdge(cell) && !board.isOpen(cell)) wallsOnEdge.push_back(cell);

  CatStep best;
  for (int cell : board.openNeighborsOf(board.catCell())) {
    SpotRating rating;
    rating.isEdge = board.isEdge(cell);
    rating.shortestSteps = shortest[cell];
    rating.canReachEdge = rating.shortestSteps != NOT_REACHABLE;
    rating.guaranteedSteps = guaranteed[cell];
    rating.openNeighborCount = (int)board.openNeighborsOf(cell).size();
    if (!rating.canReachEdge) rating.room = board.roomAround(cell);
    rating.distanceFromFence = NOT_REACHABLE;
    for (int wall : wallsOnEdge)
      rating.distanceFromFence = min(rating.distanceFromFence, hexDistance(board.toPoint(cell), board.toPoint(wall)));

    bool isFirstOption = !best.canMove;
    if (isFirstOption || rating.isBetterThan(best.rating)) {
      best.canMove = true;
      best.cell = cell;
      best.rating = rating;
    }
  }
  return best;
}



// Planning against a catcher built from the default template

int Agent::templateCatcherWall(const SearchBoard& board) {
  // Breadth-first search from the cat; the first edge cell discovered is the one it walls.
  // neighborsOf() uses the same order as CatWorld::neighbors(), so ties break the same way.
  vector<bool> visited(board.cellCount(), false);
  vector<int> frontier{board.catCell()};
  visited[board.catCell()] = true;
  for (size_t next = 0; next < frontier.size(); next++)
    for (int neighbor : board.neighborsOf(frontier[next])) {
      if (!board.isOpen(neighbor) || visited[neighbor]) continue;
      visited[neighbor] = true;
      if (board.isEdge(neighbor)) return neighbor;
      frontier.push_back(neighbor);
    }
  return OFF_BOARD;  // the cat can't reach the edge at all
}

// Does stepping onto `step` lead to an escape within `stepsLeft` more cat steps,
// if the catcher always answers with the template's wall?
bool Agent::escapesTemplateCatcher(SearchBoard& board, int step, int stepsLeft, chrono::steady_clock::time_point deadline) {
  if (board.isEdge(step)) return true;
  if (stepsLeft == 0 || chrono::steady_clock::now() > deadline) return false;

  int previousCat = board.catCell();
  board.moveCatTo(step);
  int wall = templateCatcherWall(board);
  if (wall != OFF_BOARD) board.placeWall(wall);

  bool escapes = false;
  vector<int> shortest = board.shortestEscapeSteps();
  for (int nextStep : board.openNeighborsOf(step)) {
    bool canStillMakeIt = shortest[nextStep] <= stepsLeft - 1;  // otherwise it can't reach the edge in time
    if (canStillMakeIt && escapesTemplateCatcher(board, nextStep, stepsLeft - 1, deadline)) {
      escapes = true;
      break;
    }
  }

  if (wall != OFF_BOARD) board.removeWall(wall);
  board.moveCatTo(previousCat);
  return escapes;
}

int Agent::bestStepAgainstTemplateCatcher(const SearchBoard& startingBoard, int thinkingTimeMs) {
  SearchBoard board = startingBoard;
  auto deadline = chrono::steady_clock::now() + chrono::milliseconds(thinkingTimeMs);
  vector<int> shortest = board.shortestEscapeSteps();
  // Look for escapes of 1 step, then 2, then 3, ...: the first one found is the fastest.
  for (int stepsAllowed = 1; stepsAllowed <= TEMPLATE_PLAN_MAX_STEPS; stepsAllowed++) {
    for (int step : board.openNeighborsOf(board.catCell())) {
      if (shortest[step] > stepsAllowed - 1) continue;
      if (escapesTemplateCatcher(board, step, stepsAllowed - 1, deadline)) return step;
    }
    if (chrono::steady_clock::now() > deadline) break;
  }
  return OFF_BOARD;
}



// GameSearch: minimax with alpha-beta pruning and iterative deepening

Agent::GameSearch::GameSearch(const SearchBoard& startingBoard, int thinkingTimeMs)
    : board(startingBoard), deadline(chrono::steady_clock::now() + chrono::milliseconds(thinkingTimeMs)) {}

bool Agent::GameSearch::outOfTime() {
  if (!firstPassDone) return false;  // never cut off the first (shallowest) pass
  if (!timeRanOut && chrono::steady_clock::now() > deadline) timeRanOut = true;
  return timeRanOut;
}

int Agent::GameSearch::bestStep() {
  int cat = board.catCell();
  vector<int> steps = board.openNeighborsOf(cat);
  if (steps.empty()) return OFF_BOARD;
  for (int step : steps)
    if (board.isEdge(step)) return step;  // an edge cell next to us means we win

  // Iterative deepening: search 2 moves ahead, then 4, then 6, etc... until time runs
  // out, always keeping the answer from the deepest search that fully finished.
  int bestSoFar = steps[0];
  for (int depth = 2; depth <= MAX_DEPTH; depth += 2) {
    // Try the previous best step first: alpha-beta prunes far more when good moves come early.
    stable_partition(steps.begin(), steps.end(), [&](int step) { return step == bestSoFar; });

    int bestThisDepth = OFF_BOARD;
    int bestScoreThisDepth = TRAPPED - 2;
    int alpha = TRAPPED - 1;  // the score the cat is already guaranteed by an earlier step
    for (int step : steps) {
      board.moveCatTo(step);
      int score = searchCatcherTurn(depth - 1, alpha, ESCAPED + 1, 1);
      board.moveCatTo(cat);
      if (outOfTime()) break;
      if (score > bestScoreThisDepth) {
        bestScoreThisDepth = score;
        bestThisDepth = step;
      }
      alpha = max(alpha, score);
    }
    if (outOfTime()) break;  // this depth didn't finish, so don't trust its answer
    bestSoFar = bestThisDepth;
    firstPassDone = true;

    bool resultIsDecided = bestScoreThisDepth >= ESCAPED - 100 || bestScoreThisDepth <= TRAPPED + 100;
    if (resultIsDecided) break;  // searching deeper can't change a forced escape or a forced capture
  }
  return bestSoFar;
}

int Agent::GameSearch::bestWall() {
  planningForCatcher = true;
  vector<int> walls = firstWallChoices();
  if (walls.empty()) {  // nothing near the cat
    for (int cell = 0; cell < board.cellCount(); cell++)
      if (cell != board.catCell() && board.isOpen(cell)) return cell;
    return OFF_BOARD;
  }

  // Iterative deepening, 1 move ahead (just this wall), then 3, 5, etc...
  int bestSoFar = walls[0];
  vector<pair<int, int>> ranking;  // (score, wall) from the deepest pass that finished, in search order
  for (int depth = 1; depth <= MAX_DEPTH; depth += 2) {
    // Try the previous best wall first: alpha-beta prunes far more when good moves come early.
    stable_partition(walls.begin(), walls.end(), [&](int wall) { return wall == bestSoFar; });

    int bestThisDepth = OFF_BOARD;
    int bestScoreThisDepth = ESCAPED + 2;  // the catcher wants this as LOW as possible
    int beta = ESCAPED + 1;                // the score the catcher is already guaranteed by an earlier wall
    vector<pair<int, int>> rankingThisDepth;
    for (int wall : walls) {
      board.placeWall(wall);
      int score = searchCatTurn(depth - 1, TRAPPED - 1, beta, 1);
      board.removeWall(wall);
      if (outOfTime()) break;
      rankingThisDepth.push_back({score, wall});
      if (score < bestScoreThisDepth) {
        bestScoreThisDepth = score;
        bestThisDepth = wall;
      }
      beta = min(beta, score);
    }
    if (outOfTime()) break;  // this depth didn't finish, so don't trust its answer
    bestSoFar = bestThisDepth;
    ranking = rankingThisDepth;
    firstPassDone = true;

    bool resultIsDecided = bestScoreThisDepth >= ESCAPED - 100 || bestScoreThisDepth <= TRAPPED + 100;
    if (resultIsDecided) break;  // a forced capture (or a lost cause) won't change with more depth
  }

  // Final safety check: try the search's walls best first (equal scores keep the search's
  // own order, so its choice is always tried first) and play the first one that leaves the
  // cat no forced escape within SAFETY_CHECK_MOVES moves.
  stable_sort(ranking.begin(), ranking.end(), [](const pair<int, int>& first, const pair<int, int>& second) { return first.first < second.first; });
  int checked = 0;
  for (auto [score, wall] : ranking) {
    if (checked++ >= SAFETY_CHECK_WALLS) break;
    board.placeWall(wall);
    bool catCanEscape = catCanForceEscape(SAFETY_CHECK_MOVES);
    board.removeWall(wall);
    if (!catCanEscape) return wall;
  }
  return bestSoFar;  // every checked wall loses anyway: trust the search
}

// Exact check, cat to move: can the cat reach the edge within `catMovesLeft` of its own
// moves no matter which walls the catcher places? Only walls on short enough escape
// routes are tried, since no other wall can stop an escape that fast.
bool Agent::GameSearch::catCanForceEscape(int catMovesLeft) {
  if (catMovesLeft <= 0) return false;
  int cat = board.catCell();
  vector<int> steps = board.openNeighborsOf(cat);
  for (int step : steps)
    if (board.isEdge(step)) return true;
  if (catMovesLeft == 1) return false;

  vector<int> toEdge = board.shortestEscapeSteps();
  for (int step : steps) {
    if (toEdge[step] > catMovesLeft - 1) continue;  // too far to escape in time from there
    board.moveCatTo(step);
    bool escapes = catcherCannotStopEscape(catMovesLeft - 1);
    board.moveCatTo(cat);
    if (escapes) return true;
  }
  return false;
}

// Exact check, catcher to move: does EVERY wall still leave the cat a forced escape?
bool Agent::GameSearch::catcherCannotStopEscape(int catMovesLeft) {
  int cat = board.catCell();
  vector<int> fromCat = board.stepsFrom({cat}, 1, catMovesLeft);
  vector<int> toEdge = board.shortestEscapeSteps();
  for (int cell = 0; cell < board.cellCount(); cell++) {
    bool onShortEnoughRoute = fromCat[cell] != NOT_REACHABLE && fromCat[cell] + toEdge[cell] <= catMovesLeft;
    if (cell == cat || !board.isOpen(cell) || !onShortEnoughRoute) continue;
    board.placeWall(cell);
    bool stillEscapes = catCanForceEscape(catMovesLeft);
    board.removeWall(cell);
    if (!stillEscapes) return false;  // this wall stops it
  }
  return true;
}

// The cat is about to move. Returns the best score the cat can get from here.
int Agent::GameSearch::searchCatTurn(int depthLeft, int alpha, int beta, int movesPlayed) {
  if (outOfTime()) return 0;  // the caller throws away results once time is up
  int cat = board.catCell();
  if (board.isEdge(cat)) return ESCAPED - movesPlayed;
  if (!board.hasOpenNeighbor(cat)) return TRAPPED + movesPlayed;
  if (depthLeft == 0) return scorePosition();  // the search stops here: no need to sort the cat's steps
  vector<int> steps = catStepsBestFirst();

  int best = TRAPPED - 1;
  for (int step : steps) {
    board.moveCatTo(step);
    best = max(best, searchCatcherTurn(depthLeft - 1, alpha, beta, movesPlayed + 1));
    board.moveCatTo(cat);
    alpha = max(alpha, best);
    if (alpha >= beta) break;  // the catcher already has a better option elsewhere; stop looking
  }
  return best;
}

// The catcher is about to place a wall. Returns the worst score it can hold the cat to.
int Agent::GameSearch::searchCatcherTurn(int depthLeft, int alpha, int beta, int movesPlayed) {
  if (outOfTime()) return 0;
  if (board.isEdge(board.catCell())) return ESCAPED - movesPlayed;
  if (depthLeft == 0) return scorePosition();

  vector<int> walls = catcherWallChoices();
  if (walls.empty()) return searchCatTurn(depthLeft - 1, alpha, beta, movesPlayed + 1);

  int worst = ESCAPED + 1;
  for (int wall : walls) {
    board.placeWall(wall);
    worst = min(worst, searchCatTurn(depthLeft - 1, alpha, beta, movesPlayed + 1));
    board.removeWall(wall);
    beta = min(beta, worst);
    if (alpha >= beta) break;  // the cat already has a better option elsewhere; stop looking
  }
  return worst;
}

// Reads a "ladder" (a Go term): the cat steps next to exactly one open edge cell, so the
// catcher MUST block that cell or lose; then the cat steps on to the next such cell, and so
// on. Every catcher reply is forced, so this follows the whole run cheaply. Returns true if
// the run ends with the cat touching two open edge cells (or an edge cell): an escape.
bool Agent::GameSearch::catWinsLadder(int stepsLeft) {
  if (stepsLeft == 0) return false;
  int cat = board.catCell();
  for (int step : board.openNeighborsOf(cat)) {
    if (board.isEdge(step)) return true;
    vector<int> openEdgesAround;
    for (int neighbor : board.neighborsOf(step))
      if (board.isOpen(neighbor) && board.isEdge(neighbor)) openEdgesAround.push_back(neighbor);
    if (openEdgesAround.size() >= 2) return true;  // two exits at once: the catcher can only block one
    if (openEdgesAround.size() == 1) {
      int forcedWall = openEdgesAround[0];
      board.moveCatTo(step);
      board.placeWall(forcedWall);
      bool escapes = catWinsLadder(stepsLeft - 1);
      board.removeWall(forcedWall);
      board.moveCatTo(cat);
      if (escapes) return true;
    }
  }
  return false;
}

// Judges a position where the search stops. Higher is better for the cat.
int Agent::GameSearch::scorePosition() {
  int bestShortest = board.fewestEscapeStepsAround(board.catCell(), 1);
  int bestGuaranteed = board.fewestEscapeStepsAround(board.catCell(), 2);
  if (bestShortest == NOT_REACHABLE) return SEALED_IN + board.roomAround(board.catCell());

  // The catcher reads ladders: a run along the edge that ends in an escape counts as one now,
  // even though it's too long for the normal search to see.
  bool closeToEdge = bestShortest <= LADDER_CHECK_RANGE;
  if (planningForCatcher && closeToEdge && catWinsLadder(LADDER_MAX_STEPS)) return ESCAPED - 1000;

  // Guaranteed steps matter most, so they get 100x the weight of the plain distance.
  int score = -100 * min(bestGuaranteed, 50) - bestShortest;

  // The catcher also counts the exits the cat could reach soon: walling off exits ahead
  // of the cat stops runners, while walls right next to it only slow them down.
  if (planningForCatcher) {
    score += NEARBY_EXIT_WEIGHT * countNearbyExits(bestShortest + 1 + EXIT_LOOKAHEAD_STEPS);
    score += FAR_EXIT_WEIGHT * countNearbyExits(bestShortest + 1 + FAR_EXIT_LOOKAHEAD_STEPS);
  }

  // Once the cat can't force an escape, the catcher's main job is done, so it closes in:
  // fewer open cells around the cat means a tighter ring and a faster capture.
  bool catHasGuaranteedEscape = bestGuaranteed != NOT_REACHABLE;
  if (planningForCatcher && !catHasGuaranteedEscape) score += CONTAINMENT_WEIGHT * countOpenCellsNearCat(CONTAINMENT_RADIUS);
  return score;
}

// How many open cells the cat can reach within `radius` steps (including where it stands).
int Agent::GameSearch::countOpenCellsNearCat(int radius) const {
  vector<int> stepsFromCat = board.stepsFrom({board.catCell()}, 1, radius);
  const int* steps = stepsFromCat.data();  // plain array: this runs at every scored position
  int cellTotal = board.cellCount(), openCells = 0;
  for (int cell = 0; cell < cellTotal; cell++)
    if (steps[cell] <= radius) openCells++;
  return openCells;
}

// How many open edge cells the cat can reach within `stepsAllowed` steps.
int Agent::GameSearch::countNearbyExits(int stepsAllowed) const {
  vector<int> stepsFromCat = board.stepsFrom({board.catCell()}, 1, stepsAllowed);
  const int* steps = stepsFromCat.data();  // plain arrays: this runs at every scored position
  const vector<int>& edgeCells = board.allEdgeCells();
  const int* edges = edgeCells.data();
  int edgeTotal = (int)edgeCells.size(), exits = 0;
  for (int index = 0; index < edgeTotal; index++)
    if (steps[edges[index]] <= stepsAllowed) exits++;
  return exits;
}

// The cat's possible steps, closest to the edge first (good moves first = more pruning).
vector<int> Agent::GameSearch::catStepsBestFirst() const {
  vector<int> steps = board.openNeighborsOf(board.catCell());
  vector<int> shortest = board.shortestEscapeSteps();
  stable_sort(steps.begin(), steps.end(), [&](int first, int second) { return shortest[first] < shortest[second]; });
  return steps;
}

// The catcher walls that the search imagined during its look-ahead
vector<int> Agent::GameSearch::catcherWallChoices() const {
  int cat = board.catCell();
  // Only cells a few steps from the cat can be chosen, so the scan can stop there.
  vector<int> stepsFromCat = board.stepsFrom({cat}, 1, max(IMAGINED_WALL_RADIUS, ESCAPE_ROUTE_RADIUS));
  vector<int> shortest = board.shortestEscapeSteps();
  bool sealedIn = shortest[cat] == NOT_REACHABLE;
  if (sealedIn) return board.openNeighborsOf(cat);

  vector<int> choices;
  for (int cell = 0; cell < board.cellCount(); cell++) {
    if (cell == cat || !board.isOpen(cell)) continue;
    bool nearCat = stepsFromCat[cell] <= IMAGINED_WALL_RADIUS;
    // A cell is on a shortest escape route when (steps from the cat to it) + (steps from it
    // to the edge) is no more than the cat's shortest escape, plus 1 for nearly-shortest routes.
    bool onEscapeRoute = planningForCatcher && stepsFromCat[cell] <= ESCAPE_ROUTE_RADIUS &&
                         stepsFromCat[cell] + shortest[cell] <= shortest[cat] + 1;
    if (nearCat || onEscapeRoute) choices.push_back(cell);
  }
  stable_sort(choices.begin(), choices.end(), [&](int first, int second) { return stepsFromCat[first] < stepsFromCat[second]; });
  return choices;
}

// The walls the catcher actually considers for its real move: open cells near the cat,
// closest first. A wider ring than inside the search, since this choice really happens.
vector<int> Agent::GameSearch::firstWallChoices() const {
  int cat = board.catCell();
  bool sealedIn = board.shortestEscapeSteps()[cat] == NOT_REACHABLE;
  int radius = sealedIn ? FIRST_WALL_RADIUS_SEALED : FIRST_WALL_RADIUS;
  vector<int> stepsFromCat = board.stepsFrom({cat}, 1, radius);

  vector<int> choices;
  for (int cell = 0; cell < board.cellCount(); cell++)
    if (cell != cat && board.isOpen(cell) && stepsFromCat[cell] <= radius) choices.push_back(cell);
  stable_sort(choices.begin(), choices.end(), [&](int first, int second) { return stepsFromCat[first] < stepsFromCat[second]; });
  return choices;
}

// Original Template
// Unused
std::vector<Point2D> Agent::generatePath(CatWorld* w) {
  unordered_map<Point2D, Point2D> cameFrom;  // to build the flowfield and build the path
  queue<Point2D> frontier;                   // to store next ones to visit
  unordered_set<Point2D> frontierSet;        // OPTIMIZATION to check faster if a point is in the queue
  unordered_map<Point2D, bool> visited;      // use .at() to get data, if the element dont exist [] will give you wrong results

  // bootstrap state
  auto catPos = w->getCat();
  frontier.push(catPos);
  frontierSet.insert(catPos);
  Point2D borderExit = {INT32_MAX, INT32_MAX};  // sentinel: no border found yet

  while (!frontier.empty()) {
    // get the current from frontier
    // remove the current from frontierset
    // mark current as visited
    // getVisitableNeightbors(world, current) returns a vector of neighbors that are not visited, not cat, not block, not in the queue
    // iterate over the neighs:
    // for every neighbor set the cameFrom
    // enqueue the neighbors to frontier and frontierset
    // do this up to find a visitable border and break the loop
  }

  // if the border is not infinity, build the path from border to the cat using the camefrom map
  // if there isnt a reachable border, just return empty vector
  // if your vector is filled from the border to the cat, the first element is the catcher move, and the last element is the cat move
  return vector<Point2D>();
}