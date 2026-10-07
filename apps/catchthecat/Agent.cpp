#include "Agent.h"
#include <algorithm>
#include <climits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include "World.h"

using namespace std;

// SearchBoard

Agent::SearchBoard::SearchBoard(const CatWorld& world)
    : side(world.getWorldSideSize()),
      half(side / 2),
      walls(world.worldState()),
      edgeCells(side * side, false),
      neighborTable(side * side) {
  cat = toCell(world.getCat());
  for (int cell = 0; cell < cellCount(); cell++) {
    Point2D point = toPoint(cell);
    edgeCells[cell] = world.catWinsOnSpace(point);
    vector<Point2D> around = CatWorld::neighbors(point);  // same neighbor order as the game uses
    for (int direction = 0; direction < 6; direction++)
      neighborTable[cell][direction] = world.isValidPosition(around[direction]) ? toCell(around[direction]) : OFF_BOARD;
  }
}

vector<int> Agent::SearchBoard::openNeighborsOf(int cell) const {
  vector<int> open;
  for (int neighbor : neighborsOf(cell))
    if (isOpen(neighbor)) open.push_back(neighbor);
  return open;
}

vector<int> Agent::SearchBoard::stepsFrom(const vector<int>& startCells, int neighborsNeeded) const {
  vector<int> steps(cellCount(), NOT_REACHABLE);
  vector<int> timesReached(cellCount(), 0);
  vector<int> frontier = startCells;  // used as a queue: we read it front to back while appending
  for (int start : startCells) steps[start] = 0;

  for (size_t next = 0; next < frontier.size(); next++) {
    int current = frontier[next];
    for (int neighbor : neighborsOf(current)) {
      if (!isOpen(neighbor) || steps[neighbor] != NOT_REACHABLE) continue;
      timesReached[neighbor]++;
      if (timesReached[neighbor] < neighborsNeeded) continue;
      steps[neighbor] = steps[current] + 1;
      frontier.push_back(neighbor);
    }
  }
  return steps;
}

vector<int> Agent::SearchBoard::openEdgeCells() const {
  vector<int> edges;
  for (int cell = 0; cell < cellCount(); cell++)
    if (isEdge(cell) && isOpen(cell)) edges.push_back(cell);
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
  return openNeighborCount > other.openNeighborCount;
}

Agent::CatStep Agent::bestStepForCat(const SearchBoard& board) {
  vector<int> guaranteed = board.guaranteedEscapeSteps();
  vector<int> shortest = board.shortestEscapeSteps();
  CatStep best;
  for (int cell : board.openNeighborsOf(board.catCell())) {
    SpotRating rating;
    rating.isEdge = board.isEdge(cell);
    rating.shortestSteps = shortest[cell];
    rating.canReachEdge = rating.shortestSteps != NOT_REACHABLE;
    rating.guaranteedSteps = guaranteed[cell];
    rating.openNeighborCount = (int)board.openNeighborsOf(cell).size();
    if (!rating.canReachEdge) rating.room = board.roomAround(cell);

    bool isFirstOption = !best.canMove;
    if (isFirstOption || rating.isBetterThan(best.rating)) {
      best.canMove = true;
      best.cell = cell;
      best.rating = rating;
    }
  }
  return best;
}



// GameSearch: minimax with alpha-beta pruning and iterative deepening

Agent::GameSearch::GameSearch(const SearchBoard& startingBoard, int thinkingTimeMs)
    : board(startingBoard), deadline(chrono::steady_clock::now() + chrono::milliseconds(thinkingTimeMs)) {}

bool Agent::GameSearch::outOfTime() {
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
  for (int depth = 1; depth <= MAX_DEPTH; depth += 2) {
    // Try the previous best wall first: alpha-beta prunes far more when good moves come early.
    stable_partition(walls.begin(), walls.end(), [&](int wall) { return wall == bestSoFar; });

    int bestThisDepth = OFF_BOARD;
    int bestScoreThisDepth = ESCAPED + 2;  // the catcher wants this as LOW as possible
    int beta = ESCAPED + 1;                // the score the catcher is already guaranteed by an earlier wall
    for (int wall : walls) {
      board.placeWall(wall);
      int score = searchCatTurn(depth - 1, TRAPPED - 1, beta, 1);
      board.removeWall(wall);
      if (outOfTime()) break;
      if (score < bestScoreThisDepth) {
        bestScoreThisDepth = score;
        bestThisDepth = wall;
      }
      beta = min(beta, score);
    }
    if (outOfTime()) break;  // this depth didn't finish, so don't trust its answer
    bestSoFar = bestThisDepth;

    bool resultIsDecided = bestScoreThisDepth >= ESCAPED - 100 || bestScoreThisDepth <= TRAPPED + 100;
    if (resultIsDecided) break;  // a forced capture (or a lost cause) won't change with more depth
  }
  return bestSoFar;
}

// The cat is about to move. Returns the best score the cat can get from here.
int Agent::GameSearch::searchCatTurn(int depthLeft, int alpha, int beta, int movesPlayed) {
  if (outOfTime()) return 0;  // the caller throws away results once time is up
  int cat = board.catCell();
  if (board.isEdge(cat)) return ESCAPED - movesPlayed;
  vector<int> steps = catStepsBestFirst();
  if (steps.empty()) return TRAPPED + movesPlayed;
  if (depthLeft == 0) return scorePosition();

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

// Judges a position where the search stops. Higher is better for the cat.
int Agent::GameSearch::scorePosition() const {
  vector<int> guaranteed = board.guaranteedEscapeSteps();
  vector<int> shortest = board.shortestEscapeSteps();
  int bestGuaranteed = NOT_REACHABLE;
  int bestShortest = NOT_REACHABLE;
  for (int step : board.openNeighborsOf(board.catCell())) {
    bestGuaranteed = min(bestGuaranteed, guaranteed[step]);
    bestShortest = min(bestShortest, shortest[step]);
  }
  if (bestShortest == NOT_REACHABLE) return SEALED_IN + board.roomAround(board.catCell());

  // Guaranteed steps matter most, so they get 100x the weight of the plain distance.
  int score = -100 * min(bestGuaranteed, 50) - bestShortest;

  // The catcher also counts the exits the cat could reach soon: walling off exits ahead
  // of the cat stops runners, while walls right next to it only slow them down.
  if (planningForCatcher) score += NEARBY_EXIT_WEIGHT * countNearbyExits(bestShortest + 1 + EXIT_LOOKAHEAD_STEPS);
  return score;
}

// How many open edge cells the cat can reach within `stepsAllowed` steps.
int Agent::GameSearch::countNearbyExits(int stepsAllowed) const {
  vector<int> stepsFromCat = board.stepsFrom({board.catCell()}, 1);
  int exits = 0;
  for (int cell = 0; cell < board.cellCount(); cell++)
    if (board.isEdge(cell) && stepsFromCat[cell] <= stepsAllowed) exits++;
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
  vector<int> stepsFromCat = board.stepsFrom({cat}, 1);
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
  vector<int> stepsFromCat = board.stepsFrom({cat}, 1);
  bool sealedIn = board.shortestEscapeSteps()[cat] == NOT_REACHABLE;
  int radius = sealedIn ? FIRST_WALL_RADIUS_SEALED : FIRST_WALL_RADIUS;

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
