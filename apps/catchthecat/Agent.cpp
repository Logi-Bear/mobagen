#include "Agent.h"
#include <climits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include "World.h"

using namespace std;

// CellNumbers

Agent::CellNumbers::CellNumbers(const CatWorld& world, int startingValue)
    : side(world.getWorldSideSize()), numbers(side * side, startingValue) {}

int Agent::CellNumbers::get(Point2D cell) const { return numbers[indexOf(cell)]; }

void Agent::CellNumbers::set(Point2D cell, int value) { numbers[indexOf(cell)] = value; }

int Agent::CellNumbers::countCellsNotEqualTo(int value) const {
  int count = 0;
  for (int number : numbers)
    if (number != value) count++;
  return count;
}

int Agent::CellNumbers::indexOf(Point2D cell) const { return (cell.y + side / 2) * side + (cell.x + side / 2); }



// SpotRating

bool Agent::SpotRating::isBetterThan(const SpotRating& other) const {
  if (isEdge != other.isEdge) return isEdge;
  if (canReachEdge != other.canReachEdge) return canReachEdge;
  if (!canReachEdge) return room > other.room;  // sealed in: more room means surviving longer
  if (guaranteedSteps != other.guaranteedSteps) return guaranteedSteps < other.guaranteedSteps;
  if (shortestSteps != other.shortestSteps) return shortestSteps < other.shortestSteps;
  return openNeighborCount > other.openNeighborCount;
}



// Board helpers

bool Agent::isOpen(const CatWorld& world, Point2D cell) {
  return world.isValidPosition(cell) && !world.getContent(cell);
}

vector<Point2D> Agent::allCells(const CatWorld& world) {
  int half = world.getWorldSideSize() / 2;
  vector<Point2D> cells;
  for (int row = -half; row <= half; row++)
    for (int column = -half; column <= half; column++) cells.push_back({column, row});
  return cells;
}

vector<Point2D> Agent::openNeighbors(const CatWorld& world, Point2D cell) {
  vector<Point2D> open;
  for (Point2D neighbor : CatWorld::neighbors(cell))
    if (isOpen(world, neighbor)) open.push_back(neighbor);
  return open;
}

vector<Point2D> Agent::openEdgeCells(const CatWorld& world) {
  vector<Point2D> edges;
  for (Point2D cell : allCells(world))
    if (world.catWinsOnSpace(cell) && isOpen(world, cell)) edges.push_back(cell);
  return edges;
}

Agent::CellNumbers Agent::countStepsFrom(const CatWorld& world, const vector<Point2D>& startCells, int neighborsNeeded) {
  CellNumbers steps(world, NOT_REACHABLE);
  CellNumbers timesReached(world, 0);
  queue<Point2D> frontier;
  for (Point2D start : startCells) {
    steps.set(start, 0);
    frontier.push(start);
  }
  while (!frontier.empty()) {
    Point2D current = frontier.front();
    frontier.pop();
    for (Point2D neighbor : openNeighbors(world, current)) {
      if (steps.get(neighbor) != NOT_REACHABLE) continue;
      timesReached.set(neighbor, timesReached.get(neighbor) + 1);
      if (timesReached.get(neighbor) < neighborsNeeded) continue;
      steps.set(neighbor, steps.get(current) + 1);
      frontier.push(neighbor);
    }
  }
  return steps;
}

Agent::CellNumbers Agent::shortestEscapeSteps(const CatWorld& world) { return countStepsFrom(world, openEdgeCells(world), 1); }

Agent::CellNumbers Agent::guaranteedEscapeSteps(const CatWorld& world) { return countStepsFrom(world, openEdgeCells(world), 2); }

int Agent::roomAround(const CatWorld& world, Point2D cell) {
  return countStepsFrom(world, {cell}, 1).countCellsNotEqualTo(NOT_REACHABLE);
}

Agent::CatStep Agent::bestStepForCat(const CatWorld& world) {
  CellNumbers guaranteed = guaranteedEscapeSteps(world);
  CellNumbers shortest = shortestEscapeSteps(world);
  CatStep best;
  for (Point2D cell : openNeighbors(world, world.getCat())) {
    SpotRating rating;
    rating.isEdge = world.catWinsOnSpace(cell);
    rating.shortestSteps = shortest.get(cell);
    rating.canReachEdge = rating.shortestSteps != NOT_REACHABLE;
    rating.guaranteedSteps = guaranteed.get(cell);
    rating.openNeighborCount = (int)openNeighbors(world, cell).size();
    if (!rating.canReachEdge) rating.room = roomAround(world, cell);

    if (!best.canMove || rating.isBetterThan(best.rating)) best = {true, cell, rating};
  }
  return best;
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
