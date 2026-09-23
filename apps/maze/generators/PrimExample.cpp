#include "PrimExample.h"
#include "../World.h"
#include "Random.h"

static const Color32 kUnvisited = {0.0f, 0.0f, 0.0f, 1.0f};
static const Color32 kWhite = {1.0f, 1.0f, 1.0f, 1.0f};
static const Color32 kRed = Color::Red;

bool PrimExample::Step(World* w) {
  if (!initialized) {
    //setup start cell
    Point2D const start = {w->GetWidth() / 2, w->GetHeight() / 2};
    w->SetNodeColor(start, kWhite);
    for (auto& v : getVisitables(w, start)) {
      w->SetNodeColor(v, kRed);       // "in the frontier"
      toBeVisited.push_back(v);
    }
    initialized = true;
    return true;
  }

  if (toBeVisited.empty()) return false; //maze is done

  //pick random cell
  int const index = Random::Range(0, static_cast<int>(toBeVisited.size()) - 1);
  Point2D const current = toBeVisited[index];
  toBeVisited[index] = toBeVisited.back();
  toBeVisited.pop_back();

  //pick random visited neighbor
  auto visited = getVisitedNeighbors(w, current);
  if (!visited.empty()) {
    Point2D other = visited[Random::Range(0, (int)visited.size() - 1)];

    //remove wall between them
    int dx = other.x - current.x;
    int dy = other.y - current.y;
    if (dy == -1) w->SetNorth(current, false);
    else if (dy == 1) w->SetSouth(current, false);
    else if (dx == -1) w->SetWest(current, false);
    else if (dx == 1) w->SetEast(current, false);
  }

  //mark cell as in maze
  w->SetNodeColor(current, kWhite);

  //add its visitables to frontier
  for (auto& v : getVisitables(w, current)) {
    w->SetNodeColor(v, Color::Red);
    toBeVisited.push_back(v);
  }

  return true;
}
void PrimExample::Clear(World* world) {
  toBeVisited.clear();
  initialized = false;
}

std::vector<Point2D> PrimExample::getVisitables(World* w, const Point2D& p) {
  std::vector<Point2D> visitables;

  int const width = w->GetWidth();
  int const height = w->GetHeight();
  std::vector<Point2D> const deltas = {Point2D(0, -1), Point2D(0, 1), Point2D(-1, 0), Point2D(1, 0)};

  for (const auto& d : deltas) {
    Point2D const n(p.x + d.x, p.y + d.y);
    if (n.x < 0 || n.x >= width || n.y < 0 || n.y >= height) continue;
    Color32 nColor = w->GetNodeColor(n);
    if (nColor.r == kUnvisited.r && nColor.g == kUnvisited.g && nColor.b == kUnvisited.b && nColor.a == kUnvisited.a) visitables.push_back(n);
  }

  return visitables;
}

std::vector<Point2D> PrimExample::getVisitedNeighbors(World* w, const Point2D& p) {
  std::vector<Point2D> deltas = {Point2D(0, -1), Point2D(0, 1), Point2D(-1, 0), Point2D(1, 0)};  // N, S, W, E
  std::vector<Point2D> neighbors;

  int const width = w->GetWidth();
  int const height = w->GetHeight();

  for (const auto& d : deltas) {
    Point2D const n(p.x + d.x, p.y + d.y);
    if (n.x < 0 || n.x >= width || n.y < 0 || n.y >= height) continue;
    Color32 nColor = w->GetNodeColor(n);
    if (nColor.r == kWhite.r && nColor.g == kWhite.g && nColor.b == kWhite.b && nColor.a == kWhite.a) neighbors.push_back(n);
  }

  return neighbors;
}
