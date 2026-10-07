#ifndef AGENT_H
#define AGENT_H

#include <glm/glm.hpp>
#include <array>
#include <chrono>
#include <functional>
#include <vector>

// Point2D is now glm::ivec2 — same x,y interface, no OOP wrapper needed.
using Point2D = glm::ivec2;

// Hash specialization so Point2D (= glm::ivec2) works in unordered containers.
namespace std {
  template <> struct hash<glm::ivec2> {
    std::size_t operator()(const glm::ivec2& v) const noexcept {
      std::size_t seed = std::hash<int>{}(v.x);
      seed ^= std::hash<int>{}(v.y) + 0x9e3779b9u + (seed << 6) + (seed >> 2);
      return seed;
    }
  };
}  // namespace std

class CatWorld;

class Agent {
public:
  explicit Agent() = default;
  virtual ~Agent() = default;

  virtual Point2D Move(CatWorld*) = 0;

  std::vector<Point2D> generatePath(CatWorld* w);

protected:
  static constexpr int NOT_REACHABLE = 1000000;
  static constexpr int OFF_BOARD = -1;
  static constexpr int TEMPLATE_PLAN_MAX_STEPS = 20;  // longest escape plan tried against a template catcher

  // A fast, editable copy of the board for imagining future moves.
  class SearchBoard {
  public:
    explicit SearchBoard(const CatWorld& world);

    int cellCount() const { return (int)walls.size(); }
    int catCell() const { return cat; }
    bool isOpen(int cell) const { return cell != OFF_BOARD && !walls[cell]; }
    bool hasOpenNeighbor(int cell) const;
    bool isEdge(int cell) const { return edgeCells[cell]; }
    const std::array<int, 6>& neighborsOf(int cell) const { return neighborTable[cell]; }
    std::vector<int> openNeighborsOf(int cell) const;

    void placeWall(int cell) { walls[cell] = true; }
    void removeWall(int cell) { walls[cell] = false; }
    void moveCatTo(int cell) { cat = cell; }

    int toCell(Point2D point) const { return (point.y + half) * side + (point.x + half); }
    Point2D toPoint(int cell) const { return {cell % side - half, cell / side - half}; }

    // Steps from the start cells to every open cell.
    std::vector<int> stepsFrom(const std::vector<int>& startCells, int neighborsNeeded, int maxSteps = NOT_REACHABLE) const;

    // Fewest steps from each cell to the edge, if nobody gets in the way.
    std::vector<int> shortestEscapeSteps() const { return stepsFrom(openEdgeCells(), 1); }

    // Fewest steps from each cell to the edge even if the catcher blocks the cat's
    // best move every turn. Since one way forward always gets blocked, a cell only
    // counts if TWO of its neighbors lead out.
    std::vector<int> guaranteedEscapeSteps() const { return stepsFrom(openEdgeCells(), 2); }

    // How many open cells the cat could still reach from this cell.
    int roomAround(int cell) const;

  private:
    std::vector<int> openEdgeCells() const;

    int side;
    int half;
    int cat;

    // One byte per cell. Plain bytes instead of std::vector<bool>, which packs
    // 8 cells into each byte and makes every lookup slower, and these are checked constantly.
    std::vector<unsigned char> walls;
    std::vector<unsigned char> edgeCells;
    std::vector<int> edgeCellList;                  // every edge cell, so we don't scan the whole board to find them
    std::vector<std::array<int, 6>> neighborTable;

    // Working memory for stepsFrom, reused between calls instead of reallocated every time.
    mutable std::vector<int> scratchTimesReached;
    mutable std::vector<int> scratchFrontier;
  };

  // Everything the cat cares about when deciding where to step.
  struct SpotRating {
    bool isEdge = false;        // stepping here wins the game
    bool canReachEdge = false;  // false means the cat is sealed in
    int room = 0;               // open cells the cat could still roam (only matters when sealed in)
    int guaranteedSteps = NOT_REACHABLE;
    int shortestSteps = NOT_REACHABLE;
    int openNeighborCount = 0;
    int distanceFromFence = 0;  // how far this cell is from the nearest walled edge cell

    // Compares the rules in order of importance; the first difference decides.
    bool isBetterThan(const SpotRating& other) const;
  };

  struct CatStep {
    bool canMove = false;  // false means the cat is completely surrounded
    int cell = OFF_BOARD;
    SpotRating rating;
  };

  // The best step for the cat by the simple rules in SpotRating, with no lookahead.
  // Used against fence builders, where running straight for the exit works best.
  static CatStep bestStepForCat(const SearchBoard& board);

  // The default generatePath makes a catcher that searches outward from the
  // cat (in the game's neighbor order) and walls the first edge cell it finds. That rule
  // is fully predictable, so against it the cat can plan an exact escape.
  static int templateCatcherWall(const SearchBoard& board);

  // The first step of the fastest escape against a template catcher, or OFF_BOARD if
  // none was found within the time limit (then the caller should use another plan).
  static int bestStepAgainstTemplateCatcher(const SearchBoard& board, int thinkingTimeMs);
  static bool escapesTemplateCatcher(SearchBoard& board, int step, int stepsLeft,
                                     std::chrono::steady_clock::time_point deadline);

  // The search engine both agents use: plays the next few moves out in its head
  // (minimax with alpha-beta pruning) and picks the move that holds up best.
  // Scores are always from the cat's point of view, so the cat looks for the
  // highest score and the catcher for the lowest.
  class GameSearch {
  public:
    GameSearch(const SearchBoard& startingBoard, int thinkingTimeMs);

    // For the cat: the best step found before time ran out, or OFF_BOARD if the cat can't move.
    int bestStep();

    // For the catcher: the best wall found before time ran out.
    int bestWall();

  private:
    // Scores are from the cat's point of view: higher is better for the cat.
    static constexpr int ESCAPED = 1000000;    // minus moves played, so faster escapes score higher
    static constexpr int TRAPPED = -1000000;   // plus moves played, so later captures score higher
    static constexpr int SEALED_IN = -100000;  // plus pocket size, so bigger pockets score higher
    static constexpr int IMAGINED_WALL_RADIUS = 2;  // inside the search: catcher walls considered within this many steps of the cat
    static constexpr int FIRST_WALL_RADIUS = 5;     // the catcher's actual move: walls considered within this many steps
    static constexpr int FIRST_WALL_RADIUS_SEALED = 3;
    static constexpr int ESCAPE_ROUTE_RADIUS = 5;   // when planning for the catcher: also imagine walls on the cat's
                                                    // escape routes up to this many steps away (stops edge-runners)
    static constexpr int MAX_DEPTH = 12;
    static constexpr int NEARBY_EXIT_WEIGHT = 50;   // For closing exits ahead of the cat instead of chasing it.
    static constexpr int EXIT_LOOKAHEAD_STEPS = 2;
    static constexpr int LADDER_CHECK_RANGE = 2;  // when planning for the catcher: read ladders once the cat is this close to the edge
    static constexpr int LADDER_MAX_STEPS = 12;   // how far the ladder reader follows a run along the edge
    static constexpr int CONTAINMENT_WEIGHT = 100;
    static constexpr int CONTAINMENT_RADIUS = 3;

    int searchCatTurn(int depthLeft, int alpha, int beta, int movesPlayed);
    int searchCatcherTurn(int depthLeft, int alpha, int beta, int movesPlayed);
    int scorePosition();
    bool catWinsLadder(int stepsLeft);
    int countOpenEdgeNeighbors(int cell) const;
    int countNearbyExits(int stepsAllowed) const;
    int countOpenCellsNearCat(int radius) const;
    std::vector<int> catStepsBestFirst() const;
    std::vector<int> catcherWallChoices() const;
    std::vector<int> firstWallChoices() const;
    bool outOfTime();

    SearchBoard board;
    std::chrono::steady_clock::time_point deadline;
    bool timeRanOut = false;
    bool firstPassDone = false;  // the shallowest search always finishes, so there's always a real answer
    bool planningForCatcher = false;  // set by bestWall(): lets the search imagine walls along escape routes
  };
};

#endif  // AGENT_H
