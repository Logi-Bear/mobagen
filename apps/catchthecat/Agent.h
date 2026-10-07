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
  static const int NOT_REACHABLE = 1000000;
  static const int OFF_BOARD = -1;

  // A fast, editable copy of the board for imagining future moves.
  // A cell is just an int, and each cell's six neighbors are looked up once.
  class SearchBoard {
  public:
    explicit SearchBoard(const CatWorld& world);

    int cellCount() const { return (int)walls.size(); }
    int catCell() const { return cat; }
    bool isOpen(int cell) const { return cell != OFF_BOARD && !walls[cell]; }
    bool isEdge(int cell) const { return edgeCells[cell]; }
    const std::array<int, 6>& neighborsOf(int cell) const { return neighborTable[cell]; }
    std::vector<int> openNeighborsOf(int cell) const;

    void placeWall(int cell) { walls[cell] = true; }
    void removeWall(int cell) { walls[cell] = false; }
    void moveCatTo(int cell) { cat = cell; }

    int toCell(Point2D point) const { return (point.y + half) * side + (point.x + half); }
    Point2D toPoint(int cell) const { return {cell % side - half, cell / side - half}; }

    // Steps from the start cells to every open cell.
    std::vector<int> stepsFrom(const std::vector<int>& startCells, int neighborsNeeded) const;

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
    std::vector<bool> walls;
    std::vector<bool> edgeCells;
    std::vector<std::array<int, 6>> neighborTable;
  };

  // Everything the cat cares about when deciding where to step.
  struct SpotRating {
    bool isEdge = false;        // stepping here wins the game
    bool canReachEdge = false;  // false means the cat is sealed in
    int room = 0;               // open cells the cat could still roam (only matters when sealed in)
    int guaranteedSteps = NOT_REACHABLE;
    int shortestSteps = NOT_REACHABLE;
    int openNeighborCount = 0;

    // Compares the rules in order of importance; the first difference decides.
    bool isBetterThan(const SpotRating& other) const;
  };

  struct CatStep {
    bool canMove = false;  // false means the cat is completely surrounded
    int cell = OFF_BOARD;
    SpotRating rating;
  };

  // The best step for the cat by the simple rules in SpotRating, with no lookahead.
  static CatStep bestStepForCat(const SearchBoard& board);

  // The cat's search engine: plays the next few moves out in its head
  // (minimax with alpha-beta pruning) and picks the step that holds up best.
  class CatSearch {
  public:
    CatSearch(const SearchBoard& startingBoard, int thinkingTimeMs);

    // The best step found before time ran out, or OFF_BOARD if the cat can't move.
    int bestStep();

  private:
    // Scores are from the cat's point of view: higher is better for the cat.
    static const int ESCAPED = 1000000;    // minus moves played, so faster escapes score higher
    static const int TRAPPED = -1000000;   // plus moves played, so later captures score higher
    static const int SEALED_IN = -100000;  // plus pocket size, so bigger pockets score higher
    static const int IMAGINED_WALL_RADIUS = 2;  // catcher walls considered within this many steps of the cat
    static const int MAX_DEPTH = 12;

    int searchCatTurn(int depthLeft, int alpha, int beta, int movesPlayed);
    int searchCatcherTurn(int depthLeft, int alpha, int beta, int movesPlayed);
    int scorePosition() const;
    std::vector<int> catStepsBestFirst() const;
    std::vector<int> catcherWallChoices() const;
    bool outOfTime();

    SearchBoard board;
    std::chrono::steady_clock::time_point deadline;
    bool timeRanOut = false;
  };
};

#endif  // AGENT_H
