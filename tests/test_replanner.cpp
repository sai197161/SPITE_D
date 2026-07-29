#include "planner/replanner.hpp"

#include <cassert>
#include <set>
#include <utility>

int main() {
  using namespace spite_d;
  using VID = Replanner::VID;

  // 0 -- 1 -- 2
  //  \       /
  //   3 ----4     (longer detour)
  const std::vector<Replanner::Edge> edges = {
      {0, 1, 1.0}, {1, 2, 1.0}, {0, 3, 2.0}, {3, 4, 2.0}, {4, 2, 2.0}};
  Replanner planner(5, edges);

  const auto allValid = [](VID, VID) { return true; };

  auto path = planner.Plan(0, 2, allValid);
  assert((path == std::vector<VID>{0, 1, 2}));
  assert(!planner.PathBlocked(path, allValid));

  // Block edge (1,2): the detour must be taken. The predicate is
  // queried per direction; block both orientations like the validity
  // server (which canonicalizes) would report.
  const auto blocked12 = [](VID a, VID b) {
    return !((a == 1 && b == 2) || (a == 2 && b == 1));
  };
  assert(planner.PathBlocked(path, blocked12));
  path = planner.Plan(0, 2, blocked12);
  assert((path == std::vector<VID>{0, 3, 4, 2}));

  // Fully disconnect the goal.
  const auto noneValid = [](VID, VID) { return false; };
  assert(planner.Plan(0, 2, noneValid).empty());

  return 0;
}
