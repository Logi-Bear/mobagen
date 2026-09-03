#include "BoundedAreaRule.h"
#include "imgui.h"
#include <glm/glm.hpp>
#include <algorithm>

glm::vec2 BoundedAreaRule::computeForce(const std::vector<BoidView>& neighborhood, const BoidView& boid) {
  glm::vec2 force(0.f);
  ImVec2 displaySize = ImGui::GetIO().DisplaySize;
  // desiredDistance is the distance from the borders that the boids should try to maintain. 

  // begin solution
  float displayWidth = displaySize.x;
  float displayHeight = displaySize.y;

  float lookAheadFrames = 5.f;
  glm::vec2 futurePos = boid.position + (boid.velocity * lookAheadFrames);

  // Left wall
  if (futurePos.x < desiredDistance) {
    float penetration = desiredDistance - futurePos.x;
    force.x += penetration;
  }
  // Right wall
  else if (futurePos.x > displayWidth - desiredDistance) {
    float penetration = futurePos.x - (displayWidth - desiredDistance);
    force.x -= penetration;
  }

  // Top wall
  if (futurePos.y < desiredDistance) {
    float penetration = desiredDistance - futurePos.y;
    force.y += penetration;
  }
  // Bottom wall
  else if (futurePos.y > displayHeight - desiredDistance) {
    float penetration = futurePos.y - (displayHeight - desiredDistance);
    force.y -= penetration;
  }
  // end solution

  return force;
}

bool BoundedAreaRule::drawImguiRuleExtra() {
  ImVec2 displaySize = ImGui::GetIO().DisplaySize;
  float widthWindows = displaySize.x > 0.f ? displaySize.x : 1280.f;
  float heightWindows = displaySize.y > 0.f ? displaySize.y : 800.f;
  bool valueHasChanged = false;
  int minHeightWidth = static_cast<int>(std::min(widthWindows, heightWindows));

  if (ImGui::SliderInt("Desired Distance From Borders", &desiredDistance, 0, minHeightWidth / 3, "%i")) {
    valueHasChanged = true;
  }
  return valueHasChanged;
}

void BoundedAreaRule::drawWorldOverlay(ImDrawList* dl) const {
  ImVec2 displaySize = ImGui::GetIO().DisplaySize;
  float w = displaySize.x > 0.f ? displaySize.x : 1280.f;
  float h = displaySize.y > 0.f ? displaySize.y : 800.f;
  float d = static_cast<float>(desiredDistance);
  ImU32 col = IM_COL32(128, 128, 128, 200);

  dl->AddLine({d, d}, {w - d, d}, col);
  dl->AddLine({w - d, d}, {w - d, h - d}, col);
  dl->AddLine({w - d, h - d}, {d, h - d}, col);
  dl->AddLine({d, h - d}, {d, d}, col);
}
