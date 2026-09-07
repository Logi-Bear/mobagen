#include "BoundedAreaRule.h"
#include "imgui.h"
#include <glm/glm.hpp>
#include <algorithm>

float BoundedAreaRule::raycastToBounds(glm::vec2 rayOrigin, glm::vec2 rayDirection, float boundsWidth, float boundsHeight, float maxSearchDistance) {
  float closestHitDistance = maxSearchDistance;
  float leftBound   = desiredDistance;
  float rightBound  = boundsWidth  - desiredDistance;
  float topBound    = desiredDistance;
  float bottomBound = boundsHeight - desiredDistance;

  if (rayDirection.x < 0.f) {
    float distanceToLeftWall = (leftBound - rayOrigin.x) / rayDirection.x;
    if (distanceToLeftWall > 0.f && distanceToLeftWall < closestHitDistance) {
      float hitY = rayOrigin.y + distanceToLeftWall * rayDirection.y;
      if (hitY >= topBound && hitY <= bottomBound) closestHitDistance = distanceToLeftWall;
    }
  } else if (rayDirection.x > 0.f) {
    float distanceToRightWall = (rightBound - rayOrigin.x) / rayDirection.x;
    if (distanceToRightWall > 0.f && distanceToRightWall < closestHitDistance) {
      float hitY = rayOrigin.y + distanceToRightWall * rayDirection.y;
      if (hitY >= topBound && hitY <= bottomBound) closestHitDistance = distanceToRightWall;
    }
  }

  if (rayDirection.y < 0.f) {
    float distanceToTopWall = (topBound - rayOrigin.y) / rayDirection.y;
    if (distanceToTopWall > 0.f && distanceToTopWall < closestHitDistance) {
      float hitX = rayOrigin.x + distanceToTopWall * rayDirection.x;
      if (hitX >= leftBound && hitX <= rightBound) closestHitDistance = distanceToTopWall;
    }
  } else if (rayDirection.y > 0.f) {
    float distanceToBottomWall = (bottomBound - rayOrigin.y) / rayDirection.y;
    if (distanceToBottomWall > 0.f && distanceToBottomWall < closestHitDistance) {
      float hitX = rayOrigin.x + distanceToBottomWall * rayDirection.x;
      if (hitX >= leftBound && hitX <= rightBound) closestHitDistance = distanceToBottomWall;
    }
  }

  return closestHitDistance;
}

glm::vec2 BoundedAreaRule::computeForce(const std::vector<BoidView>& neighborhood, const BoidView& boid) {
  glm::vec2 force(0.f);
  ImVec2 displaySize = ImGui::GetIO().DisplaySize;
  // desiredDistance is the distance from the borders that the boids should try to maintain. 

  // begin solution
  float displayWidth = displaySize.x;
  float displayHeight = displaySize.y;

  float searchAngle = glm::radians(90.f);
  float cosAngle = cos(searchAngle);
  float sinAngle = sin(searchAngle);

  glm::vec2 headingDirection = glm::normalize(boid.velocity);
  glm::vec2 leftSearchDirection(headingDirection.x * cosAngle - headingDirection.y * sinAngle, headingDirection.x * sinAngle + headingDirection.y * cosAngle);
  glm::vec2 rightSearchDirection(headingDirection.x * cosAngle + headingDirection.y * sinAngle, -headingDirection.x * sinAngle + headingDirection.y * cosAngle);

  float wallScanDistance = 200.f;

  float steerPower = 15.f;

  float forwardHitDistance = raycastToBounds(boid.position, headingDirection, displayWidth, displayHeight, wallScanDistance);
  float leftHitDistance = raycastToBounds(boid.position, leftSearchDirection, displayWidth, displayHeight, wallScanDistance * .75f);
  float rightHitDistance = raycastToBounds(boid.position, rightSearchDirection, displayWidth, displayHeight, wallScanDistance *.75f);

  float searchHitDistance = glm::min(forwardHitDistance, glm::min(leftHitDistance, rightHitDistance));

  if (boid.position.x < desiredDistance || boid.position.x > displayWidth - desiredDistance || boid.position.y < desiredDistance || boid.position.y > displayHeight - desiredDistance)
  {
    force.x += .015f * ((displayWidth * .5f) - boid.position.x);
    force.y += .015f * ((displayHeight * .5f) - boid.position.y);
  }
  else if (searchHitDistance < wallScanDistance)
  {
    float avoidanceAngle = glm::radians(90.f);

    float cosAvoidAngle = cos(avoidanceAngle);
    float sinAvoidAngle = sin(avoidanceAngle);
    glm::vec2 leftScanDirection(headingDirection.x * cosAvoidAngle - headingDirection.y * sinAvoidAngle, headingDirection.x * sinAvoidAngle + headingDirection.y * cosAvoidAngle);
    glm::vec2 rightScanDirection(headingDirection.x * cosAvoidAngle + headingDirection.y * sinAvoidAngle, -headingDirection.x * sinAvoidAngle + headingDirection.y * cosAvoidAngle);
    float leftAvoidDistance  = raycastToBounds(boid.position, leftScanDirection,  displayWidth, displayHeight, wallScanDistance);
    float rightAvoidDistance = raycastToBounds(boid.position, rightScanDirection, displayWidth, displayHeight, wallScanDistance);

    glm::vec2 avoidanceDirection;
    if (leftAvoidDistance > rightAvoidDistance)
      avoidanceDirection = leftScanDirection;
    else if (rightAvoidDistance > leftAvoidDistance)
      avoidanceDirection = rightScanDirection;
    else
      avoidanceDirection = glm::vec2(0, 0);
    force += avoidanceDirection * steerPower;
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
