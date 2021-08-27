/*
 * Copyright (C) 2018 Ola Benderius
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "behavior.hpp"
#include <cmath>

Behavior::Behavior() noexcept:
  m_steeringGain(0.5f),
  m_steeringDeadZone(0.05f),
  m_maxSteeringAngle(0.2f),
  m_defaultPedalPosition(0.09f),
  m_maxPedalPosition(0.12f),
  m_frontUltrasonicReading{},
  m_rearUltrasonicReading{},
  m_leftIrReading{},
  m_rightIrReading{},
  m_groundSteeringAngleRequest{},
  m_pedalPositionRequest{},
  m_angleReading{},
  m_frontUltrasonicReadingMutex{},
  m_rearUltrasonicReadingMutex{},
  m_leftIrReadingMutex{},
  m_rightIrReadingMutex{},
  m_groundSteeringAngleRequestMutex{},
  m_pedalPositionRequestMutex{},
  m_angleReadingMutex{}
{
}

Behavior::Behavior(float maxSteer, float defaultPedal, float maxPedal, float steerGain, float steeringDeadZone) :
  Behavior::Behavior() {
  m_maxSteeringAngle = maxSteer;
  m_defaultPedalPosition = defaultPedal;
  m_maxPedalPosition = maxPedal;
  m_steeringGain = steerGain;
  m_steeringDeadZone = steeringDeadZone;
}

opendlv::proxy::GroundSteeringRequest Behavior::getGroundSteeringAngle() noexcept
{
  std::lock_guard<std::mutex> lock(m_groundSteeringAngleRequestMutex);
  return m_groundSteeringAngleRequest;
}

opendlv::proxy::PedalPositionRequest Behavior::getPedalPositionRequest() noexcept
{
  std::lock_guard<std::mutex> lock(m_pedalPositionRequestMutex);
  return m_pedalPositionRequest;
}

void Behavior::setFrontUltrasonic(opendlv::proxy::DistanceReading const &frontUltrasonicReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_frontUltrasonicReadingMutex);
  m_frontUltrasonicReading = frontUltrasonicReading;
}

void Behavior::setRearUltrasonic(opendlv::proxy::DistanceReading const &rearUltrasonicReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_rearUltrasonicReadingMutex);
  m_rearUltrasonicReading = rearUltrasonicReading;
}

void Behavior::setLeftIr(opendlv::proxy::VoltageReading const &leftIrReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_leftIrReadingMutex);
  m_leftIrReading = leftIrReading;
}

void Behavior::setRightIr(opendlv::proxy::VoltageReading const &rightIrReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_rightIrReadingMutex);
  m_rightIrReading = rightIrReading;
}

void Behavior::setAngleReading(opendlv::proxy::AngleReading const &angleReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_angleReadingMutex);
  m_angleReading = angleReading;
}


void Behavior::step() noexcept
{
  opendlv::proxy::DistanceReading frontUltrasonicReading;
  opendlv::proxy::DistanceReading rearUltrasonicReading;
  opendlv::proxy::VoltageReading leftIrReading;
  opendlv::proxy::VoltageReading rightIrReading;
  opendlv::proxy::AngleReading angleReading;
  {
    std::lock_guard<std::mutex> lock1(m_frontUltrasonicReadingMutex);
    std::lock_guard<std::mutex> lock2(m_rearUltrasonicReadingMutex);
    std::lock_guard<std::mutex> lock3(m_leftIrReadingMutex);
    std::lock_guard<std::mutex> lock4(m_rightIrReadingMutex);
    std::lock_guard<std::mutex> lock5(m_angleReadingMutex);

    frontUltrasonicReading = m_frontUltrasonicReading;
    rearUltrasonicReading = m_rearUltrasonicReading;
    leftIrReading = m_leftIrReading;
    rightIrReading = m_rightIrReading;
    angleReading = m_angleReading;
  }

  float angle = angleReading.angle();
  angle = angle * -1;

  float pedalPosition = 0.2f;
  float groundSteeringAngle = 0.0f;


  //---- own control implementation ----
  //TODO: Use as command line arguments
  // float maxSteeringAngle = 0.5f;
  // float steeringGain = 0.5f;

  //TODO: Try to implement a deadzone for small angles
  // groundSteeringAngle = m_steeringGain * angle;
  // if (groundSteeringAngle > m_maxSteeringAngle) {
  //   groundSteeringAngle = m_maxSteeringAngle ;
  // } else if (groundSteeringAngle < -m_maxSteeringAngle) {
  //   groundSteeringAngle = -m_maxSteeringAngle;
  // }

  // Steering control
  // slightly advanced steering control with deadzone and quadratic relationship to aimpoint angle
  if (angle > 0) {
    if (angle > m_steeringDeadZone) {
      groundSteeringAngle = m_steeringGain * static_cast<float> (std::pow((angle - m_steeringDeadZone),2));
    }
  } else if (angle < 0) {
    if (angle < -m_steeringDeadZone) {
      groundSteeringAngle = m_steeringGain * -static_cast<float> (std::pow((angle - m_steeringDeadZone),2));
    }
  }

  // Steering limiter
  if (groundSteeringAngle > m_maxSteeringAngle) {
    groundSteeringAngle = m_maxSteeringAngle ;
  } else if (groundSteeringAngle < -m_maxSteeringAngle) {
    groundSteeringAngle = -m_maxSteeringAngle;
  }

  // Simple Longitudinal Control
  pedalPosition = m_maxPedalPosition - (m_maxPedalPosition - m_defaultPedalPosition) * (std::abs(groundSteeringAngle) / m_maxSteeringAngle);

  if (pedalPosition > m_maxPedalPosition) {
    pedalPosition = m_maxPedalPosition;
  }

  {
    std::lock_guard<std::mutex> lock1(m_groundSteeringAngleRequestMutex);
    std::lock_guard<std::mutex> lock2(m_pedalPositionRequestMutex);

    opendlv::proxy::GroundSteeringRequest groundSteeringAngleRequest;
    groundSteeringAngleRequest.groundSteering(groundSteeringAngle);
    m_groundSteeringAngleRequest = groundSteeringAngleRequest;

    opendlv::proxy::PedalPositionRequest pedalPositionRequest;
    pedalPositionRequest.position(pedalPosition);
    m_pedalPositionRequest = pedalPositionRequest;
  }
}

// TODO: This is a rough estimate, improve by looking into the sensor specifications.
double Behavior::convertIrVoltageToDistance(float voltage) const noexcept
{
  double voltageDividerR1 = 1000.0;
  double voltageDividerR2 = 1000.0;

  double sensorVoltage = (voltageDividerR1 + voltageDividerR2) / voltageDividerR2 * voltage;
  double distance = (2.5 - sensorVoltage) / 0.07;
  return distance;
}
