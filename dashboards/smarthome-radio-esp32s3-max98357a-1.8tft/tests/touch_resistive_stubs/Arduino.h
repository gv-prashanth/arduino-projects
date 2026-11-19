#pragma once
#include <cassert>
#include <cmath>
#include <cstdint>
#include <deque>
#include <vector>

namespace FakeResistive {
enum { Floating = 0, Input = 1, Output = 2, Analog = 3 };
struct Pin { int mode = Floating, level = 0; bool pullup = false; };
struct Plate { int first, second; double ohms; };
struct State {
  Pin pins[49];
  Plate plates[2] = {{8, 15, 300.0}, {16, 18, 500.0}};
  double positions[2] = {0.25, 0.75};
  bool down = false, displayActive = false, denyUnit[2] = {false, false};
  unsigned gpioCalls = 0, gpioFailAt = 0, configureCalls = 0, configureFailAt = 0;
  unsigned readCalls = 0, readFailAt = 0, newCalls[2] = {}, delCalls[2] = {};
  unsigned delayTotal = 0, contactReads = 0, writeEdges = 0;
  unsigned digitalReadFailAt = 0, liftAtAdcRead = 0;
  int noiseAmplitude = 0;
  std::deque<int> samples;
  std::vector<int> sampledPins, configuredPins, changedPins;
};
inline State& state() { static State value; return value; }
inline void reset() {
  state() = State{};
  // RadioApp initializes TFT_eSPI before its touch reader. Preserve the
  // untouched display pads as outputs rather than inventing a reset-state
  // contract for GPIOs that the reader never owns.
  for (int pin : {7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 21, 38})
    state().pins[pin].mode = Output;
  for (int pin : {7, 15, 16, 17, 21, 38}) state().pins[pin].level = 1;
}
inline bool allowed(int pin) {
  return pin == 7 || pin == 8 || pin == 9 || pin == 10 || pin == 11 || pin == 12 ||
      pin == 13 || pin == 14 || pin == 15 || pin == 16 || pin == 17 || pin == 18 ||
      pin == 21 || pin == 38;
}
inline void checkPin(int pin) {
  assert(pin >= 0 && pin < 49 && allowed(pin));
  state().changedPins.push_back(pin);
}
inline bool digitalLine(int pin) {
  const auto& p = state().pins[pin];
  return p.mode == Output ? p.level != 0 : p.pullup;
}
inline void checkWriteEdge(bool before) {
  auto& s = state();
  if (s.displayActive && !before && digitalLine(7) && !digitalLine(16)) ++s.writeEdges;
}

// A passive electrical model, rather than canned readings for expected GPIO
// sequences: two resistive plates and a finger joining their contact points.
// Output drivers and pull-ups feed a nodal network; ADC reads its voltage.
inline double voltage(int pin) {
  const int nodes = 51;
  double matrix[nodes][nodes + 1] = {};
  auto conduct = [&](int first, int second, double resistance) {
    const double g = 1.0 / resistance;
    matrix[first][first] += g; matrix[second][second] += g;
    matrix[first][second] -= g; matrix[second][first] -= g;
  };
  const auto& s = state();
  for (int i = 0; i < nodes; ++i) matrix[i][i] += 1e-12;
  for (int i = 0; i < 49; ++i) {
    const auto& p = s.pins[i];
    if (p.mode == Output) {
      matrix[i][i] += 1e3;
      matrix[i][nodes] += p.level ? 1e3 : 0.0;
    } else if (p.pullup) {
      matrix[i][i] += 1.0 / 45000.0;
      matrix[i][nodes] += 1.0 / 45000.0;
    }
  }
  for (int axis = 0; axis < 2; ++axis) {
    const auto& plate = s.plates[axis];
    if (plate.first < 0 || plate.second < 0) continue;
    const double position = s.positions[axis];
    conduct(plate.first, 49 + axis, plate.ohms * position);
    conduct(49 + axis, plate.second, plate.ohms * (1.0 - position));
  }
  if (s.down) conduct(49, 50, 600.0);
  for (int pivot = 0; pivot < nodes; ++pivot) {
    int best = pivot;
    for (int row = pivot + 1; row < nodes; ++row)
      if (std::fabs(matrix[row][pivot]) > std::fabs(matrix[best][pivot])) best = row;
    assert(std::fabs(matrix[best][pivot]) > 1e-15);
    if (best != pivot)
      for (int column = pivot; column <= nodes; ++column) {
        const double temporary = matrix[pivot][column];
        matrix[pivot][column] = matrix[best][column]; matrix[best][column] = temporary;
      }
    const double divisor = matrix[pivot][pivot];
    for (int column = pivot; column <= nodes; ++column) matrix[pivot][column] /= divisor;
    for (int row = pivot + 1; row < nodes; ++row) {
      const double factor = matrix[row][pivot];
      if (std::fabs(factor) < 1e-20) continue;
      for (int column = pivot; column <= nodes; ++column)
        matrix[row][column] -= factor * matrix[pivot][column];
    }
  }
  double result[nodes] = {};
  for (int row = nodes - 1; row >= 0; --row) {
    result[row] = matrix[row][nodes];
    for (int column = row + 1; column < nodes; ++column) result[row] -= matrix[row][column] * result[column];
  }
  return result[pin];
}
}
inline void delayMicroseconds(unsigned duration) { FakeResistive::state().delayTotal += duration; }
#ifndef bit
#define bit(b) (1UL << (b))
#endif
