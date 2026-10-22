#include "CurrentMonitor.h"
#include "Config.h"
#include "HardwareIO.h"
#include "JointMap.h"
#include <math.h>

// joints[] index whose result the NEXT transaction will return. -1 until
// the first transaction has primed the pipeline.
static int8_t monitorLastIdx = -1;
static unsigned long lastMonitorMs = 0;
static uint8_t monitorNextIdx = 0;   // round-robin cursor over PCA joints, PC mode only

// Decodes a raw 24-bit LTC2497 word into milliamps.
static float rawToMilliamps(uint32_t raw24) {
  uint32_t adc18 = (raw24 & 0x00FFFFC0) >> 5;
  uint16_t adc14 = adc18 & 0x3FFF;
  return (adc14 * (float)LTC2497_VREF_MV) / 65535.0f;
}

void currentMonitorSingleStep(JointId id) {
  if (!pca9685Present) return;
  if (id >= NUM_JOINTS) return;
  unsigned long now = millis();
  if (now - lastMonitorMs < LTC2497_CONV_MS) return;
  lastMonitorMs = now;

  uint32_t raw24 = hwLtc2497Transact(LTC2497_CH_CMD[joints[id].pcaChannel]);

  // Only trust this result if the PREVIOUS transaction also selected this
  // same joint's channel. The first call after switching joints is a
  // throwaway priming read; every call after that is valid.
  if (monitorLastIdx == (int8_t)id && raw24 != LTC2497_READ_FAILED) {
    joints[id].currentMA = rawToMilliamps(raw24);
  }

  monitorLastIdx = (int8_t)id;
}

// Wrist current. The three wrist motors share one INA219 on their supply
// rail, so all three report the same total. Called once per S reply rather
// than every loop pass — v2 read it on EVERY pass, which in PC mode meant
// thousands of I2C transactions a second for a value needed 30 times.
void currentMonitorSampleRail() {
  float rail = hwReadRailMilliamps();
  if (isnan(rail)) return;
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    if (joints[i].driver != DRV_PCA9685) joints[i].currentMA = rail;
  }
}

// Advances the round-robin cursor to the next joint that actually has an
// LTC2497 channel. v2 cycled through all 19 joints, spending 3 of every 19
// conversions (~480 ms per sweep) on wrist joints that aren't on the chip,
// and briefly writing channel 0's current into their slots.
static uint8_t nextPcaJoint(uint8_t from) {
  for (uint8_t k = 1; k <= NUM_JOINTS; k++) {
    uint8_t i = (uint8_t)((from + k) % NUM_JOINTS);
    if (joints[i].driver == DRV_PCA9685) return i;
  }
  return from;
}

void currentMonitorStep() {
  if (!pca9685Present) return;

  unsigned long now = millis();
  if (now - lastMonitorMs < LTC2497_CONV_MS) return;
  lastMonitorMs = now;

  if (joints[monitorNextIdx].driver != DRV_PCA9685) {
    monitorNextIdx = nextPcaJoint(monitorNextIdx);
  }

  uint32_t raw24 = hwLtc2497Transact(LTC2497_CH_CMD[joints[monitorNextIdx].pcaChannel]);

  // Pipelined: this transaction returns the channel selected LAST time, so
  // the result belongs to monitorLastIdx, not monitorNextIdx.
  if (monitorLastIdx >= 0 && raw24 != LTC2497_READ_FAILED &&
      joints[monitorLastIdx].driver == DRV_PCA9685) {
    joints[monitorLastIdx].currentMA = rawToMilliamps(raw24);
  }

  monitorLastIdx = (int8_t)monitorNextIdx;
  monitorNextIdx = nextPcaJoint(monitorNextIdx);
}
