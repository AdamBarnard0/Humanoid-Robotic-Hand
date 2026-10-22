#pragma once

// Boot-time hardware bring-up: Serial + pin init, I2C bus, PCA9685, an
// initial I2C scan, manual bounds, centering, and the first menu printout.
// Call once from setup().
void startupSequence();
