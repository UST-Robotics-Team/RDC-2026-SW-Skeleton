#ifndef __ROBOTICS_LOGO_H
#define __ROBOTICS_LOGO_H

#include <stdint.h>

#define ROBOTICS_IMG_WIDTH   128
#define ROBOTICS_IMG_HEIGHT  85

/**
 * @brief Downsampled HKUST Robotics logo (RGB565 big-endian byte array for ST7735).
 * Dimensions: 128x85 pixels (21760 bytes).
 */
extern const uint8_t robotics_img[21760];

#endif /* __ROBOTICS_LOGO_H */
