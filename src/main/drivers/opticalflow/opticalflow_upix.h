/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "drivers/rangefinder/rangefinder.h"
#include "drivers/opticalflow/opticalflow.h"
#include "sensors/rangefinder.h"

#define RANGEFINDER_UPIX_DETECTION_CONE_DECIDEGREES 900

typedef struct {
    rangefinderType_e deviceType;
    uint8_t delayMs;
    uint16_t maxRangeCm;
} UpixRangefinderConfig;

typedef struct {
    uint32_t timestampUs;
    int32_t distanceMm;
} upixRangefinderData_t;

bool upixRangefinderDetect(rangefinderDev_t * dev, rangefinderType_e upixRangefinderToUse);
void upixRangefinderReceiveNewData(const uint8_t * bufferPtr);
const UpixRangefinderConfig* getUpixRangefinderDeviceConf(rangefinderType_e upixRangefinderToUse);

bool upixOpticalflowDetect(opticalflowDev_t * dev, rangefinderType_e upixRangefinderToUse);
void upixOpticalflowReceiveNewData(const uint8_t * bufferPtr);

