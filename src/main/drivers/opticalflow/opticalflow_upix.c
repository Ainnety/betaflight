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

/*
 * Upix rangefinder/opticalflow bridge for MSP-fed sensors.
 * Mirrors the MT bridge behaviour: distances via MSP, reuse the
 * same data for opticalflow quality gating.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_RANGEFINDER_UPIX

#include "build/build_config.h"

#include "common/utils.h"

#include "drivers/time.h"

#include "drivers/opticalflow/opticalflow_upix.h"
#include "sensors/rangefinder.h"

#include "drivers/opticalflow/opticalflow.h"

#define UPIX_OPTICALFLOW_MIN_RANGE 20            // mm, mirror MT behaviour
#define UPIX_OPTICALFLOW_MIN_QUALITY_THRESHOLD 30

static opticalflowData_t opticalflowSensorData = {0};

static bool hasRFNewData = false;
static upixRangefinderData_t rfSensorData = {RANGEFINDER_NO_NEW_DATA, 0};
static const UpixRangefinderConfig * deviceConf = NULL;
static uint32_t lastOpticalflowTimeStampUs = 0;

// TODO: replace placeholder ranges with real Upix model specs
static const UpixRangefinderConfig rangefinderConfigs[] = {
    { .deviceType = RANGEFINDER_UPIX, .delayMs = 20, .maxRangeCm = 400}, // placeholder
};
 
typedef struct __attribute__((packed)) {
    uint8_t quality;    // [0;255]
    int32_t distanceMm; // Negative value for out of range
} mspSensorRangefinderUpixDataMessage_t;
 
 static void upixRangefinderInit(rangefinderDev_t * dev) {
     UNUSED(dev);
 }
 
 static void upixRangefinderUpdate(rangefinderDev_t * dev) {
     UNUSED(dev);
 }
 
 static int32_t upixRangefinderGetDistance(rangefinderDev_t * dev) {
     UNUSED(dev);
     if (hasRFNewData) {
         hasRFNewData = false;
         return (rfSensorData.distanceMm >= 0) ? (rfSensorData.distanceMm / 10) : RANGEFINDER_OUT_OF_RANGE;
     } else {
         return RANGEFINDER_NO_NEW_DATA;
     }
 }
 
 bool upixRangefinderDetect(rangefinderDev_t * dev, rangefinderType_e upixRangefinderToUse) {
     deviceConf = getUpixRangefinderDeviceConf(upixRangefinderToUse);
     if (!deviceConf) {
         return false;
     }
 
     dev->delayMs    = deviceConf->delayMs;
     dev->maxRangeCm = deviceConf->maxRangeCm;
 
     dev->detectionConeDeciDegrees = RANGEFINDER_UPIX_DETECTION_CONE_DECIDEGREES;
     dev->detectionConeExtendedDeciDegrees = RANGEFINDER_UPIX_DETECTION_CONE_DECIDEGREES;
 
     dev->init   = &upixRangefinderInit;
     dev->update = &upixRangefinderUpdate;
     dev->read   = &upixRangefinderGetDistance;
     return true;
 }
 
 void upixRangefinderReceiveNewData(const uint8_t * bufferPtr) {
     const mspSensorRangefinderUpixDataMessage_t * pkt = (const mspSensorRangefinderUpixDataMessage_t *)bufferPtr;
 
     rfSensorData.distanceMm = pkt->distanceMm;
     rfSensorData.timestampUs = micros();
     hasRFNewData = true;
 }
 
 const UpixRangefinderConfig* getUpixRangefinderDeviceConf(rangefinderType_e upixRangefinderToUse) {
    for (const UpixRangefinderConfig* cfg =  rangefinderConfigs; cfg < ARRAYEND(rangefinderConfigs); cfg++) {
         if (cfg->deviceType == upixRangefinderToUse) {
             return cfg;
         }
     }
     return NULL;
 }
 
 static const upixRangefinderData_t * getUpixRangefinderData(void) {
     return &rfSensorData;
 }
 
 typedef struct __attribute__((packed)) {
     uint8_t quality;    // [0;255]
     int32_t motionX;
     int32_t motionY;
 } upixOpticalflowDataMessage_t;
 
 static void upixOpticalflowInit(opticalflowDev_t * dev) {
     UNUSED(dev);
 }
 
 static void upixOpticalflowUpdate(opticalflowDev_t * dev) {
     UNUSED(dev);
 }
 
 static void upixOpticalflowGetData(opticalflowDev_t * dev, opticalflowData_t * result) {
     UNUSED(dev);
     *result = opticalflowSensorData;
 }
 
 bool upixOpticalflowDetect(opticalflowDev_t * dev, rangefinderType_e upixRangefinderToUse) {
     deviceConf = getUpixRangefinderDeviceConf(upixRangefinderToUse);
     if (!deviceConf) {
         return false;
     }
 
     dev->delayMs = deviceConf->delayMs;
     dev->minRangeCm = UPIX_OPTICALFLOW_MIN_RANGE;               // mm gate, same as MT
     dev->minQualityThreshold = UPIX_OPTICALFLOW_MIN_QUALITY_THRESHOLD;
 
     dev->init   = &upixOpticalflowInit;
     dev->update = &upixOpticalflowUpdate;
     dev->read   = &upixOpticalflowGetData;
 
     return true;
 }
 
void upixOpticalflowReceiveNewData(const uint8_t * bufferPtr) {
   const upixRangefinderData_t * latestRangefinderData = getUpixRangefinderData();

    const upixOpticalflowDataMessage_t * pkt = (const upixOpticalflowDataMessage_t *)bufferPtr;

    uint32_t currentTimeUs = micros();
    opticalflowSensorData.timeStampUs = currentTimeUs;
    
    // Calculate time interval in seconds
    float deltaTimeS = 0.0f;
    if (lastOpticalflowTimeStampUs != 0) {
        uint32_t deltaTimeUs = cmp32(currentTimeUs, lastOpticalflowTimeStampUs);
        if (deltaTimeUs > 0) {
            deltaTimeS = (float)deltaTimeUs / 1000000.0f;
        }
    }
    lastOpticalflowTimeStampUs = currentTimeUs;
    
    // Convert from pixels to rad/s: (pixel * 36 / 128 / 10000) / time
    // Step 1: Convert pixel to rad: pixel * 36 / 128 / 10000
    // Step 2: Convert rad to rad/s: rad / deltaTimeS
    // Formula: motionX * 36 / 128 / 10000 / deltaTimeS = motionX * 36 / (128 * 10000 * deltaTimeS)
    if (deltaTimeS > 0.0f) {
        const float conversionFactor = 36.0f / (128.0f * 10000.0f);
        opticalflowSensorData.flowRate.x  = (float)pkt->motionX * conversionFactor / deltaTimeS;
        opticalflowSensorData.flowRate.y  = (float)pkt->motionY * conversionFactor / deltaTimeS;
    } else {
        // First frame or invalid time interval, set to zero
        opticalflowSensorData.flowRate.x  = 0.0f;
        opticalflowSensorData.flowRate.y  = 0.0f;
    }
    opticalflowSensorData.quality = pkt->quality * 100 / 255;
 
    // distance is reported in mm; gate against fixed 80 mm (same as MT)
    if (latestRangefinderData->distanceMm < UPIX_OPTICALFLOW_MIN_RANGE) {
         opticalflowSensorData.quality = OPTICALFLOW_OUT_OF_RANGE;
     } else if (cmp32(micros(), latestRangefinderData->timestampUs) > (5000 * deviceConf->delayMs)) {   // 5 updates missing
         opticalflowSensorData.quality = OPTICALFLOW_HARDWARE_FAILURE;
     }
 }
 #endif // USE_RANGEFINDER_UPIX
 
 