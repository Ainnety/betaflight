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
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */


 #include <stdint.h>
 #include <math.h>
 
 #include "platform.h"
 
 #ifdef USE_OPTICALFLOW
 
 #include "build/build_config.h"
 #include "build/debug.h"
 
 #include "common/maths.h"
 #include "common/time.h"
 #include "common/utils.h"
 #include "common/filter.h"
 
 #include "config/config.h"
 #include "config/feature.h"
 
 #include "fc/runtime_config.h"
 
 #include "scheduler/scheduler.h"
 
 #include "pg/pg.h"
 #include "pg/pg_ids.h"
 
 #include "drivers/time.h"
 #include "drivers/rangefinder/rangefinder.h"
 #include "drivers/rangefinder/rangefinder_lidarmt.h"
 #include "drivers/opticalflow/opticalflow_upix.h"
 
 #include "io/beeper.h"
 
 #include "sensors/sensors.h"
 #include "sensors/gyro.h"
 #include "sensors/acceleration.h"
 #include "sensors/opticalflow.h"
 
 #include "flight/imu.h"
 
 #define OPTICALFLOW_CALIBRATION_DURATION_MS 30000
 #define RATE_SCALE_RESOLUTION (1000.0f)
 
 // static prototypes
 static void applySensorRotation(vector2_t * dst, vector2_t * src);
 static void applyIMUAlignment(vector2_t * flowRates);
 static void applyLPF(vector2_t * flowRates);
 
 PG_REGISTER_WITH_RESET_TEMPLATE(opticalflowConfig_t, opticalflowConfig, PG_OPTICALFLOW_CONFIG, 0);
 
 PG_RESET_TEMPLATE(opticalflowConfig_t, opticalflowConfig,
     .opticalflow_hardware = OPTICALFLOW_NONE,
     .rotation = 0,
     .flip_x = 0,
     .flow_lpf = 0
 );
 
 static opticalflow_t opticalflow;
 float cosRotAngle = 1.0f;
 float sinRotAngle = 0.0f;
 static pt2Filter_t xFlowLpf, yFlowLpf;
 
 // ======================================================================
 // =================== Opticalflow Main Functions =======================
 // ======================================================================
 static bool opticalflowDetect(opticalflowDev_t * dev, uint8_t opticalflowHardwareToUse) {
     UNUSED(dev);
 
     opticalflowType_e opticalflowHardware = OPTICALFLOW_NONE;
 
     switch (opticalflowHardwareToUse) {
         case OPTICALFLOW_MT:
 #ifdef USE_RANGEFINDER_MT
             if (mtOpticalflowDetect(dev, rangefinderConfig()->rangefinder_hardware)) {
                 opticalflowHardware = OPTICALFLOW_MT;
                 rescheduleTask(TASK_OPTICALFLOW, TASK_PERIOD_MS(dev->delayMs));
             }
 #endif
             break;
         case OPTICALFLOW_UPIX:
 #if defined(USE_RANGEFINDER_UPIX)
             if (upixOpticalflowDetect(dev, rangefinderConfig()->rangefinder_hardware)) {
                 opticalflowHardware = OPTICALFLOW_UPIX;
                 rescheduleTask(TASK_OPTICALFLOW, TASK_PERIOD_MS(dev->delayMs));
             }
 #endif
             break;
 
         case OPTICALFLOW_NONE:
             opticalflowHardware = OPTICALFLOW_NONE;
             break;
     }
 
     if (opticalflowHardware == OPTICALFLOW_NONE) {
         sensorsClear(SENSOR_OPTICALFLOW);
         return false;
     }
 
     detectedSensors[SENSOR_INDEX_OPTICALFLOW] = opticalflowHardware;
     sensorsSet(SENSOR_OPTICALFLOW);
     return true;
 }
 
 bool opticalflowInit(void) {
     if (!opticalflowDetect(&opticalflow.dev, opticalflowConfig()->opticalflow_hardware)) {
         return false;
     }
 
     opticalflow.dev.init(&opticalflow.dev);
     opticalflow.quality = OPTICALFLOW_NO_NEW_DATA;
     opticalflow.rawFlowRates.x = 0;
     opticalflow.rawFlowRates.y = 0;
     opticalflow.processedFlowRates.x = 0;
     opticalflow.processedFlowRates.y = 0;
     opticalflow.timeStampUs = micros();
 
     cosRotAngle = cosf(DEGREES_TO_RADIANS(opticalflowConfig()->rotation));
     sinRotAngle = sinf(DEGREES_TO_RADIANS(opticalflowConfig()->rotation));
     //low pass filter
     if (opticalflowConfig()->flow_lpf != 0) {
         const float flowCutoffHz = (float)opticalflowConfig()->flow_lpf / 100.0f;
         const float flowGain     = pt2FilterGain(flowCutoffHz, opticalflow.dev.delayMs / 1000.0f);
 
         pt2FilterInit(&xFlowLpf, flowGain);
         pt2FilterInit(&yFlowLpf, flowGain);
     }
     return true;
 }
 
 void opticalflowUpdate(void) {
     if (opticalflow.dev.update) {
         opticalflow.dev.update(&opticalflow.dev);
     }
 }
 
 void opticalflowProcess(void) {
     opticalflowData_t data = {0};
     int32_t deltaTimeUs = 0;
     opticalflow.dev.read(&opticalflow.dev, &data);
 
     opticalflow.quality = data.quality;
     deltaTimeUs = cmp32(data.timeStampUs, opticalflow.timeStampUs);
 
     // 只接受正的时间间隔，防止时间戳回退或异常导致的负值 / 溢出
     if (deltaTimeUs > 0) { // New data
         vector2_t raw = data.flowRate;
         vector2_t processed;
 
         applySensorRotation(&processed, &raw);
         applyIMUAlignment(&processed);
         applyLPF(&processed);
 
         opticalflow.rawFlowRates = raw;
         opticalflow.processedFlowRates = processed;
         opticalflow.timeStampUs  = data.timeStampUs;
         opticalflow.deltaTimeUs  = deltaTimeUs;
 
         // DEBUG SECTION
         // DEBUG_SET(DEBUG_OPTICALFLOW, 0, opticalflow.quality);
         // DEBUG_SET(DEBUG_OPTICALFLOW, 1, lrintf(opticalflow.rawFlowRates.x * 1000));
         // DEBUG_SET(DEBUG_OPTICALFLOW, 2, lrintf(opticalflow.rawFlowRates.y * 1000));
         // DEBUG_SET(DEBUG_OPTICALFLOW, 3, lrintf(opticalflow.processedFlowRates.x * 1000));
         // DEBUG_SET(DEBUG_OPTICALFLOW, 4, lrintf(opticalflow.processedFlowRates.y * 1000));
         // DEBUG_SET(DEBUG_OPTICALFLOW, 5, deltaTimeUs);
 
         DEBUG_SET(DEBUG_OPTICALFLOW, 0, opticalflow.quality);
         DEBUG_SET(DEBUG_OPTICALFLOW, 1, lrintf(opticalflow.rawFlowRates.x ));
         DEBUG_SET(DEBUG_OPTICALFLOW, 2, lrintf(opticalflow.rawFlowRates.y ));
         DEBUG_SET(DEBUG_OPTICALFLOW, 3, lrintf(opticalflow.processedFlowRates.x ));
         DEBUG_SET(DEBUG_OPTICALFLOW, 4, lrintf(opticalflow.processedFlowRates.y ));
         DEBUG_SET(DEBUG_OPTICALFLOW, 5, deltaTimeUs);
     }
 }
 
 int32_t getOpticalflowDeltaTimeUs(void) {
     return opticalflow.deltaTimeUs;
 }
 
 static void applySensorRotation(vector2_t * dst, vector2_t * src) {
     dst->x = (opticalflowConfig()->flip_x ? -1.0f : 1.0f) * (src->x * cosRotAngle - src->y * sinRotAngle);
     dst->y = src->x * sinRotAngle + src->y * cosRotAngle;
 }
 
 static void applyIMUAlignment(vector2_t * flowRates) {
     // Apply tilt compensation: when the craft is tilted, optical flow measures
     // velocity perpendicular to the sensor plane, which needs to be compensated
     // by dividing by cos(tilt angle) to get the true ground velocity.
     const float cosTilt = getCosTiltAngle();
     
     // Avoid division by very small values (tilt > 84 degrees)
     // Data becomes unreliable at high tilt angles
     if (cosTilt > 0.1f) {
         flowRates->x /= cosTilt;
         flowRates->y /= cosTilt;
     } else {
         // Tilt too large, data unreliable - set to zero
         flowRates->x = 0.0f;
         flowRates->y = 0.0f;
     }
     
     // Note: Yaw rotation (body frame to earth frame) is handled in position control
     // where the velocity is converted using attitude.values.yaw
 }
 
 static void applyLPF(vector2_t * flowRates) {
     if (opticalflowConfig()->flow_lpf == 0) {
         return;
     }
 
     flowRates->x = pt2FilterApply(&xFlowLpf, flowRates->x);
     flowRates->y = pt2FilterApply(&yFlowLpf, flowRates->y);
 }
 
  const opticalflow_t * getOpticalflowData(void) {
     return &opticalflow;
 }
 
 bool isOpticalflowHealthy(void) {
     return cmp32(micros(), opticalflow.timeStampUs) < OPTICALFLOW_HARDWARE_TIMEOUT_US;
 }
 #endif // USE_OPTICALFLOW
 