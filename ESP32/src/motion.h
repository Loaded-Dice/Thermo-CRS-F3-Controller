#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>

// Factory offsets from "home" position (in pulses)
#define AXIS1_OFFSET  569
#define AXIS2_OFFSET  1271
#define AXIS3_OFFSET  2036
#define AXIS4_OFFSET  1334
#define AXIS5_OFFSET  60
#define AXIS6_OFFSET  1396

// Pulses per 90 degrees
#define AXIS1_PULSES_PER_90  -51200
#define AXIS2_PULSES_PER_90   51200
#define AXIS3_PULSES_PER_90   51200
#define AXIS4_PULSES_PER_90  -40960
#define AXIS5_PULSES_PER_90   40960
#define AXIS6_PULSES_PER_90  -40960

// Structure for absolute motor pulses (encoder positions)
typedef struct {
    int32_t axis1;
    int32_t axis2;
    int32_t axis3;
    int32_t axis4;
    int32_t axis5;
    int32_t axis6;
} AxisPulses;

// Structure for joint angles in degrees (kinematic representation)
typedef struct {
    float axis1;
    float axis2;
    float axis3;
    float axis4;
    float axis5;
    float axis6;
} JointAngles;

/**
 * @brief Convert joint angles to absolute motor pulses
 * 
 * Takes kinematic joint angles and converts them to absolute motor positions.
 * Accounts for mechanical coupling between axis 2 and axis 3.
 * 
 * @param joints Joint angles in degrees
 * @return AxisPulses Absolute motor pulse positions
 * 
 * @note When joints = {0,0,0,0,0,0}, returns the offset values
 */
AxisPulses jointAnglesToPulses(const JointAngles& joints);

/**
 * @brief Convert absolute motor pulses to joint angles
 * 
 * Takes absolute motor positions and converts them to kinematic joint angles.
 * Accounts for mechanical coupling between axis 2 and axis 3.
 * 
 * @param pulses Absolute motor pulse positions
 * @return JointAngles Joint angles in degrees
 * 
 * @note When pulses = offsets, returns {0,0,0,0,0,0}
 */
JointAngles pulsesToJointAngles(const AxisPulses& pulses);

#endif // MOTION_H
