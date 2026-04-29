#include "motion.h"

/**
 * Convert joint angles to absolute motor pulses
 * 
 * Kinematic joint angles are converted to motor positions.
 * Axis 2 and 3 are mechanically coupled:
 * - θ3_joint = θ3_motor + θ2_motor
 * - Therefore: θ3_motor = θ3_joint - θ2_motor = θ3_joint - θ2_joint
 */
AxisPulses jointAnglesToPulses(const JointAngles& joints) {
    AxisPulses pulses;
    
    // Axis 1: Direct conversion
    pulses.axis1 = (int32_t)(joints.axis1 * (AXIS1_PULSES_PER_90 / 90.0f)) + AXIS1_OFFSET;
    
    // Axis 2: Direct conversion
    pulses.axis2 = (int32_t)(joints.axis2 * (AXIS2_PULSES_PER_90 / 90.0f)) + AXIS2_OFFSET;
    
    // Axis 3: Account for mechanical coupling
    // Motor angle = Joint angle - Axis2 angle
    float axis3_motor = joints.axis3 - joints.axis2;
    pulses.axis3 = (int32_t)(axis3_motor * (AXIS3_PULSES_PER_90 / 90.0f)) + AXIS3_OFFSET;
    
    // Axis 4: Direct conversion
    pulses.axis4 = (int32_t)(joints.axis4 * (AXIS4_PULSES_PER_90 / 90.0f)) + AXIS4_OFFSET;
    
    // Axis 5: Direct conversion
    pulses.axis5 = (int32_t)(joints.axis5 * (AXIS5_PULSES_PER_90 / 90.0f)) + AXIS5_OFFSET;
    
    // Axis 6: Direct conversion
    pulses.axis6 = (int32_t)(joints.axis6 * (AXIS6_PULSES_PER_90 / 90.0f)) + AXIS6_OFFSET;
    
    return pulses;
}

/**
 * Convert absolute motor pulses to joint angles
 * 
 * Motor positions are converted to kinematic joint angles.
 * Axis 2 and 3 are mechanically coupled:
 * - θ3_joint = θ3_motor + θ2_motor (= θ2_joint)
 */
JointAngles pulsesToJointAngles(const AxisPulses& pulses) {
    JointAngles joints;
    
    // Axis 1: Direct conversion
    joints.axis1 = (float)(pulses.axis1 - AXIS1_OFFSET) / (AXIS1_PULSES_PER_90 / 90.0f);
    
    // Axis 2: Direct conversion
    joints.axis2 = (float)(pulses.axis2 - AXIS2_OFFSET) / (AXIS2_PULSES_PER_90 / 90.0f);
    
    // Axis 3: Account for mechanical coupling
    // First get motor angle, then add axis2 joint angle
    float axis3_motor = (float)(pulses.axis3 - AXIS3_OFFSET) / (AXIS3_PULSES_PER_90 / 90.0f);
    joints.axis3 = axis3_motor + joints.axis2;
    
    // Axis 4: Direct conversion
    joints.axis4 = (float)(pulses.axis4 - AXIS4_OFFSET) / (AXIS4_PULSES_PER_90 / 90.0f);
    
    // Axis 5: Direct conversion
    joints.axis5 = (float)(pulses.axis5 - AXIS5_OFFSET) / (AXIS5_PULSES_PER_90 / 90.0f);
    
    // Axis 6: Direct conversion
    joints.axis6 = (float)(pulses.axis6 - AXIS6_OFFSET) / (AXIS6_PULSES_PER_90 / 90.0f);
    
    return joints;
}
