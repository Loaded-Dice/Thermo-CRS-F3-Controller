#include "kinematics.h"
#include <cmath>
#include <cstring>

// Inline wrappers for consistency
inline float cosf_compat(float x) { return cosf(x); }
inline float sinf_compat(float x) { return sinf(x); }

// Matrix multiplication: m1[rows1 x cols1_rows2] * m2[cols1_rows2 x cols2] = out[rows1 x cols2]
void CRS_Kinematics::matMultiply(const float* m1, const float* m2, float* out,
                                  int rows1, int cols1_rows2, int cols2) {
    for (int i = 0; i < rows1; i++) {
        for (int j = 0; j < cols2; j++) {
            float sum = 0.0f;
            for (int k = 0; k < cols1_rows2; k++) {
                sum += m1[i * cols1_rows2 + k] * m2[k * cols2 + j];
            }
            out[i * cols2 + j] = sum;
        }
    }
}

// Convert rotation matrix to Euler angles (ZYX convention)
void CRS_Kinematics::rotMatToEulerAngle(const float* R, float* euler) {
    float A, B, C, cb;
    
    if (fabsf(R[6]) >= 1.0f - 0.0001f) {
        // Gimbal lock case
        if (R[6] < 0.0f) {
            B = M_PI_VAL / 2.0f;
            A = 0.0f;
            C = atan2f(R[1], R[4]);
        } else {
            B = -M_PI_VAL / 2.0f;
            A = 0.0f;
            C = atan2f(-R[1], R[4]);
        }
    } else {
        B = -asinf(R[6]);
        cb = cosf(B);
        A = atan2f(R[3] / cb, R[0] / cb);
        C = atan2f(R[7] / cb, R[8] / cb);
    }
    
    euler[0] = C; // Z-rotation
    euler[1] = B; // Y-rotation
    euler[2] = A; // X-rotation
}

// Convert Euler angles to rotation matrix (ZYX convention)
void CRS_Kinematics::eulerAngleToRotMat(const float* euler, float* R) {
    float ca = cosf(euler[2]);
    float cb = cosf(euler[1]);
    float cc = cosf(euler[0]);
    float sa = sinf(euler[2]);
    float sb = sinf(euler[1]);
    float sc = sinf(euler[0]);
    
    R[0] = ca * cb;
    R[1] = ca * sb * sc - sa * cc;
    R[2] = ca * sb * cc + sa * sc;
    R[3] = sa * cb;
    R[4] = sa * sb * sc + ca * cc;
    R[5] = sa * sb * cc - ca * sc;
    R[6] = -sb;
    R[7] = cb * sc;
    R[8] = cb * cc;
}

// Compute Modified DH transformation matrix for a joint
void CRS_Kinematics::computeDHMatrix(int joint, float theta_rad, float* T) {
    // Modified DH: T = Rx(α_{i-1}) * Tx(a_{i-1}) * Rz(θ_i) * Tz(d_i)
    
    const DHParams& curr = dh[joint];
    float theta = theta_rad + curr.theta_offset;
    float ct = cosf(theta);
    float st = sinf(theta);
    
    // Get previous joint's parameters (for Modified DH)
    float a_prev = (joint == 0) ? 0.0f : dh[joint - 1].a;
    float alpha_prev = (joint == 0) ? 0.0f : dh[joint - 1].alpha;
    float ca = cosf(alpha_prev);
    float sa = sinf(alpha_prev);
    
    // Build transformation matrix (4x4, row-major)
    memset(T, 0, 16 * sizeof(float));
    
    T[0]  = ct;
    T[1]  = -st;
    T[2]  = 0.0f;
    T[3]  = a_prev;
    
    T[4]  = st * ca;
    T[5]  = ct * ca;
    T[6]  = -sa;
    T[7]  = -curr.d * sa;
    
    T[8]  = st * sa;
    T[9]  = ct * sa;
    T[10] = ca;
    T[11] = curr.d * ca;
    
    T[12] = 0.0f;
    T[13] = 0.0f;
    T[14] = 0.0f;
    T[15] = 1.0f;
}

CRS_Kinematics::CRS_Kinematics() {
    // CRS F3 Robot Arm - Modified DH Parameters
    // Joint 1: Base rotation
    dh[0].a = 100.0f;
    dh[0].alpha = M_PI_VAL / 2.0f;  // 90°
    dh[0].d = 350.0f;
    dh[0].theta_offset = 0.0f;
    
    // Joint 2: Shoulder
    dh[1].a = 265.0f;
    dh[1].alpha = 0.0f;
    dh[1].d = 0.0f;
    dh[1].theta_offset = M_PI_VAL / 2.0f;  // 90°
    
    // Joint 3: Elbow (mechanically coupled with J2)
    dh[2].a = 0.0f;
    dh[2].alpha = M_PI_VAL / 2.0f;  // 90°
    dh[2].d = 0.0f;
    dh[2].theta_offset = M_PI_VAL / 2.0f;  // 90°
    
    // Joint 4: Wrist roll
    dh[3].a = 0.0f;
    dh[3].alpha = -M_PI_VAL / 2.0f;  // -90°
    dh[3].d = 270.0f;
    dh[3].theta_offset = 0.0f;
    
    // Joint 5: Wrist pitch
    dh[4].a = 0.0f;
    dh[4].alpha = M_PI_VAL / 2.0f;  // 90°
    dh[4].d = 0.0f;
    dh[4].theta_offset = 0.0f;
    
    // Joint 6: Wrist yaw
    dh[5].a = 0.0f;
    dh[5].alpha = 0.0f;
    dh[5].d = 90.0f;
    dh[5].theta_offset = 0.0f;
    
    // Precompute geometric values
    l_ew = sqrtf(dh[3].d * dh[3].d);
    atan_e = 0.0f;
}

bool CRS_Kinematics::solveFK(const Joint6D& joints, Pose6D& pose) {
    // Convert joint angles to motor angles (account for J2-J3 coupling)
    // J3_motor = J3_joint - J2_joint
    float q_motor[6];
    q_motor[0] = joints.a[0] * K_DEG_TO_RAD;
    q_motor[1] = joints.a[1] * K_DEG_TO_RAD;
    q_motor[2] = (joints.a[2] - joints.a[1]) * K_DEG_TO_RAD;  // Mechanical coupling
    q_motor[3] = joints.a[3] * K_DEG_TO_RAD;
    q_motor[4] = joints.a[4] * K_DEG_TO_RAD;
    q_motor[5] = joints.a[5] * K_DEG_TO_RAD;
    
    // Compute transformation matrices for each joint
    float T[7][16];  // T[0] = identity, T[1..6] = joint transforms
    
    // T[0] = Identity
    memset(T[0], 0, 16 * sizeof(float));
    T[0][0] = T[0][5] = T[0][10] = T[0][15] = 1.0f;
    
    // Compute cumulative transformations
    for (int i = 0; i < 6; i++) {
        float T_joint[16];
        computeDHMatrix(i, q_motor[i], T_joint);
        matMultiply(T[i], T_joint, T[i + 1], 4, 4, 4);
    }
    
    // Extract position from T[6]
    pose.X = T[6][3];
    pose.Y = T[6][7];
    pose.Z = T[6][11];
    
    // Extract rotation matrix (3x3 upper-left of T[6])
    pose.R[0] = T[6][0];  pose.R[1] = T[6][1];  pose.R[2] = T[6][2];
    pose.R[3] = T[6][4];  pose.R[4] = T[6][5];  pose.R[5] = T[6][6];
    pose.R[6] = T[6][8];  pose.R[7] = T[6][9];  pose.R[8] = T[6][10];
    pose.hasR = true;
    
    // Convert rotation matrix to Euler angles
    float euler[3];
    rotMatToEulerAngle(pose.R, euler);
    pose.C = euler[0] * K_RAD_TO_DEG;  // Z-rotation
    pose.B = euler[1] * K_RAD_TO_DEG;  // Y-rotation
    pose.A = euler[2] * K_RAD_TO_DEG;  // X-rotation
    
    return true;
}

bool CRS_Kinematics::solvePositionIK(float wx, float wy, float wz, 
                                      float q1[2], float q2[2][2], float q3[2][2]) {
    // Step 1: Solve J1 (base rotation) - 2 solutions (±180°)
    q1[0] = atan2f(wy, wx);
    q1[1] = q1[0] + M_PI_VAL;
    if (q1[1] > M_PI_VAL) q1[1] -= 2.0f * M_PI_VAL;
    
    // For each J1 solution, solve J2 and J3
    for (int i = 0; i < 2; i++) {
        // Project wrist center into shoulder plane
        float c1 = cosf(q1[i]);
        float s1 = sinf(q1[i]);
        
        // Wrist position in base frame, accounting for base offset
        float wx_local = wx * c1 + wy * s1 - dh[0].a;  // Subtract base offset
        float wz_local = wz - dh[0].d;  // Subtract base height
        
        // Distance from shoulder to wrist in vertical plane
        float r = sqrtf(wx_local * wx_local + wz_local * wz_local);
        
        // Link lengths for 2D problem
        float L2 = dh[1].a;  // Upper arm = 265mm
        float L3 = dh[3].d;  // Forearm = 270mm (from J4 offset)
        
        // Check if position is reachable
        if (r > L2 + L3 || r < fabsf(L2 - L3)) {
            // Out of reach
            q2[i][0] = q2[i][1] = 0.0f;
            q3[i][0] = q3[i][1] = 0.0f;
            continue;
        }
        
        // Law of cosines for elbow angle
        // cos(q3) = (r² - L2² - L3²) / (2 * L2 * L3)
        float cos_q3 = (r * r - L2 * L2 - L3 * L3) / (2.0f * L2 * L3);
        cos_q3 = fmaxf(-1.0f, fminf(1.0f, cos_q3));  // Clamp to [-1, 1]
        
        // Two elbow solutions: elbow up and elbow down
        float q3_temp_up = acosf(cos_q3);
        float q3_temp_down = -q3_temp_up;
        
        // For each elbow configuration, solve shoulder angle
        for (int elbow = 0; elbow < 2; elbow++) {
            float q3_temp = (elbow == 0) ? q3_temp_up : q3_temp_down;
            
            // Shoulder angle from geometry
            float alpha = atan2f(wz_local, wx_local);
            float beta = atan2f(L3 * sinf(q3_temp), L2 + L3 * cosf(q3_temp));
            float q2_temp = alpha - beta;
            
            // Store solutions (account for DH offset of 90° on J2)
            q2[i][elbow] = q2_temp - dh[1].theta_offset;
            
            // For J3: Due to mechanical coupling, J3_motor = J3_joint - J2_joint
            // But in DH we need the motor angle
            // The kinematic chain angle q3_temp is already the motor angle here
            q3[i][elbow] = q3_temp - dh[2].theta_offset;
        }
    }
    
    return true;
}

bool CRS_Kinematics::solveOrientationIK(const float* R_0_3, const float* R_desired,
                                         float q4[2], float q5[2], float q6[2]) {
    // Compute wrist orientation: R_3_6 = R_0_3^T * R_desired
    float R_3_6[9];
    
    // R_0_3^T * R_desired (3x3 matrix multiplication)
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R_3_6[i * 3 + j] = 0.0f;
            for (int k = 0; k < 3; k++) {
                R_3_6[i * 3 + j] += R_0_3[k * 3 + i] * R_desired[k * 3 + j];  // Transpose of R_0_3
            }
        }
    }
    
    // Extract Euler angles from R_3_6 (ZYZ Euler angles for spherical wrist)
    // This is the standard decomposition for a spherical wrist
    
    float r31 = R_3_6[6];  // Element (3,1)
    float r32 = R_3_6[7];  // Element (3,2)
    float r33 = R_3_6[8];  // Element (3,3)
    float r13 = R_3_6[2];  // Element (1,3)
    float r23 = R_3_6[5];  // Element (2,3)
    
    // Check for singularity (q5 = 0 or ±180°)
    float sin_q5 = sqrtf(r31 * r31 + r32 * r32);
    
    if (sin_q5 < 0.001f) {
        // Singularity: q5 ≈ 0, infinite solutions for q4 and q6
        // Set q4 = 0 and solve for q6
        q5[0] = 0.0f;
        q5[1] = 0.0f;
        q4[0] = 0.0f;
        q4[1] = 0.0f;
        q6[0] = atan2f(R_3_6[3], R_3_6[0]);  // atan2(r21, r11)
        q6[1] = q6[0];
    } else {
        // Two solutions for q5
        q5[0] = atan2f(sin_q5, r33);
        q5[1] = atan2f(-sin_q5, r33);
        
        for (int i = 0; i < 2; i++) {
            float s5 = sinf(q5[i]);
            if (fabsf(s5) > 0.001f) {
                q4[i] = atan2f(r23 / s5, r13 / s5);
                q6[i] = atan2f(r32 / s5, -r31 / s5);
            } else {
                q4[i] = 0.0f;
                q6[i] = atan2f(R_3_6[3], R_3_6[0]);
            }
        }
    }
    
    return true;
}

bool CRS_Kinematics::solveIK(const Pose6D& pose, const Joint6D& lastJoints, 
                             IKSolves& solutions) {
    // Step 1: Compute wrist center position
    // Get rotation matrix from pose
    float R_desired[9];
    if (!pose.hasR) {
        // Convert Euler angles to rotation matrix
        float euler[3] = {
            pose.C * K_DEG_TO_RAD,  // Z
            pose.B * K_DEG_TO_RAD,  // Y
            pose.A * K_DEG_TO_RAD   // X
        };
        eulerAngleToRotMat(euler, R_desired);
    } else {
        memcpy(R_desired, pose.R, 9 * sizeof(float));
    }
    
    // Wrist center = TCP - d6 * z_axis_of_tcp
    // z_axis is the 3rd column of rotation matrix
    float d6 = dh[5].d;  // 90mm
    float wx = pose.X - d6 * R_desired[2];
    float wy = pose.Y - d6 * R_desired[5];
    float wz = pose.Z - d6 * R_desired[8];
    
    // Step 2: Solve position IK (J1, J2, J3)
    float q1[2];           // 2 shoulder solutions
    float q2[2][2];        // 2 shoulder x 2 elbow = 4 solutions
    float q3[2][2];
    
    if (!solvePositionIK(wx, wy, wz, q1, q2, q3)) {
        return false;
    }
    
    // Step 3: For each position solution, solve orientation IK
    int sol_idx = 0;
    bool found_solution = false;
    
    for (int shoulder = 0; shoulder < 2; shoulder++) {
        for (int elbow = 0; elbow < 2; elbow++) {
            // Compute R_0_3 for this position solution
            float q_pos[3] = {q1[shoulder], q2[shoulder][elbow], q3[shoulder][elbow]};
            
            // Build transformation to joint 3
            float T[4][16];
            memset(T[0], 0, 16 * sizeof(float));
            T[0][0] = T[0][5] = T[0][10] = T[0][15] = 1.0f;
            
            for (int i = 0; i < 3; i++) {
                float T_joint[16];
                computeDHMatrix(i, q_pos[i], T_joint);
                matMultiply((float*)T[i], T_joint, (float*)T[i + 1], 4, 4, 4);
            }
            
            // Extract R_0_3 (upper-left 3x3 of T[3])
            float R_0_3[9];
            R_0_3[0] = T[3][0];  R_0_3[1] = T[3][1];  R_0_3[2] = T[3][2];
            R_0_3[3] = T[3][4];  R_0_3[4] = T[3][5];  R_0_3[5] = T[3][6];
            R_0_3[6] = T[3][8];  R_0_3[7] = T[3][9];  R_0_3[8] = T[3][10];
            
            // Solve orientation
            float q4[2], q5[2], q6[2];
            if (solveOrientationIK(R_0_3, R_desired, q4, q5, q6)) {
                // Store both wrist flip solutions
                for (int wrist = 0; wrist < 2; wrist++) {
                    if (sol_idx < 8) {
                        // Convert motor angles back to joint angles (account for J2-J3 coupling)
                        solutions.config[sol_idx].a[0] = q1[shoulder] * K_RAD_TO_DEG;
                        solutions.config[sol_idx].a[1] = q2[shoulder][elbow] * K_RAD_TO_DEG;
                        
                        // J3_joint = J3_motor + J2_joint
                        float j3_joint = (q3[shoulder][elbow] + q2[shoulder][elbow]) * K_RAD_TO_DEG;
                        solutions.config[sol_idx].a[2] = j3_joint;
                        
                        solutions.config[sol_idx].a[3] = q4[wrist] * K_RAD_TO_DEG;
                        solutions.config[sol_idx].a[4] = q5[wrist] * K_RAD_TO_DEG;
                        solutions.config[sol_idx].a[5] = q6[wrist] * K_RAD_TO_DEG;
                        
                        // Mark solution as valid
                        solutions.solFlag[sol_idx][0] = 1;
                        solutions.solFlag[sol_idx][1] = 1;
                        solutions.solFlag[sol_idx][2] = 1;
                        
                        found_solution = true;
                        sol_idx++;
                    }
                }
            }
        }
    }
    
    return found_solution;
}

bool CRS_Kinematics::selectBestSolution(const IKSolves& solutions, 
                                         const Joint6D& currentJoints, 
                                         Joint6D& bestSolution) {
    int best_idx = -1;
    float min_distance = 1e9f;
    
    // Joint weights - larger/heavier joints get higher weight
    // This prioritizes minimizing movement of base/shoulder over wrist
    const float weights[6] = {
        2.0f,  // J1 - Base (heavy, slow)
        2.5f,  // J2 - Shoulder (heaviest, slowest)
        2.0f,  // J3 - Elbow (heavy)
        1.0f,  // J4 - Wrist roll (light, fast)
        1.0f,  // J5 - Wrist pitch (light, fast)
        0.5f   // J6 - Wrist yaw (lightest, fastest)
    };
    
    // Find solution with minimum weighted distance
    for (int i = 0; i < 8; i++) {
        // Check if solution is valid
        if (solutions.solFlag[i][0] == 0) {
            continue;
        }
        
        // Compute weighted distance from current position
        float distance = 0.0f;
        for (int j = 0; j < 6; j++) {
            float diff = solutions.config[i].a[j] - currentJoints.a[j];
            
            // Normalize to [-180, 180] range
            while (diff > 180.0f) diff -= 360.0f;
            while (diff < -180.0f) diff += 360.0f;
            
            // Weighted squared distance
            distance += weights[j] * diff * diff;
        }
        
        // Update best solution
        if (distance < min_distance) {
            min_distance = distance;
            best_idx = i;
        }
    }
    
    // Check if valid solution found
    if (best_idx < 0) {
        return false;
    }
    
    // Copy best solution
    for (int j = 0; j < 6; j++) {
        bestSolution.a[j] = solutions.config[best_idx].a[j];
    }
    
    return true;
}

bool CRS_Kinematics::solveIKBest(const Pose6D& pose, const Joint6D& currentJoints, 
                                  Joint6D& bestSolution) {
    IKSolves solutions;
    
    // Solve IK to get all candidates
    if (!solveIK(pose, currentJoints, solutions)) {
        return false;
    }
    
    // Select best solution
    return selectBestSolution(solutions, currentJoints, bestSolution);
}

// Utility operator for joint difference
Joint6D operator-(const Joint6D& j1, const Joint6D& j2) {
    Joint6D result;
    for (int i = 0; i < 6; i++) {
        result.a[i] = j1.a[i] - j2.a[i];
    }
    return result;
}
