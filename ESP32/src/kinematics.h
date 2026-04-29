#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>

// Forward declaration
class CRS_Kinematics;

// Structure for 6D joint angles (in degrees)
struct Joint6D {
    float a[6];
    
    Joint6D() : a{0, 0, 0, 0, 0, 0} {}
    Joint6D(float a1, float a2, float a3, float a4, float a5, float a6) 
        : a{a1, a2, a3, a4, a5, a6} {}
};

// Structure for 6D pose (position in mm, orientation in degrees)
struct Pose6D {
    float X, Y, Z;        // Position in mm
    float A, B, C;        // Euler angles ZYX in degrees
    float R[9];           // Rotation matrix (row-major)
    bool hasR;            // True if R is valid
    
    Pose6D() : X(0), Y(0), Z(0), A(0), B(0), C(0), hasR(false) {
        for(int i = 0; i < 9; i++) R[i] = 0;
    }
    
    Pose6D(float x, float y, float z, float a, float b, float c)
        : X(x), Y(y), Z(z), A(a), B(b), C(c), hasR(false) {
        for(int i = 0; i < 9; i++) R[i] = 0;
    }
};

// Structure for IK solutions (up to 8 solutions)
struct IKSolves {
    Joint6D config[8];     // 8 possible solutions
    char solFlag[8][3];    // Solution validity flags
    
    IKSolves() {
        for(int i = 0; i < 8; i++) {
            for(int j = 0; j < 3; j++) {
                solFlag[i][j] = 0;
            }
        }
    }
};

// Modified DH parameters for CRS F3 robot arm
struct DHParams {
    float a;           // Link length (mm)
    float alpha;       // Link twist (rad)
    float d;           // Link offset (mm)
    float theta_offset; // Joint angle offset (rad)
};

/**
 * @brief CRS F3 Robot Arm Kinematics Solver
 * 
 * Uses Modified DH parameters specific to the CRS F3 robot arm.
 * Accounts for mechanical coupling between joints 2 and 3.
 * 
 * Modified DH transformation order:
 *   T_i = Rx(α_{i-1}) * Tx(a_{i-1}) * Rz(θ_i) * Tz(d_i)
 */
class CRS_Kinematics {
private:
    static constexpr float K_RAD_TO_DEG = 57.295779513082321f;
    static constexpr float K_DEG_TO_RAD = 0.017453292519943295f;
    static constexpr float M_PI_VAL = 3.141592653589793f;
    
    // Modified DH parameters for CRS F3
    DHParams dh[6];
    
    // Precomputed geometric values
    float l_ew;      // Elbow-wrist distance
    float atan_e;    // Cached atan value
    
    // Helper functions
    void matMultiply(const float* m1, const float* m2, float* out, 
                     int rows1, int cols1_rows2, int cols2);
    void rotMatToEulerAngle(const float* R, float* euler);
    void eulerAngleToRotMat(const float* euler, float* R);
    void computeDHMatrix(int joint, float theta, float* T);
    
public:
    /**
     * @brief Construct kinematics solver with CRS F3 parameters
     * 
     * CRS F3 Robot Arm dimensions (in mm):
     * - Base height: 350mm
     * - Base offset: 100mm
     * - Upper arm: 265mm
     * - Forearm: 270mm
     * - Wrist length: 90mm
     */
    CRS_Kinematics();
    
    /**
     * @brief Solve forward kinematics
     * 
     * Converts joint angles to end-effector pose.
     * Accounts for mechanical coupling: J3_motor = J3_joint - J2_joint
     * 
     * @param joints Input joint angles in degrees
     * @param pose Output end-effector pose
     * @return true if successful
     */
    bool solveFK(const Joint6D& joints, Pose6D& pose);
    
    /**
     * @brief Solve inverse kinematics (analytical solution)
     * 
     * Computes up to 8 possible joint configurations for a given pose.
     * Uses geometric decoupling: position (J1-J3) and orientation (J4-J6).
     * Accounts for mechanical coupling in the solutions.
     * 
     * Solution variants:
     * - 2 shoulder configurations (left/right)
     * - 2 elbow configurations (up/down)  
     * - 2 wrist configurations (flip/no-flip)
     * = 8 total solutions
     * 
     * @param pose Desired end-effector pose
     * @param lastJoints Previous joint configuration (for solution selection)
     * @param solutions Output solutions structure
     * @return true if at least one solution found
     * 
     * @note J2-J3 coupling: Motor angles are converted internally
     */
    bool solveIK(const Pose6D& pose, const Joint6D& lastJoints, IKSolves& solutions);
    
    /**
     * @brief Select the best IK solution from multiple solutions
     * 
     * Chooses the solution that requires minimal joint movement from the
     * current position. Uses weighted distance metric to account for
     * different joint speeds and ranges.
     * 
     * @param solutions IK solutions from solveIK()
     * @param currentJoints Current joint configuration
     * @param bestSolution Output: selected best solution
     * @return true if a valid solution was found, false otherwise
     * 
     * @note Joint weights: J1-J3 weighted higher due to larger inertia
     */
    bool selectBestSolution(const IKSolves& solutions, const Joint6D& currentJoints, 
                            Joint6D& bestSolution);
    
    /**
     * @brief Solve IK and automatically select best solution
     * 
     * Convenience function that combines solveIK() and selectBestSolution().
     * Returns the single best solution instead of all 8 candidates.
     * 
     * @param pose Desired end-effector pose
     * @param currentJoints Current joint configuration
     * @param bestSolution Output: best joint configuration
     * @return true if solution found
     */
    bool solveIKBest(const Pose6D& pose, const Joint6D& currentJoints, 
                     Joint6D& bestSolution);
    
private:
    // IK helper functions
    bool solvePositionIK(float wx, float wy, float wz, float q1[2], float q2[2][2], float q3[2][2]);
    bool solveOrientationIK(const float* R_0_3, const float* R_desired, float q4[2], float q5[2], float q6[2]);
    
    /**
     * @brief Get DH parameters (read-only)
     */
    const DHParams* getDHParams() const { return dh; }
};

// Utility operator for joint difference
Joint6D operator-(const Joint6D& j1, const Joint6D& j2);

#endif // KINEMATICS_H
