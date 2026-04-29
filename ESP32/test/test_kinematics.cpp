#include <Arduino.h>
#include "kinematics.h"

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("\n=== CRS F3 Kinematics Sanity Check ===\n");
    
    CRS_Kinematics kin;
    
    // Test: Forward Kinematics at home position (0,0,0,0,0,0)
    Joint6D joints_home(0, 0, 0, 0, 0, 0);
    Pose6D pose;
    
    Serial.println("Testing FK at HOME position:");
    Serial.println("Joint Angles: J1=0 J2=0 J3=0 J4=0 J5=0 J6=0");
    Serial.println();
    
    if (kin.solveFK(joints_home, pose)) {
        Serial.println("Forward Kinematics Result:");
        Serial.printf("  Position: X=%.2f  Y=%.2f  Z=%.2f mm\n", pose.X, pose.Y, pose.Z);
        Serial.printf("  Orientation: A=%.2f  B=%.2f  C=%.2f degrees\n", pose.A, pose.B, pose.C);
        Serial.println();
        
        Serial.println("  Rotation Matrix:");
        Serial.printf("    [%.4f  %.4f  %.4f]\n", pose.R[0], pose.R[1], pose.R[2]);
        Serial.printf("    [%.4f  %.4f  %.4f]\n", pose.R[3], pose.R[4], pose.R[5]);
        Serial.printf("    [%.4f  %.4f  %.4f]\n", pose.R[6], pose.R[7], pose.R[8]);
        Serial.println();
        
        // Expected position calculation:
        // At home (0,0,0,0,0,0) arm points straight up
        // Base: 350mm height, 100mm offset
        // Arm: 265mm pointing up
        // Forearm: 270mm pointing up
        // Wrist: 90mm pointing up
        // Expected: X=100, Y=0, Z=350+265+270+90=975
        
        Serial.println("Expected (home position - arm vertical):");
        Serial.println("  X ≈ 100mm (base_offset)");
        Serial.println("  Y ≈ 0mm");
        Serial.println("  Z ≈ 975mm (base_height + arm + forearm + wrist)");
        Serial.println();
        
        // Calculate error
        float expected_x = 100.0f;
        float expected_y = 0.0f;
        float expected_z = 975.0f;
        
        float error = sqrtf(powf(pose.X - expected_x, 2) + 
                           powf(pose.Y - expected_y, 2) + 
                           powf(pose.Z - expected_z, 2));
        
        Serial.printf("Position error from expected: %.2f mm\n", error);
        
        if (error < 50.0f) {
            Serial.println("✓ PASS - Position is close to expected");
        } else {
            Serial.println("✗ WARNING - Position differs from expected");
        }
    } else {
        Serial.println("✗ FAILED - FK returned false");
    }
    
    Serial.println("\n=== Test Complete ===");
}

void loop() {
    // Nothing to do
}
