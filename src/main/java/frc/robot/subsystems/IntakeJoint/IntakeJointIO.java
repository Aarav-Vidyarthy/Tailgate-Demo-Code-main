package frc.robot.subsystems.IntakeJoint;

import org.littletonrobotics.junction.AutoLog;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

public interface IntakeJointIO {
    // MJW: IO Layering 11/11/2024
    @AutoLog
    class IntakeJointIOInputs {
        // USED CURRENTLY
        double supplyCurrent = 0.0;
        double position = 0.0;
        double motorVelocity = 0.0;
    }

    // Update Inputs
    default void updateInputs(IntakeJointIOInputs inputs) {}

    // Turn motors to Nuetral Mode
    default void stop() {}

    // Run Duty Cycle
    default void runDutyCycle(double output) {}

    // Set MotionMagic Control
    default void setControl(MotionMagicVoltage goal) {}

    // Set Setpoint
    default void setPosition(double goal) {}
}
