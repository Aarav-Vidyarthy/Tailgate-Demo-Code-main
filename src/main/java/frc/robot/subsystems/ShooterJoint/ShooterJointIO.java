package frc.robot.subsystems.ShooterJoint;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;

public interface ShooterJointIO {
    // MJW: IO Layering 11/11/2024
    @AutoLog
    class ShooterJointIOInputs {
        // USED CURRENTLY
        double supplyCurrent = 0.0;
        double position = 0.0;
        double positionDegrees = 0.0;
        double absolutePosition = 0.0;
        double absolutePositionDegrees = 0.0;
        double motorVelocity = 0.0;
    }

    // Update Inputs
    default void updateInputs(ShooterJointIOInputs inputs) {}

    // Turn motors to Nuetral Mode
    default void stop() {}

    // Set MotionMagic Control
    default void setControl(MotionMagicVoltage goal) {}

    // Set Setpoint
    default void setPosition(PositionVoltage goal) {}
}
