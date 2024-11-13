// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.RobotState;

import static frc.robot.Constants.LimelightConstants.*;

import java.util.OptionalDouble;

public class Limelight extends SubsystemBase {
    Alliance alliance;
    private String ll = kCameraName;
    private Boolean hasTarget = false;

    /** Creates a new Limelight. */
    public Limelight() {

    }

    @Override
    public void periodic() {
        if (!Robot.isSimulation()) {
            LimelightHelpers.Results result = LimelightHelpers.getLatestResults(ll).targetingResults;
            if (result.valid && LimelightHelpers.getTA(kCameraName) > .25) {
                hasTarget = true;
                RobotState.getInstance().setAngleToNote(OptionalDouble.of(LimelightHelpers.getTX(kCameraName)));
            } else {
                hasTarget = false;
                RobotState.getInstance().setAngleToNote(OptionalDouble.empty());
            }
            if (RobotConstants.kIsTuningMode) {
                SmartDashboard.putBoolean("Limelight has note detected", hasTarget);
            }

        }
    }

    public boolean hasTarget() {
        return hasTarget;
    }

}
