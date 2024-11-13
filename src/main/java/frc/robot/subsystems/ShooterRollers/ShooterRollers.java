// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ShooterRollers;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.Constants.ShooterRollersConstants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.AutoLogOutput;

public class ShooterRollers extends SubsystemBase {

    @RequiredArgsConstructor
    @Getter
    public enum State { // RPS
        OFF(() -> 0.0),
        PASSTHROUGH(() -> 10.0), // Poop & Scoot
        SUBWOOFER(() -> 35.0),
        SPEAKER(() -> 45.0), // Default
        FEED(() -> 22.0),
        REVERSE(() -> -20.0), // Hopefully never have to use this irl
        TUNING(() -> RobotState.getInstance().getShooterTuningSpeed().get());

        private final DoubleSupplier velocitySupplier;

        private double getStateOutput() {
            return velocitySupplier.getAsDouble();
        }
    }

    @Getter
    @Setter
    private State state = State.OFF;

    // Initialize motor controllers
    //TalonFX m_motor = new TalonFX(ShooterRollersConstants.ID_LEADER);   // Can we change this to be Bottom Motor? MJW
    //TalonFX m_follower = new TalonFX(ShooterRollersConstants.ID_FOLLOWER); // Can we change this to be Top Motor? MJW
    
    //private final VelocityVoltage m_velocity = new VelocityVoltage(0).withSlot(1);
    //private final NeutralOut m_neutral = new NeutralOut();

    private double goalSpeed;

    //AdvantageKit addition MJW 10/29/2024
    private final ShooterRollersIO io;
    private final ShooterRollersIOInputsAutoLogged inputs = new ShooterRollersIOInputsAutoLogged();

    /** Creates a new Flywheel. */
    public ShooterRollers(ShooterRollersIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("ShooterRollers", inputs);
        if (state == State.OFF) {
            //m_motor.setControl(m_neutral);
            io.stop();  //MJW: See the ShooterRollersIO to see all the commands we feed to motors
        } else {
            goalSpeed = MathUtil.clamp(state.getStateOutput(), ShooterRollersConstants.lowerLimit, ShooterRollersConstants.upperLimit); //TODO:Remove 
            io.setPoint(goalSpeed);
        }

        displayInfo(true);
    }

    public boolean atGoal() {
        return Math.abs(state.getStateOutput() - inputs.motorVelocity) < ShooterRollersConstants.tolerance;
    }

    public Command setStateCommand(State state) {
        return startEnd(() -> this.state = state, () -> this.state = State.OFF);
    }

    public void displayInfo(boolean debug) {
        if (debug) {
            SmartDashboard.putString(this.getClass().getSimpleName() + " State ", state.toString());
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Setpoint ", state.getStateOutput());
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Output ", inputs.motorVelocity);
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Current Draw", inputs.topSupplyCurrentAmps);
            SmartDashboard.putBoolean(this.getClass().getSimpleName() + " atGoal", atGoal());
        }
        // AdvantageKit Logging
        /* If you want to log a variable not already logged use this:
         * Logger.recordOutput("<name>", data);
         */
    }
}
 