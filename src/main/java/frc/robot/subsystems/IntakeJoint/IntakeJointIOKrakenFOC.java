package frc.robot.subsystems.IntakeJoint;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.google.flatbuffers.Constants;

import edu.wpi.first.math.util.Units;

import frc.robot.Constants.IntakeJointConstants;

public class IntakeJointIOKrakenFOC implements IntakeJointIO {
    // Hardware
    private final TalonFX m_motor;

    // Status Signals

    private final StatusSignal<Double> m_motorPosition;
    private final StatusSignal<Double> m_motorVelocity;
    private final StatusSignal<Double> m_motorAppliedVolts;
    private final StatusSignal<Double> m_motorSupplyCurrent;
    private final StatusSignal<Double> m_motorTorqueCurrent;

    // Control
    private final VelocityVoltage m_velocity = new VelocityVoltage(0).withSlot(1); //Line 50
    private final NeutralOut m_neutral = new NeutralOut().withUpdateFreqHz(0.0); //Line 51
    private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0.0);
    private final DutyCycleOut m_duty = new DutyCycleOut(0.0);

    public IntakeJointIOKrakenFOC() {
        m_motor = new TalonFX(IntakeJointConstants.ID_Motor);

        TalonFXConfiguration m_configuration = new TalonFXConfiguration();

        m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        m_configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.35;
        m_configuration.Voltage.PeakForwardVoltage = 12.0;
        m_configuration.Voltage.PeakReverseVoltage = -12.0;

        m_configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        m_configuration.Feedback.SensorToMechanismRatio = 500.0/7.0;

        m_configuration.Slot0.kP = 1; // output per unit of error in position (output/rotation)
        m_configuration.Slot0.kI = 0; // output per unit of integrated error in position (output/(rotation*s))
        m_configuration.Slot0.kD = 0; // output per unit of error derivative in position (output/rps)

        m_configuration.Slot1.kG = 0; // output to overcome gravity (output)
        m_configuration.Slot1.kS = 0; // output to overcome static friction (output)
        m_configuration.Slot1.kV = 0; // output per unit of requested velocity (output/rps)
        m_configuration.Slot1.kA = 0; // unused, as there is no target acceleration
        m_configuration.Slot1.kP = 100; // output per unit of error in position (output/rotation)
        m_configuration.Slot1.kI = 0; // output per unit of integrated error in position (output/(rotation*s))
        m_configuration.Slot1.kD = 0; // output per unit of error derivative in position (output/rps)

        m_configuration.MotionMagic.MotionMagicCruiseVelocity = 50;
        m_configuration.MotionMagic.MotionMagicAcceleration = 500;
        m_configuration.MotionMagic.MotionMagicJerk = 0;

        m_configuration.CurrentLimits.SupplyCurrentLimit = 40;
        m_configuration.CurrentLimits.SupplyCurrentThreshold = 40;
        m_configuration.CurrentLimits.SupplyTimeThreshold = 0.01;
        m_configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_configuration.CurrentLimits.StatorCurrentLimit = 70;
        m_configuration.CurrentLimits.StatorCurrentLimitEnable = true;

        // Apply Configs
        m_motor.getConfigurator().apply(m_configuration);

        // Set Signals
        m_motorPosition = m_motor.getPosition();
        m_motorVelocity = m_motor.getVelocity();
        m_motorAppliedVolts = m_motor.getMotorVoltage();
        m_motorSupplyCurrent = m_motor.getSupplyCurrent();
        m_motorTorqueCurrent = m_motor.getTorqueCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            m_motorPosition,
            m_motorVelocity,
            m_motorAppliedVolts,
            m_motorSupplyCurrent,
            m_motorTorqueCurrent
            );
    }
    // Update Inputs
    public void updateInputs(IntakeJointIOInputs inputs) {
        inputs.motorVelocity = m_motorVelocity.getValueAsDouble();
        inputs.position = m_motorPosition.getValueAsDouble();
        inputs.supplyCurrent = m_motorSupplyCurrent.getValueAsDouble();
    }

    // Turn motors to Nuetral Mode
    public void stop() {
        m_motor.setControl(m_neutral);
    }

    // Run Duty Cycle
    public void runDutyCycle(double output) {
        m_motor.setControl(m_duty.withOutput(output));
    }

    // Set MotionMagic Control
    public void setControl(MotionMagicVoltage goal) {
        m_motor.setControl(goal.withSlot(1));
    }

    // Set Setpoint
    public void setPosition(double goal) {
        m_motor.setPosition(goal);
    }
}
