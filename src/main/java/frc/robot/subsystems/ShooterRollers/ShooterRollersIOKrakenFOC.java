package frc.robot.subsystems.ShooterRollers;

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

import frc.robot.Constants.ShooterRollersConstants;

public class ShooterRollersIOKrakenFOC implements ShooterRollersIO {
    // Hardware
    private final TalonFX TopTalon;
    private final TalonFX BottomTalon;

    // Status Signals
    
    private final StatusSignal<Double> topPosition;
    private final StatusSignal<Double> topVelocity;
    private final StatusSignal<Double> topAppliedVolts;
    private final StatusSignal<Double> topSupplyCurrent;
    private final StatusSignal<Double> topTorqueCurrent;
    private final StatusSignal<Double> bottomPosition;
    private final StatusSignal<Double> bottomVelocity;
    private final StatusSignal<Double> bottomAppliedVolts;
    private final StatusSignal<Double> bottomSupplyCurrent;
    private final StatusSignal<Double> bottomTorqueCurrent;
    
    // Control
    private final VelocityVoltage m_velocity = new VelocityVoltage(0).withSlot(1); //Line 50
    private final NeutralOut m_neutral = new NeutralOut().withUpdateFreqHz(0.0); //Line 51
    private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0.0);

    public ShooterRollersIOKrakenFOC() {
        BottomTalon = new TalonFX(ShooterRollersConstants.ID_LEADER); // Leader
        TopTalon = new TalonFX(ShooterRollersConstants.ID_FOLLOWER); // Follower
        // General Config
        TalonFXConfiguration m_configuration = new TalonFXConfiguration();
        m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_configuration.Voltage.PeakForwardVoltage = 12.0;
        m_configuration.Voltage.PeakReverseVoltage = -12.0;

        m_configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        m_configuration.Feedback.SensorToMechanismRatio = 24.0/15.0;

        m_configuration.Slot0.kP = 1; // output per unit of error in position (output/rotation)
        m_configuration.Slot0.kI = 0; // output per unit of integrated error in position (output/(rotation*s))
        m_configuration.Slot0.kD = 0; // output per unit of error derivative in position (output/rps)

        m_configuration.Slot1.kG = 0; // output to overcome gravity (output)
        m_configuration.Slot1.kS = 0; // output to overcome static friction (output)
        m_configuration.Slot1.kV = 0.13; // output per unit of requested velocity (output/rps)
        m_configuration.Slot1.kA = 0; // unused, as there is no target acceleration
        m_configuration.Slot1.kP = 1; // output per unit of error in position (output/rotation)
        m_configuration.Slot1.kI = 0; // output per unit of integrated error in position (output/(rotation*s))
        m_configuration.Slot1.kD = 0; // output per unit of error derivative in position (output/rps)

        m_configuration.MotionMagic.MotionMagicCruiseVelocity = 10;
        m_configuration.MotionMagic.MotionMagicAcceleration = 10;
        m_configuration.MotionMagic.MotionMagicJerk = 10;

        m_configuration.CurrentLimits.SupplyCurrentLimit = 20;
        m_configuration.CurrentLimits.SupplyCurrentThreshold = 40;
        m_configuration.CurrentLimits.SupplyTimeThreshold = 0.1;
        m_configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_configuration.CurrentLimits.StatorCurrentLimit = 70;
        m_configuration.CurrentLimits.StatorCurrentLimitEnable = true;

        // Apply Configs
        BottomTalon.getConfigurator().apply(m_configuration);
        TopTalon.getConfigurator().apply(m_configuration);
        TopTalon.setControl(new Follower(BottomTalon.getDeviceID(), true));

        // Set Signals
        bottomPosition = BottomTalon.getPosition();
        bottomVelocity = BottomTalon.getVelocity();
        bottomAppliedVolts = BottomTalon.getMotorVoltage();
        bottomSupplyCurrent = BottomTalon.getSupplyCurrent();
        bottomTorqueCurrent = BottomTalon.getTorqueCurrent();

        topPosition = TopTalon.getPosition();
        topVelocity = TopTalon.getVelocity();
        topAppliedVolts = TopTalon.getMotorVoltage();
        topSupplyCurrent = TopTalon.getSupplyCurrent();
        topTorqueCurrent = TopTalon.getTorqueCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            bottomPosition,
            bottomVelocity,
            bottomAppliedVolts,
            bottomSupplyCurrent,
            bottomTorqueCurrent,
            topPosition,
            topVelocity,
            topAppliedVolts,
            topSupplyCurrent,
            topTorqueCurrent);
    }

    @Override
    public void updateInputs(ShooterRollersIOInputs inputs) {
        inputs.topPositionRads = Units.rotationsToRadians(topPosition.getValueAsDouble());
        inputs.topVelocityRpm = topVelocity.getValueAsDouble() * 60.0;
        inputs.topAppliedVolts = topAppliedVolts.getValueAsDouble();
        inputs.topSupplyCurrentAmps = topSupplyCurrent.getValueAsDouble();
        inputs.topTorqueCurrentAmps = topTorqueCurrent.getValueAsDouble();

        inputs.bottomPositionRads = Units.rotationsToRadians(bottomPosition.getValueAsDouble());
        inputs.bottomVelocityRpm = bottomVelocity.getValueAsDouble() * 60.0;
        inputs.bottomAppliedVolts = bottomAppliedVolts.getValueAsDouble();
        inputs.bottomSupplyCurrentAmps = bottomSupplyCurrent.getValueAsDouble();
        inputs.bottomTorqueCurrentAmps = bottomTorqueCurrent.getValueAsDouble();

        // CURRENTLY USED
        inputs.motorVelocity = TopTalon.getVelocity().getValueAsDouble();
    }

    @Override
    public void runVolts(double topVolts, double bottomVolts) {
        TopTalon.setControl(voltageControl.withOutput(topVolts));
        BottomTalon.setControl(voltageControl.withOutput(bottomVolts));
    }

    @Override
    public void stop() {
        BottomTalon.setControl(m_neutral);
        TopTalon.setControl(m_neutral);
    }

    @Override
    public void runVelocity(double Rpm, double Feedforward) {
        BottomTalon.setControl(m_velocity.withVelocity(Rpm/60.0).withFeedForward(Feedforward));
        TopTalon.setControl(m_velocity.withVelocity(Rpm/60.0).withFeedForward(Feedforward));
    }

    @Override
    public void setPoint(double goalSpeed) {
        TopTalon.setControl(m_velocity.withVelocity(goalSpeed).withSlot(1)); // create a velocity closed-loop request, voltage output, slot 1 configs
    }
}
