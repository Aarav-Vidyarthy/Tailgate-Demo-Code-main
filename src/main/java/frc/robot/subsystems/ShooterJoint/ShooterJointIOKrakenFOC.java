package frc.robot.subsystems.ShooterJoint;

import java.util.Set;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.google.flatbuffers.Constants;

import edu.wpi.first.math.util.Units;

import frc.robot.Constants.ShooterJointConstants;

public class ShooterJointIOKrakenFOC implements ShooterJointIO{
    // Hardware
    private final TalonFX m_motor;
    private final CANcoder m_encoder;

    // Status Signals

    private final StatusSignal<Double> m_motorPosition;
    private final StatusSignal<Double> m_motorVelocity;
    private final StatusSignal<Double> encoderAbsolutePositionRotations;
  private final StatusSignal<Double> encoderRelativePositionRotations;
    private final StatusSignal<Double> m_motorAppliedVolts;
    private final StatusSignal<Double> m_motorSupplyCurrent;
    private final StatusSignal<Double> m_motorTorqueCurrent;

    // Control
    private final VelocityVoltage m_velocity = new VelocityVoltage(0).withSlot(1); //Line 50
    private final NeutralOut m_neutral = new NeutralOut().withUpdateFreqHz(0.0); //Line 51
    private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0.0);

    public ShooterJointIOKrakenFOC() {
        m_motor = new TalonFX(ShooterJointConstants.ID_MOTOR);
        m_encoder = new CANcoder(ShooterJointConstants.ID_ENCODER);

        TalonFXConfiguration m_configuration = new TalonFXConfiguration();

        m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        m_configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(46);
        m_configuration.Voltage.PeakForwardVoltage = 12.0;
        m_configuration.Voltage.PeakReverseVoltage = -12.0;

        m_configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        m_configuration.Feedback.FeedbackRemoteSensorID = m_encoder.getDeviceID();
        m_configuration.Feedback.RotorToSensorRatio = 54.4/7.04;
        m_configuration.Feedback.SensorToMechanismRatio = 7.04;

        m_configuration.Slot0.kP = 50; // output per unit of error in position (output/rotation)
        m_configuration.Slot0.kI = 0; // output per unit of integrated error in position (output/(rotation*s))
        m_configuration.Slot0.kD = 0; // output per unit of error derivative in position (output/rps)

        m_configuration.Slot1.kG = 0.1; // output to overcome gravity (output)
        m_configuration.Slot1.kS = 0; // output to overcome static friction (output)
        m_configuration.Slot1.kV = 0; // output per unit of requested velocity (output/rps)
        m_configuration.Slot1.kA = 0; // unused, as there is no target acceleration
        m_configuration.Slot1.kP = 50; // output per unit of error in position (output/rotation)
        m_configuration.Slot1.kI = 0; // output per unit of integrated error in position (output/(rotation*s))
        m_configuration.Slot1.kD = 0; // output per unit of error derivative in position (output/rps)

        m_configuration.MotionMagic.MotionMagicCruiseVelocity = 500;
        m_configuration.MotionMagic.MotionMagicAcceleration = 5;
        m_configuration.MotionMagic.MotionMagicJerk = 0;

        m_configuration.CurrentLimits.SupplyCurrentLimit = 20;
        m_configuration.CurrentLimits.SupplyCurrentThreshold = 40;
        m_configuration.CurrentLimits.SupplyTimeThreshold = 0.1;
        m_configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_configuration.CurrentLimits.StatorCurrentLimit = 70;
        m_configuration.CurrentLimits.StatorCurrentLimitEnable = true;

        CANcoderConfiguration mEnc_configuration = new CANcoderConfiguration();
        mEnc_configuration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        mEnc_configuration.MagnetSensor.MagnetOffset = -0.396240234375;
        mEnc_configuration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        // Apply Configs
        m_motor.getConfigurator().apply(m_configuration);
        m_encoder.getConfigurator().apply(mEnc_configuration);

        // Set Signals
        m_motorPosition = m_motor.getPosition();
        m_motorVelocity = m_motor.getVelocity();
        m_motorAppliedVolts = m_motor.getMotorVoltage();
        m_motorSupplyCurrent = m_motor.getSupplyCurrent();
        m_motorTorqueCurrent = m_motor.getTorqueCurrent();

        encoderAbsolutePositionRotations = m_encoder.getAbsolutePosition();
        encoderRelativePositionRotations = m_encoder.getPosition();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            m_motorPosition,
            m_motorVelocity,
            m_motorAppliedVolts,
            m_motorSupplyCurrent,
            m_motorTorqueCurrent
            );
        BaseStatusSignal.setUpdateFrequencyForAll(
            500, encoderAbsolutePositionRotations, encoderRelativePositionRotations);
    }
    // Update Inputs
    public void updateInputs(ShooterJointIOInputs inputs) {
        inputs.motorVelocity = m_motorVelocity.getValueAsDouble();
        inputs.position = m_motorPosition.getValueAsDouble();
        inputs.positionDegrees = Units.rotationsToDegrees(m_motor.getPosition().getValueAsDouble());
        inputs.absolutePosition = encoderAbsolutePositionRotations.getValueAsDouble();
        inputs.absolutePositionDegrees = Units.rotationsToDegrees(encoderAbsolutePositionRotations.getValueAsDouble());
        inputs.supplyCurrent = m_motorSupplyCurrent.getValueAsDouble();
    }

    // Turn motors to Nuetral Mode
    public void stop() {
        m_motor.setControl(m_neutral);
    }

    // Set MotionMagic Control
    public void setControl(MotionMagicVoltage goal) {
        m_motor.setControl(goal.withSlot(1));
    }

    // Set Setpoint
    public void setPosition(PositionVoltage goal) {
        m_motor.setControl(goal);
    }
}
