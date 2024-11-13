// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

/** Add your docs here. */
public class Constants {

    public static final Mode currentMode = Mode.REAL;

    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public static final class RobotConstants {
        public static final boolean kIsTuningMode = true;
    }

    public static final class DriveConstants {
        public static final double headingAngleTolerance = 3.0;
        public static final double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
        public static final double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
        public static final double driverSpeed = 0.6; //Multiplier to the controller input
    }

    public static final class ClimberJointConstants {
        public static final int ID_LEADER = 25;
        public static final int ID_FOLLOWER = 26;

        public static final double tolerance = 1;

        public static TalonFXConfiguration motorConfig() {
            TalonFXConfiguration m_configuration = new TalonFXConfiguration();

            m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            m_configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            m_configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            m_configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 77.6;
            m_configuration.Voltage.PeakForwardVoltage = 12.0;
            m_configuration.Voltage.PeakReverseVoltage = -12.0;

            m_configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
            m_configuration.Feedback.SensorToMechanismRatio = 1;

            m_configuration.Slot0.kP = 0; // output per unit of error in position (output/rotation)
            m_configuration.Slot0.kI = 0; // output per unit of integrated error in position (output/(rotation*s))
            m_configuration.Slot0.kD = 0; // output per unit of error derivative in position (output/rps)

            m_configuration.Slot1.kG = 0; // output to overcome gravity (output)
            m_configuration.Slot1.kS = 0; // output to overcome static friction (output)
            m_configuration.Slot1.kV = 0; // output per unit of requested velocity (output/rps)
            m_configuration.Slot1.kA = 0; // unused, as there is no target acceleration
            m_configuration.Slot1.kP = 5; // output per unit of error in position (output/rotation)
            m_configuration.Slot1.kI = 0; // output per unit of integrated error in position (output/(rotation*s))
            m_configuration.Slot1.kD = 0; // output per unit of error derivative in position (output/rps)

            m_configuration.MotionMagic.MotionMagicCruiseVelocity = 600;
            m_configuration.MotionMagic.MotionMagicAcceleration = 200;
            m_configuration.MotionMagic.MotionMagicJerk = 0;

            m_configuration.CurrentLimits.SupplyCurrentLimit = 20;
            m_configuration.CurrentLimits.SupplyCurrentThreshold = 40;
            m_configuration.CurrentLimits.SupplyTimeThreshold = 0.1;
            m_configuration.CurrentLimits.SupplyCurrentLimitEnable = false;
            m_configuration.CurrentLimits.StatorCurrentLimit = 70;
            m_configuration.CurrentLimits.StatorCurrentLimitEnable = false;

            return m_configuration;
        }
    }

    public static final class YSplitRollersConstants {
        public static final int ID_YSPLIT_ROLLER1 = 16;
        public static final int ID_YSPLIT_ROLLER2 = 17;

        public static TalonFXConfiguration motorConfig() {
            TalonFXConfiguration m_configuration = new TalonFXConfiguration();

            m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            m_configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            m_configuration.Voltage.PeakForwardVoltage = 12.0;
            m_configuration.Voltage.PeakReverseVoltage = -12.0;

            m_configuration.CurrentLimits.SupplyCurrentLimit = 20;
            m_configuration.CurrentLimits.SupplyCurrentThreshold = 60;
            m_configuration.CurrentLimits.SupplyTimeThreshold = 0.25;
            m_configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
            m_configuration.CurrentLimits.StatorCurrentLimit = 140;
            m_configuration.CurrentLimits.StatorCurrentLimitEnable = true;

            return m_configuration;
        }
    }

    public static final class ShooterJointConstants {
        public static final int ID_MOTOR = 18;
        public static final int ID_ENCODER = 19;

        public static final double tolerance = Units.degreesToRotations(2);

        public static TalonFXConfiguration motorConfig() {
            TalonFXConfiguration m_configuration = new TalonFXConfiguration();

            m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            m_configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            m_configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            m_configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(46);
            m_configuration.Voltage.PeakForwardVoltage = 12.0;
            m_configuration.Voltage.PeakReverseVoltage = -12.0;

            m_configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
            m_configuration.Feedback.FeedbackRemoteSensorID = ID_ENCODER;
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

            return m_configuration;
        }

        public static CANcoderConfiguration encoderConfig() {
            CANcoderConfiguration m_configuration = new CANcoderConfiguration();
            m_configuration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
            m_configuration.MagnetSensor.MagnetOffset = -0.396240234375;
            m_configuration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

            return m_configuration;
        }
    }

    public static final class ShooterRollersConstants {
        public static final int ID_LEADER = 20;
        public static final int ID_FOLLOWER = 21;

        //RPS
        public static final double upperLimit = 50.0;
        public static final double lowerLimit = -upperLimit;
        public static final double tolerance = 10;

        public static TalonFXConfiguration motorConfig() {
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
            m_configuration.Slot1.kV = 0.19; // output per unit of requested velocity (output/rps)
            m_configuration.Slot1.kA = 0; // unused, as there is no target acceleration
            m_configuration.Slot1.kP = 1; // output per unit of error in position (output/rotation)
            m_configuration.Slot1.kI = 0; // output per unit of integrated error in position (output/(rotation*s))
            m_configuration.Slot1.kD = 0; // output per unit of error derivative in position (output/rps)

            m_configuration.MotionMagic.MotionMagicCruiseVelocity = 10;
            m_configuration.MotionMagic.MotionMagicAcceleration = 10;
            m_configuration.MotionMagic.MotionMagicJerk = 10;

            m_configuration.CurrentLimits.SupplyCurrentLimit = 20;
            m_configuration.CurrentLimits.SupplyTimeThreshold = 0.5;
            m_configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
            m_configuration.CurrentLimits.StatorCurrentLimit = 80;
            m_configuration.CurrentLimits.StatorCurrentLimitEnable = true;

            return m_configuration;
        }
    }

    public static final class IntakeRollersConstants {
        public static final int ID_Motor = 15;

        public static TalonFXConfiguration motorConfig() {
            TalonFXConfiguration m_configuration = new TalonFXConfiguration();

            m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            m_configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            m_configuration.Voltage.PeakForwardVoltage = 12.0;
            m_configuration.Voltage.PeakReverseVoltage = -12.0;

            m_configuration.CurrentLimits.SupplyCurrentLimit = 40;
            m_configuration.CurrentLimits.SupplyCurrentThreshold = 40;
            m_configuration.CurrentLimits.SupplyTimeThreshold = 0.1;
            m_configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
            m_configuration.CurrentLimits.StatorCurrentLimit = 80;
            m_configuration.CurrentLimits.StatorCurrentLimitEnable = true;

            return m_configuration;
        }
    }

    public static final class IntakeJointConstants {
        public static final int ID_Motor = 14;

        public static final double tolerance = Units.degreesToRotations(5);
        public static final double homingCurrent = .5;

        public static TalonFXConfiguration motorConfig() {
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

            return m_configuration;
        }
    }

    public static final class ElevatorRollersConstants {
        public static final int ID_Motor = 24;

        public static TalonFXConfiguration motorConfig() {
            TalonFXConfiguration m_configuration = new TalonFXConfiguration();

            m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            m_configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            m_configuration.Voltage.PeakForwardVoltage = 12.0;
            m_configuration.Voltage.PeakReverseVoltage = -12.0;

            m_configuration.CurrentLimits.SupplyCurrentLimit = 20;
            m_configuration.CurrentLimits.SupplyCurrentThreshold = 40;
            m_configuration.CurrentLimits.SupplyTimeThreshold = 0.1;
            m_configuration.CurrentLimits.SupplyCurrentLimitEnable = false;
            m_configuration.CurrentLimits.StatorCurrentLimit = 70;
            m_configuration.CurrentLimits.StatorCurrentLimitEnable = false;

            return m_configuration;
        }
    }

    public static final class ElevatorJointConstants {
        public static final int ID_LEADER = 22;
        public static final int ID_FOLLOWER = 23;

        public static final double tolerance = .5;
        public static final double homingCurrent = 0.5;

        public static TalonFXConfiguration motorConfig() {
            TalonFXConfiguration m_configuration = new TalonFXConfiguration();

            m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            m_configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            m_configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            m_configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 33.5;
            m_configuration.Voltage.PeakForwardVoltage = 12.0;
            m_configuration.Voltage.PeakReverseVoltage = -12.0;

            m_configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
            m_configuration.Feedback.SensorToMechanismRatio = 1;

            m_configuration.Slot0.kP = 0; // output per unit of error in position (output/rotation)
            m_configuration.Slot0.kI = 0; // output per unit of integrated error in position (output/(rotation*s))
            m_configuration.Slot0.kD = 0; // output per unit of error derivative in position (output/rps)

            m_configuration.Slot1.kG = 0.1; // output to overcome gravity (output)
            m_configuration.Slot1.kS = 0; // output to overcome static friction (output)
            m_configuration.Slot1.kV = 0; // output per unit of requested velocity (output/rps)
            m_configuration.Slot1.kA = 0; // unused, as there is no target acceleration
            m_configuration.Slot1.kP = 2; // output per unit of error in position (output/rotation)
            m_configuration.Slot1.kI = 0; // output per unit of integrated error in position (output/(rotation*s))
            m_configuration.Slot1.kD = 0; // output per unit of error derivative in position (output/rps)

            m_configuration.MotionMagic.MotionMagicCruiseVelocity = 1000;
            m_configuration.MotionMagic.MotionMagicAcceleration = 1000;
            m_configuration.MotionMagic.MotionMagicJerk = 0.0;

            m_configuration.CurrentLimits.SupplyCurrentLimit = 20;
            m_configuration.CurrentLimits.SupplyCurrentThreshold = 40;
            m_configuration.CurrentLimits.SupplyTimeThreshold = 0.1;
            m_configuration.CurrentLimits.SupplyCurrentLimitEnable = false;
            m_configuration.CurrentLimits.StatorCurrentLimit = 70;
            m_configuration.CurrentLimits.StatorCurrentLimitEnable = false;

            return m_configuration;
        }
    }

    public static final class SensorConstants {

        public static final int ID_LC1 = 30; //CAN ID for Lasercan 1
        public static final int ID_LC2 = 31; //CAN ID for Lasercan 2
        public static final int PORT_BB1 = 0; //DIO port for beam break

    }



        public static class PhotonVisionConstants {
        public static class front_left_cam {
            public static final String kCameraName = "front_left";
            public static final Transform3d kRobotToCam = new Transform3d(
                    new Translation3d(-0.09, 0.170,0.628),
                    new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(-15)));
        }

        public static class front_right_cam {
            public static final String kCameraName = "front_right";
            public static final Transform3d kRobotToCam = new Transform3d(
                    new Translation3d(-0.09, -0.170,0.628),
                    new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(15)));
        }

        public static class back_right_cam {
            public static final String kCameraName = "back_right";
            public static final Transform3d kRobotToCam = new Transform3d(
                    new Translation3d(-0.143, -0.321,0.534),
                    new Rotation3d(0, 0, Units.degreesToRadians(-155)));
        }
       
        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
        
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

        public static final double VISION_XY_MARGIN = 0.5;
        public static final double VISION_Z_MARGIN = 0.75;
        public static final double VISION_STD_XY_SCALE = 0.02 * 10;
        public static final double VISION_STD_ROT_SCALE = 0.035 * 10;


    }

    public static class LimelightConstants {
        public static final String kCameraName = "limelight";
    }

    public static class FieldConstants {

        public static final Pose2d BLUE_SPEAKER = new Pose2d(Units.inchesToMeters(-1.5 + 12), Units.inchesToMeters(218.42), new Rotation2d(0));
        public static final Pose2d RED_SPEAKER = new Pose2d(Units.inchesToMeters(652.73 - 12), Units.inchesToMeters(218.42), new Rotation2d(Math.PI));
        public static final Pose2d BLUE_FEED = new Pose2d(1.25, 6.2, new Rotation2d(0));
        public static final Pose2d RED_FEED = new Pose2d(15.250, 6.2, new Rotation2d(0));
        public static final Pose2d BLUE_AMP = new Pose2d(Units.inchesToMeters(72.5),Units.inchesToMeters(323.00),new Rotation2d(-Math.PI/2));
        public static final Pose2d RED_AMP = new Pose2d(Units.inchesToMeters(578.77),Units.inchesToMeters(323.00),new Rotation2d(-Math.PI/2));
        public static final double BLUE_AUTO_PENALTY_LINE = 9; // X distance from origin to center of the robot almost fully crossing the midline
        public static final double RED_AUTO_PENALTY_LINE = 7.4; // X distance from origin to center of the robot almost fully crossing the midline
        public static final double FIELD_X_LENGTH = 16.5417;
        public static final double FIELD_Y_LENGTH = 8.0136;

    }

}

