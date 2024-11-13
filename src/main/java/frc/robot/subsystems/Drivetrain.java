package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

import org.littletonrobotics.junction.Logger;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class Drivetrain extends SwerveDrivetrain implements Subsystem {

    @RequiredArgsConstructor
    @Getter
    public enum State {
        TELEOP,
        HEADING,
        CARDINAL,
        RELATIVE;
    }

    @Setter
    private State state = State.TELEOP;

    public State getDrivetrainState() {
        return state;
    }

    public Field2d fieldMap = new Field2d();
    
    private RobotState.TARGET target = RobotState.TARGET.NONE;

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private double xVelocity = 0.0;
    private double yVelocity = 0.0;
    private double omegaVelocity = 0.0;

    private final ModuleConfig moduleConfig = new ModuleConfig(Units.inchesToMeters(3.95/2), 5.1, 1.2, DCMotor.getKrakenX60(1).withReduction(6.122), 90, 1);
    
    private final RobotConfig robotConfig = new RobotConfig(Units.lbsToKilograms(129), 4.785, moduleConfig, Units.inchesToMeters(10.375*2), Units.inchesToMeters(10.375*2));


    private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();

    private SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
            .withDeadband(DriveConstants.MaxSpeed * 0.01).withRotationalDeadband(DriveConstants.MaxAngularRate * 0.01)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(DriveConstants.MaxSpeed * 0.01).withRotationalDeadband(DriveConstants.MaxAngularRate * 0.01)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


/*     private SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(DriveConstants.MaxSpeed * 0.1).withRotationalDeadband(DriveConstants.MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.Velocity); */

    // private final SwerveRequest.SwerveDriveBrake brake = new
    // SwerveRequest.SwerveDriveBrake();

    public Drivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        setHeadingPID();
        setSwerveDriveCustomCurrentLimits();
        SmartDashboard.putData("Robot Pose Field Map",fieldMap);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

		AutoBuilder.configure(
            () -> this.getState().Pose, // Supplier
            this::seedFieldRelative, // Reset Pose
            this::getCurrentRobotChassisSpeeds, //Robot Relative Speed Supplier
            (speeds) -> this.setControl(AutoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new PPHolonomicDriveController(new PIDConstants(5, 0, 0.08), // Translation PID
                new PIDConstants(5, 0, 0)), // Rotational PID
            robotConfig, //Robot Config
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, //Alliance Flip
            this);  // Subsystem for requirements
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void simulationPeriodic() {
        /* Assume 20ms update rate, get battery voltage from WPILib */
        updateSimState(0.02, RobotController.getBatteryVoltage());
    }

    @Override
    public void periodic() {

        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            xVelocity = -xVelocity;
            yVelocity = -yVelocity;
        }

        RobotState.getInstance().setRobotPose(getState().Pose); // Tell RobotState current pose
        RobotState.getInstance().setRobotSpeeds(getCurrentRobotChassisSpeeds()); // Tell RobotState current speeds
        
        //MJW: Added Logging for replay 11/11/2024
        Logger.recordOutput("Drivebase/RobotPose", getState().Pose);
        fieldMap.setRobotPose(getState().Pose);
        
        //MJW: Added Logging for replay 11/11/2024
        SmartDashboard.putNumber("Distance To Target", RobotState.getInstance().getDistanceToTarget());
        Logger.recordOutput("Drivebase/DistanceToTarget", RobotState.getInstance().getDistanceToTarget());

        target = RobotState.getInstance().getTarget();

        switch (target) {
            case NONE:
                setState(State.TELEOP);
                break;
            case SPEAKER:
                setState(State.HEADING);
                break;
            case FEED:
                setState(State.HEADING);
                break;
            case AMP:
                setState(State.CARDINAL);
                break;
            case NOTE:
                setState(State.RELATIVE);
                break;
            default: 
                setState(State.TELEOP);
                break;
        }

        switch (state) {
            case TELEOP -> {
                this.setControl(
                    fieldCentric
                        .withVelocityX(xVelocity)
                        .withVelocityY(yVelocity)
                        .withRotationalRate(omegaVelocity));
                        break;
            }
            
            case HEADING -> {
                this.setControl(fieldCentricFacingAngle
                        .withVelocityX(xVelocity * 0.6)
                        .withVelocityY(yVelocity * 0.6)
                        .withTargetDirection(RobotState.getInstance().getAngleToTarget()));
                        break;
            }
            case CARDINAL -> {
                this.setControl(fieldCentricFacingAngle
                        .withVelocityX(xVelocity)
                        .withVelocityY(yVelocity)
                        .withTargetDirection(RobotState.getInstance().getAngleOfTarget()));
                        break;
            }
            case RELATIVE -> { //TODO: Make this less specific, fix for auto vs teleop
/*                 if (RobotState.getInstance().getAngleToNote().isPresent()) {
                    this.setControl(robotCentric
                        .withVelocityX(-DriveConstants.MaxSpeed * (1 - Math.abs(RobotState.getInstance().getAngleToNote().getAsDouble()) / 32) * .25)
                        .withVelocityY(0)
                        //.withVelocityY(-DriveConstants.MaxSpeed * (1 - Math.abs(RobotState.getInstance().getAngleToNote()) / 32) * .1)
                        .withRotationalRate(RobotState.getInstance().getAngleToNote().getAsDouble()/10)); //TODO: TUNE THIS VALUE
                } else {
                    this.setControl(robotCentric
                    	.withVelocityX(controllerX * DriveConstants.MaxSpeed)
                    	.withVelocityY(controllerY * DriveConstants.MaxSpeed)
                    	.withRotationalRate(0));
                }
                break; */
                this.setControl(fieldCentric
                        .withVelocityX(xVelocity)
                        .withVelocityY(yVelocity)
                        .withRotationalRate(omegaVelocity));
                        break;
                
            }
            default -> {
                break;
                
            }
        }

        SmartDashboard.putString(this.getClass().getSimpleName() + " State ", state.toString());
        SmartDashboard.putString(this.getClass().getSimpleName() + " Target ", target.toString());
        SmartDashboard.putNumber("Drivetrain Heading (deg)",fieldCentricFacingAngle.HeadingController.getSetpoint());
        SmartDashboard.putBoolean("Drivetrain at Heading", fieldCentricFacingAngle.HeadingController.atSetpoint());

    }

    private void setHeadingPID() {
        fieldCentricFacingAngle.HeadingController.setPID(10, 0, 0);
        fieldCentricFacingAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        fieldCentricFacingAngle.HeadingController.setTolerance(Units.degreesToRadians(DriveConstants.headingAngleTolerance)); 

        SmartDashboard.putData("Angle PID",fieldCentricFacingAngle.HeadingController);
    }

    public void setControllerInput(double controllerX, double controllerY, double controllerOmega) {
         this.xVelocity = MathUtil.applyDeadband(controllerX, 0.1, 1) * DriveConstants.MaxSpeed * DriveConstants.driverSpeed;
        this.yVelocity = MathUtil.applyDeadband(controllerY, 0.1,1) * DriveConstants.MaxSpeed * DriveConstants.driverSpeed;
        this.omegaVelocity = MathUtil.applyDeadband(controllerOmega, 0.1,1) * DriveConstants.MaxAngularRate * DriveConstants.driverSpeed;
        //this.xVelocity = controllerX * DriveConstants.MaxSpeed;
        //this.yVelocity = controllerY * DriveConstants.MaxSpeed;
        //this.omegaVelocity = controllerOmega * DriveConstants.MaxAngularRate;
    }

    public boolean atGoal() {
        if (state == State.TELEOP) {
            return true;
        } else {
            return fieldCentricFacingAngle.HeadingController.atSetpoint();
        }

    }

/*     public Command setStateCommand(State state) {
        return startEnd(() -> setState(state), () -> setState(State.TELEOP));
    } */

    public void setSwerveDriveCustomCurrentLimits() {
        // Create a current configuration to use for the drive motor of each swerve
        // module.
        var customCurrentLimitConfigs = new CurrentLimitsConfigs();
        var customMotorConfigs = new TalonFXConfiguration();

        // Iterate through each module.
        for (var module : Modules) {
            // Get the Configurator for the current drive motor.
            var currentConfigurator = module.getDriveMotor().getConfigurator();

            // Refresh the current configuration, since the stator current limit has already
            // been set.
            currentConfigurator.refresh(customCurrentLimitConfigs);
            currentConfigurator.refresh(customMotorConfigs);

            // Set all of the parameters related to the supply current. The values should
            // come from Constants.
            customCurrentLimitConfigs.SupplyCurrentLimit = 60;
            customCurrentLimitConfigs.SupplyCurrentThreshold = 80;
            customCurrentLimitConfigs.SupplyTimeThreshold = .1;
            customCurrentLimitConfigs.SupplyCurrentLimitEnable = true;

            customCurrentLimitConfigs.StatorCurrentLimit = 90;
            customCurrentLimitConfigs.StatorCurrentLimitEnable = true;

            customMotorConfigs.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
            customMotorConfigs.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;
            customMotorConfigs.Voltage.PeakForwardVoltage = 12.0;
            customMotorConfigs.Voltage.PeakReverseVoltage = 12.0;
            customMotorConfigs.CurrentLimits = customCurrentLimitConfigs;

            
            
            

            // Apply the new current limit configuration.
            //currentConfigurator.apply(customCurrentLimitConfigs);
            currentConfigurator.apply(customMotorConfigs);
        }
    }
}
