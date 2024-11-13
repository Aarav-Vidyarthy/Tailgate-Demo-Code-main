// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Field;

import org.opencv.photo.Photo;

import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.SensorConstants;
import frc.robot.RobotState.TARGET;
import frc.robot.Util.LaserCanSensor;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

import frc.robot.subsystems.IntakeJoint.IntakeJoint;
import frc.robot.subsystems.IntakeJoint.IntakeJointIO;
import frc.robot.subsystems.IntakeJoint.IntakeJointIOKrakenFOC;
import frc.robot.subsystems.IntakeJoint.IntakeJointIOSim;
import frc.robot.subsystems.IntakeRollers.IntakeRollers;
import frc.robot.subsystems.ShooterJoint.ShooterJoint;
import frc.robot.subsystems.ShooterJoint.ShooterJointIO;
import frc.robot.subsystems.ShooterJoint.ShooterJointIOKrakenFOC;
import frc.robot.subsystems.ShooterJoint.ShooterJointIOSim;
import frc.robot.subsystems.ShooterRollers.ShooterRollers;
import frc.robot.subsystems.ShooterRollers.ShooterRollersIO;
import frc.robot.subsystems.ShooterRollers.ShooterRollersIOKrakenFOC;
import frc.robot.subsystems.ShooterRollers.ShooterRollersIOSim;
public class RobotContainer {

	//TODO: test new shooterjoint positional pid
	//TODO: change shooter rollers to MMVelocity
	//TODO: test auto intake

	public final Drivetrain drivetrain = TunerConstants.DriveTrain;
	public final RobotState robotState = RobotState.getInstance();
	//public final ClimberJoint climberJoint = new ClimberJoint();
	//public final ElevatorJoint elevatorJoint = new ElevatorJoint();
	//public final ElevatorRollers elevatorRollers = new ElevatorRollers();
	//public final IntakeJoint intakeJoint = new IntakeJoint();
	public final IntakeRollers intakeRollers = new IntakeRollers();
	//public final ShooterJoint shooterJoint = new ShooterJoint();
	//public final ShooterRollers shooterRollers = new ShooterRollers();

	/* AdvantageKit Setup */
	public ShooterRollers shooterRollers;
	public IntakeJoint intakeJoint;
	public ShooterJoint shooterJoint;
		
	private final CommandXboxController joystick = new CommandXboxController(0);
	private final GenericHID rumble = joystick.getHID();

	//Lasercan sensors in YSplitRollers to determine note location 
	private final LaserCanSensor lc1 = new LaserCanSensor(SensorConstants.ID_LC1);
	private final LaserCanSensor lc2 = new LaserCanSensor(SensorConstants.ID_LC2);
	private Trigger LC1 = new Trigger(() -> lc1.isClose());
	private Trigger LC2 = new Trigger(() -> lc2.isClose());


	//Beam Break sensor in the ElevatorRollers to determine note location
	private final DigitalInput bb1 = new DigitalInput(SensorConstants.PORT_BB1);
	private final Debouncer ampDebouncer = new Debouncer(.25, DebounceType.kBoth);
	private Trigger BB1 = new Trigger(() -> !bb1.get());

	//Photonvision and Limelight cameras
	//PhotonVision photonVision = new PhotonVision(drivetrain,0);
	PhotonGreece photonGreece = new PhotonGreece(drivetrain);
	Limelight limelight = new Limelight();


    
	private Trigger noteStored = new Trigger(() -> (lc1.isClose() || lc2.isClose())); //Note in YSplitRollers trigger
	private Trigger noteAmp = new Trigger(() -> ampDebouncer.calculate(!bb1.get())); //Note in ElevatorRollers



	private Debouncer notMovingDebouncer = new Debouncer(0.5,DebounceType.kRising);
	private Trigger notMoving = new Trigger(() -> notMovingDebouncer.calculate(drivetrain.getCurrentRobotChassisSpeeds().vxMetersPerSecond < .1));

	private Trigger readyToShoot = new Trigger(
			() -> (shooterRollers.getState() != ShooterRollers.State.OFF) && shooterRollers.atGoal() &&
					(shooterJoint.getState() != ShooterJoint.State.STOW) && shooterJoint.atGoal() &&
					drivetrain.atGoal());

  			

	private Trigger scoreRequested = joystick.rightTrigger(); //Binding right trigger to request scoring
	
	//Climbing Triggers
	private boolean climbRequested = false; //Whether or not a climb request is active
	private Trigger climbRequest = new Trigger(() -> climbRequested); //Trigger for climb request
	private int climbStep = 0; //Tracking what step in the climb sequence we are on

	//Triggers for each step of the climb sequence
	private Trigger climbStep0 = new Trigger(() -> climbStep == 0);
	private Trigger climbStep1 = new Trigger(() -> climbStep == 1);
	private Trigger climbStep2 = new Trigger(() -> climbStep == 2);
	private Trigger climbStep3 = new Trigger(() -> climbStep >= 3);

	private SendableChooser<Command> autoChooser;



	private final Telemetry logger = new Telemetry(Constants.DriveConstants.MaxSpeed);

	public RobotContainer() {
		/* base them on Null before we beform Switch Statement check */
		shooterRollers = null;
		intakeJoint = null;
		shooterJoint = null;

		/* Setup according to Which Robot we are using */

		if (Constants.currentMode != Constants.Mode.REPLAY) {
			switch (Constants.currentMode) {
				case REAL:
					shooterRollers = new ShooterRollers(new ShooterRollersIOKrakenFOC());
					intakeJoint = new IntakeJoint(new IntakeJointIOKrakenFOC());
					shooterJoint = new ShooterJoint(new ShooterJointIOKrakenFOC());
					break;
					/* We will include the other subsystems */
				case SIM:
					shooterRollers = new ShooterRollers(new ShooterRollersIOSim());
					intakeJoint = new IntakeJoint(new IntakeJointIOSim());
					shooterJoint = new ShooterJoint(new ShooterJointIOSim());
					break;
				default:
			}
		}

		if (shooterRollers == null) {
			shooterRollers = new ShooterRollers(new ShooterRollersIO() {});
		}
		if (intakeJoint == null) {
			intakeJoint = new IntakeJoint(new IntakeJointIO() {});
		}
		if (shooterJoint == null) {
			shooterJoint = new ShooterJoint(new ShooterJointIO() {});
		}
	
	

		configureBindings();
		configureDebugCommands();
		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Chooser", autoChooser);
	}

	private void configureBindings() {
		drivetrain.setDefaultCommand(drivetrain.run(() -> drivetrain.setControllerInput(-joystick.getLeftY(),
				-joystick.getLeftX(), -joystick.getRightX())));

		drivetrain.registerTelemetry(logger::telemeterize);


		joystick.leftTrigger().and(LC2).whileTrue(Commands.startEnd(() -> rumble.setRumble(GenericHID.RumbleType.kBothRumble, 1), () -> rumble.setRumble(GenericHID.RumbleType.kBothRumble, 0)));

		//Subwoofer
		joystick.a().whileTrue(
				Commands.parallel(
						robotState.setTargetCommand(RobotState.TARGET.SUBWOOFER),
						shooterRollers.setStateCommand(ShooterRollers.State.SUBWOOFER),
						shooterJoint.setStateCommand(ShooterJoint.State.SUBWOOFER)));

		joystick.rightBumper().and(noteAmp).whileTrue(Commands.startEnd(() -> rumble.setRumble(GenericHID.RumbleType.kBothRumble, 1), () -> rumble.setRumble(GenericHID.RumbleType.kBothRumble, 0)));

		//Feed
		joystick.y().whileTrue(
				Commands.parallel(
						robotState.setTargetCommand(RobotState.TARGET.FEED),
						shooterRollers.setStateCommand(ShooterRollers.State.FEED),
						shooterJoint.setStateCommand(ShooterJoint.State.DYNAMIC)));

		//Speaker
		joystick.x().whileTrue(
				Commands.parallel(
						robotState.setTargetCommand(RobotState.TARGET.SPEAKER),
						shooterRollers.setStateCommand(ShooterRollers.State.SPEAKER),
						shooterJoint.setStateCommand(ShooterJoint.State.DYNAMIC)));

		//Climb Request (toggle)
		joystick.back().onTrue(Commands.runOnce(() -> climbRequested = !climbRequested));

		//Climb sequence next step
		joystick.start().onTrue(Commands.runOnce(() -> climbStep += 1));

		//Slow drivetrain to 25% while climbing
		climbRequest.whileTrue(drivetrain.run(() -> drivetrain.setControllerInput(-joystick.getLeftY()*0.5,
		-joystick.getLeftX()*0.5, -joystick.getRightX())));

		
		joystick.povUp().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative(new Pose2d(1.332, 5.529, new Rotation2d(Math.PI)))));

	}



	private void configureDebugCommands() {
		SmartDashboard.putData("Intake Eject",Commands.parallel(intakeRollers.setStateCommand(IntakeRollers.State.EJECT)));
		SmartDashboard.putData("Intake Deploy",Commands.parallel(intakeJoint.setStateCommand(IntakeJoint.State.INTAKE)));
		SmartDashboard.putData("Shooter Eject",Commands.parallel(shooterRollers.setStateCommand(ShooterRollers.State.REVERSE)));
		SmartDashboard.putData("Shooter Tuning Angle",Commands.parallel(shooterJoint.setStateCommand(ShooterJoint.State.TUNING)));
		SmartDashboard.putData("Shooter Climber Clearance",Commands.parallel(shooterJoint.setStateCommand(ShooterJoint.State.CLIMBCLEARANCE)));
		SmartDashboard.putData("Shooter Roller Speaker",
				Commands.parallel(shooterRollers.setStateCommand(ShooterRollers.State.SPEAKER),
						robotState.setTargetCommand(TARGET.SPEAKER)));
        SmartDashboard.putData("Shooter Roller Sub",Commands.parallel(shooterRollers.setStateCommand(ShooterRollers.State.SUBWOOFER)));
		SmartDashboard.putData("Shooter Roller Speed TUNING",Commands.parallel(shooterRollers.setStateCommand(ShooterRollers.State.TUNING)));

		SmartDashboard.putData("Reset Climber Index",Commands.runOnce(() -> climbStep = 0));
	}

    public void displaySystemInfo() {
        SmartDashboard.putBoolean("Beam Break 1", BB1.getAsBoolean());
		SmartDashboard.putBoolean("Lasercan 1", LC1.getAsBoolean());
		SmartDashboard.putBoolean("Lasercan 2", LC2.getAsBoolean());
		SmartDashboard.putBoolean("readyToScore",readyToShoot.getAsBoolean());
        SmartDashboard.putBoolean("Note in YSplit", noteStored.getAsBoolean());
        SmartDashboard.putBoolean("Note in Amp", noteAmp.getAsBoolean());
		SmartDashboard.putBoolean("Climb Requested", climbRequest.getAsBoolean());
		SmartDashboard.putNumber("Climb Step", climbStep);
		SmartDashboard.putBoolean("Not Moving Trigger", notMoving.getAsBoolean());
    }

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
