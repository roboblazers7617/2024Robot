// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Head;
import frc.robot.shuffleboard.ArmTab;

import frc.robot.shuffleboard.ClimberTab;
import frc.robot.shuffleboard.DriverStationTab;
import frc.robot.shuffleboard.LEDTab;
import frc.robot.shuffleboard.ShuffleboardInfo;
import frc.robot.shuffleboard.ShuffleboardTabBase;
import frc.robot.shuffleboard.SwerveTab;
import frc.robot.shuffleboard.HeadTab;
import frc.robot.subsystems.Arm;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.commands.MechanismCommands;
import frc.robot.commands.drivetrain.TurnToTag;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	private final ShuffleboardInfo shuffleboard;
	private final Head head = new Head();
	private final LED led = new LED(SerialPort.Port.kMXP, head);
	private final Arm arm = new Arm();
	private final Climber climber = new Climber();
	public final SendableChooser<Command> autoChooser;
	
	// Replace with CommandPS4Controller or CommandJoystick if needed
	private final CommandXboxController driverControllerCommands = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
	private final CommandXboxController operatorControllerCommands = new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);
	private final XboxController driverController = driverControllerCommands.getHID();
	private final XboxController operatorController = operatorControllerCommands.getHID();
	private double speedMultiplier = SwerveConstants.REGULAR_SPEED;
	// private final Vision vision = new Vision();
	private final Drivetrain drivetrain = new Drivetrain(/* vision */);
	
	private final Command absoluteDrive = drivetrain.driveCommand(() -> processJoystickVelocity(driverControllerCommands.getLeftY()), () -> processJoystickVelocity(driverControllerCommands.getLeftX()), () -> processJoystickAngular(driverControllerCommands.getRightX()), () -> processJoystickAngular(driverControllerCommands.getRightY()));
	
	private final Command rotationDrive = drivetrain.driveCommand(() -> processJoystickVelocity(driverControllerCommands.getLeftY()), () -> processJoystickVelocity(driverControllerCommands.getLeftX()), () -> processJoystickAngularButFree(driverControllerCommands.getRightX()));

	private final Command rotationDriveFast = drivetrain.driveCommand(() -> processJoystickVelocity(driverControllerCommands.getLeftY()), () -> processJoystickVelocity(driverControllerCommands.getLeftX()), () -> processJoystickAngularButFree(driverControllerCommands.getRightX()*SwerveConstants.FAST_TURN_TIME));
	
	private final DigitalInput brakeToggleButton = new DigitalInput(ArmConstants.BRAKE_TOGGLE_BUTTON_DIO);
	private boolean isClimbMode = false;
	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// NamedCommands.registerCommand("SayHi", Commands.runOnce(() -> System.out.println("Hi")));
		// NamedCommands.registerCommand("gotoShoot", TempHead.gotoShoot());
		// NamedCommands.registerCommand("Start Intake", TempHead.deployIntake());z
		NamedCommands.registerCommand("turnToSpeaker", turnToSpeaker());
		NamedCommands.registerCommand("turnTo0", pointAwayFromSpeaker());
		NamedCommands.registerCommand("IntakeGround", MechanismCommands.IntakeGround(driverController, operatorController, arm, head));
		NamedCommands.registerCommand("ShootSpeaker", MechanismCommands.ShootSpeakerAuto(arm, head, drivetrain));
		NamedCommands.registerCommand("shootAmp", MechanismCommands.ShootAmp(driverController, operatorController, arm, head));
		NamedCommands.registerCommand("Stow", MechanismCommands.AutoStow(arm, head));
		NamedCommands.registerCommand("turnSideways", turnSideways());
		
		autoChooser = AutoBuilder.buildAutoChooser("Default Path");
		
		// Configure the trigger bindings
		configureBindings();
		shuffleboard = ShuffleboardInfo.getInstance();
		ArrayList<ShuffleboardTabBase> tabs = new ArrayList<>();
		// YOUR CODE HERE | | |
		// \/ \/ \/
		tabs.add(new DriverStationTab(autoChooser, brakeToggleButton));
		
		tabs.add(new ArmTab(arm));
		
		tabs.add(new SwerveTab(drivetrain));
		
		tabs.add(new LEDTab(led));
		
		tabs.add(new HeadTab(head));
		
		tabs.add(new ClimberTab(climber));
		
		// STOP HERE
		shuffleboard.addTabs(tabs);
	}
	
	/**
	 * Use this method to define your trigger->command mappings. Triggers can be
	 * created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
	 * an arbitrary
	 * predicate, or via the named factories in {@link
	 * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
	 * {@link
	 * CommandXboxController
	 * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
	 * PS4} controllers or
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
	 * joysticks}.
	 */
	private void configureBindings() {
		// TODO: (Lukas) There seems to be a bug that if the robot is facing toward the driver station
		// rather than away from it, even if the pose is updated to have the correct angle
		// the joysticks do not correctly drive the robot forward. Everything is reversed.
		drivetrain.setDefaultCommand(absoluteDrive);
		
		// driverControllerCommands.povRight().toggleOnTrue(drivetrain.turnToAngleCommand(Rotation2d.fromDegrees(-15)));
		driverControllerCommands.povRight().whileTrue(turnToSpeaker(() -> processJoystickVelocity(driverController.getLeftY()), () -> processJoystickVelocity(driverController.getLeftX())));
		
		driverControllerCommands.leftBumper()
				.onTrue(new ScheduleCommand(rotationDrive))
				.onFalse(Commands.runOnce(() -> rotationDrive.cancel()));

		driverControllerCommands.leftTrigger()
				.onTrue(new ScheduleCommand(rotationDriveFast))
				.onFalse(Commands.runOnce(() -> rotationDriveFast.cancel()));
				
		driverControllerCommands.rightBumper()
				.onTrue(Commands.runOnce(() -> speedMultiplier = Math.max(.1, speedMultiplier - SwerveConstants.SLOW_SPEED_DECREMENT)))
				.onFalse(Commands.runOnce(() -> speedMultiplier += SwerveConstants.SLOW_SPEED_DECREMENT));
		driverControllerCommands.rightTrigger()
				.onTrue(Commands.runOnce(() -> speedMultiplier = Math.min(1, speedMultiplier + SwerveConstants.FAST_SPEED_INCREMENT)))
				.onFalse(Commands.runOnce(() -> speedMultiplier -= SwerveConstants.FAST_SPEED_INCREMENT));
		
		driverControllerCommands.povLeft()
				.and(() -> checkAllianceColors(Alliance.Red))
				.whileTrue(drivetrain.driveCommand(() -> processJoystickVelocity(driverControllerCommands.getLeftY()), () -> processJoystickVelocity(driverControllerCommands.getLeftX()), () -> Math.cos(Units.degreesToRadians(-150)), () -> Math.sin(Units.degreesToRadians(-150))));
		
		driverControllerCommands.povLeft()
				.and(() -> checkAllianceColors(Alliance.Blue))
				.whileTrue(drivetrain.driveCommand(() -> processJoystickVelocity(driverControllerCommands.getLeftY()), () -> processJoystickVelocity(driverControllerCommands.getLeftX()), () -> Math.cos(Units.degreesToRadians(150)), () -> Math.sin(Units.degreesToRadians(150))));
		
		driverControllerCommands.povUp().onTrue(Commands.runOnce(() -> speedMultiplier = Math.min(1, speedMultiplier + SwerveConstants.PRECISE_INCREMENT)));
		driverControllerCommands.povDown().onTrue(Commands.runOnce(() -> speedMultiplier = Math.max(.1, speedMultiplier - SwerveConstants.PRECISE_INCREMENT)));
		
		driverControllerCommands.start().onTrue(Commands.runOnce(() -> drivetrain.zeroGyro()));
		driverControllerCommands.back().onTrue(Commands.runOnce(() -> drivetrain.disableVisionUpdates()));
		
		driverControllerCommands.a().onTrue(MechanismCommands.ShootOverDBot(driverController, operatorController, arm, head));
		// driverControllerCommands.b().onTrue(MechanismCommands.ShootSpeakerAuto(arm, head, drivetrain));
		
		// driverControllerCommands.a().onTrue(MechanismCommands.ShootSpeaker(arm, head, 2.97));
		// driverControllerCommands.b().onTrue(MechanismCommands.ShootSpeaker(arm, head, 4.27));
		
		arm.setDefaultCommand(arm.ArmDefaultCommand(() -> Math.abs(operatorController.getRightY()) > OperatorConstants.OPERATOR_JOYSTICK_DEADBAND ? -operatorController.getRightY() * ArmConstants.MAX_MANNUAL_ARM_SPEED : 0, () -> Math.abs(operatorController.getLeftY()) > OperatorConstants.OPERATOR_JOYSTICK_DEADBAND ? -operatorController.getLeftY() * ElevatorConstants.MAX_MANUAL_SPEED : 0));
		
		operatorControllerCommands.x().and(() -> !isClimbMode).onTrue(arm.Stow());
		operatorControllerCommands.y().and(() -> !isClimbMode).whileTrue(head.StartOutake()).onFalse(head.StopIntake());
		operatorControllerCommands.a().and(() -> !isClimbMode).onTrue(MechanismCommands.IntakeGround(driverController, operatorController, arm, head).andThen(arm.Stow()));
		operatorControllerCommands.b().and(() -> !isClimbMode).onTrue(MechanismCommands.IntakeSource(driverController, operatorController, arm, head));
		
		operatorControllerCommands.leftTrigger().onTrue(MechanismCommands.PrepareShootAmp(operatorController, arm, head));
		operatorControllerCommands.leftBumper().onTrue(MechanismCommands.ShootAmp(driverController, operatorController, arm, head));
		
		operatorControllerCommands.rightTrigger().onTrue(Commands.runOnce(() -> arm.setArmTarget(ArmConstants.SPEAKER_PODIUM_ANGLE)).andThen(Commands.runOnce(() -> arm.setElevatorTarget(ElevatorConstants.MIN_HEIGHT))).andThen(head.SpinUpShooterForPodium())).onFalse(MechanismCommands.ShootSpeakerPodium(driverController, operatorController, arm, head));
		
		operatorControllerCommands.rightBumper().onTrue(MechanismCommands.PrepareShootSpeakerSubwoofer(driverController, operatorController, arm, head)).onFalse(MechanismCommands.ShootSpeakerSubwoofer(driverController, operatorController, arm, head));
		
		operatorControllerCommands.povLeft()
				.and(() -> (!isClimbMode))
				.whileTrue(head.StopIntake().andThen(head.SpinDownShooter()));
		operatorControllerCommands.povRight()
				.and(() -> (!isClimbMode))
				.onTrue(head.ShootInSpeaker());
		
		operatorControllerCommands.povUp().onTrue(Commands.runOnce(() -> climber.setSpeed(ClimberConstants.CLIMB_RATE, ClimberConstants.CLIMB_RATE), climber)).onFalse(Commands.runOnce(() -> climber.setSpeed(0, 0), climber));
		operatorControllerCommands.povDown().onTrue(Commands.runOnce(() -> climber.setSpeed(-ClimberConstants.CLIMB_RATE, -ClimberConstants.CLIMB_RATE), climber)).onFalse(Commands.runOnce(() -> climber.setSpeed(0, 0), climber));
		
		operatorControllerCommands.x()
				.and(() -> isClimbMode)
				.onTrue(Commands.runOnce(() -> climber.setSpeed(-ClimberConstants.CLIMB_RATE, 0), climber))
				.onFalse(Commands.runOnce(() -> climber.setSpeed(0, 0), climber));
		operatorControllerCommands.y()
				.and(() -> isClimbMode)
				.onTrue(Commands.runOnce(() -> climber.setSpeed(ClimberConstants.CLIMB_RATE, 0), climber))
				.onFalse(Commands.runOnce(() -> climber.setSpeed(0, 0), climber));
		operatorControllerCommands.a()
				.and(() -> isClimbMode)
				.onTrue(Commands.runOnce(() -> climber.setSpeed(0, -ClimberConstants.CLIMB_RATE), climber))
				.onFalse(Commands.runOnce(() -> climber.setSpeed(0, 0), climber));
		operatorControllerCommands.b()
				.and(() -> isClimbMode)
				.onTrue(Commands.runOnce(() -> climber.setSpeed(0, ClimberConstants.CLIMB_RATE), climber))
				.onFalse(Commands.runOnce(() -> climber.setSpeed(0, 0), climber));
		
		operatorControllerCommands.start().onTrue(head.IntakePiece());
		operatorControllerCommands.back().onTrue(Commands.runOnce(() -> {
			isClimbMode = !isClimbMode;
		}));
		
		Trigger brakeToggleTrigger = new Trigger(() -> brakeToggleButton.get());
		brakeToggleTrigger.onTrue(arm.ToggleBrakeModes().andThen(head.ToggleBreakModes()));
		Trigger enableTrigger = new Trigger(() -> DriverStation.isEnabled());
		enableTrigger.onTrue(Commands.runOnce(() -> {
			arm.EnableBrakeMode();
			head.EnableBrakeMode();
		}));
	}
	
	private boolean checkAllianceColors(Alliance checkAgainst) {
		if (DriverStation.getAlliance().isPresent()) {
			return DriverStation.getAlliance().get() == checkAgainst;
		}
		return false;
	}
	
	private double processJoystickVelocity(double joystickInput) {
		return checkAllianceColors(Alliance.Blue) ? (-MathUtil.applyDeadband(joystickInput, OperatorConstants.DRIVER_JOYSTICK_DEADBAND)) * speedMultiplier : MathUtil.applyDeadband(joystickInput, OperatorConstants.DRIVER_JOYSTICK_DEADBAND) * speedMultiplier;
	}
	
	private double processJoystickAngular(double joystickInput) {
		return checkAllianceColors(Alliance.Blue) ? Math.pow(-MathUtil.applyDeadband(joystickInput, OperatorConstants.DRIVER_JOYSTICK_DEADBAND), 3) : Math.pow(MathUtil.applyDeadband(joystickInput, OperatorConstants.DRIVER_JOYSTICK_DEADBAND), 3);
	}
	
	private double processJoystickAngularButFree(double joystickInput) {
		return checkAllianceColors(Alliance.Blue) ? Math.pow(-MathUtil.applyDeadband(joystickInput, OperatorConstants.DRIVER_JOYSTICK_DEADBAND), 3) : Math.pow(-MathUtil.applyDeadband(joystickInput, OperatorConstants.DRIVER_JOYSTICK_DEADBAND), 3);
	}
	
	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return autoChooser.getSelected();
	}
	
	public void setMotorBrake(boolean isBraked) {
		drivetrain.setMotorBrake(isBraked);
	}
	
	public Command turnToSpeaker() {
		if (checkAllianceColors(Alliance.Red)) {
			return new ParallelRaceGroup(new TurnToTag(drivetrain, 4, true), Commands.waitSeconds(1));
		}
		return new ParallelRaceGroup(new TurnToTag(drivetrain, 7, true), Commands.waitSeconds(1));
	}
	
	public Command turnToSpeaker(Supplier<Double> yMovement, Supplier<Double> xMovement) {
		if (checkAllianceColors(Alliance.Red)) {
			return new TurnToTag(drivetrain, 4, true, yMovement, xMovement);
		}
		return new TurnToTag(drivetrain, 7, true, yMovement, xMovement);
	}
	
	public void teleopInit() {
		// arm.teleopInit();
	}
	
	/**
	 * DOES NOT ACTAULLY TURN TO ZERO BE AWARE
	 */
	public Command pointAwayFromSpeaker() {
		if (checkAllianceColors(Alliance.Red)) {
			return drivetrain.turnToAngleCommand(Rotation2d.fromDegrees(180));
		}
		return drivetrain.turnToAngleCommand(Rotation2d.fromDegrees(0));
	}

	public Command turnSideways(){
		if (checkAllianceColors(Alliance.Red)) {
			return drivetrain.turnToAngleCommand(Rotation2d.fromDegrees(-90));
		}
		return drivetrain.turnToAngleCommand(Rotation2d.fromDegrees(90));
	}


}