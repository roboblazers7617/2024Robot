// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Head;
import frc.robot.shuffleboard.ArmTab;

import frc.robot.shuffleboard.ClimberTab;
import frc.robot.shuffleboard.DriverStationTab;
import frc.robot.shuffleboard.MotorTab;
import frc.robot.shuffleboard.LEDTab;
import frc.robot.shuffleboard.ShuffleboardInfo;
import frc.robot.shuffleboard.ShuffleboardTabBase;
import frc.robot.shuffleboard.SwerveTab;
import frc.robot.shuffleboard.HeadTab;
import frc.robot.subsystems.Arm;
import frc.robot.util.TunableNumber;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveControlRequestParameters;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.commands.MechanismCommands;
import frc.robot.commands.drivetrain.LockWheelsState;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

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
	
	// Replace with CommandPS4Controller or CommandJoystick if needed
	private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
	private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);
	private double speedMultiplier = SwerveConstants.REGULAR_SPEED;
	private final Vision vision = new Vision();
	private final Drivetrain drivetrain = new Drivetrain(vision);
	
	private final Command absoluteDrive = drivetrain.driveCommand(() -> processJoystickVelocity(driverController.getLeftY()), () -> processJoystickVelocity(driverController.getLeftX()), () -> processJoystickAngular(driverController.getRightX()), () -> processJoystickAngular(driverController.getRightY()));
	
	private final Command rotationDrive = drivetrain.driveCommand(() -> processJoystickVelocity(driverController.getLeftY()), () -> processJoystickVelocity(driverController.getLeftX()), () -> processJoystickVelocity(driverController.getRightX()));
	
	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// NamedCommands.registerCommand("SayHi", Commands.runOnce(() -> System.out.println("Hi")));
		
		// Configure the trigger bindings
		configureBindings();
		shuffleboard = ShuffleboardInfo.getInstance();
		ArrayList<ShuffleboardTabBase> tabs = new ArrayList<>();
		// YOUR CODE HERE | | |
		// \/ \/ \/
		tabs.add(new DriverStationTab());
		
		tabs.add(new ArmTab(arm));
		
		//tabs.add(new SwerveTab(drivetrain));

		//tabs.add(new LEDTab(led));

		tabs.add(new HeadTab(head));

		//tabs.add(new ClimberTab(climber));
		
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
		
		driverController.povRight().toggleOnTrue(new LockWheelsState(drivetrain));
		
		driverController.leftBumper()
				.onTrue(new ScheduleCommand(rotationDrive))
				.onFalse(Commands.runOnce(() -> rotationDrive.cancel()));
		driverController.rightBumper()
				.onTrue(Commands.runOnce(() -> speedMultiplier = Math.max(.1, speedMultiplier - SwerveConstants.SLOW_SPEED_DECREMENT)))
				.onFalse(Commands.runOnce(() -> speedMultiplier += SwerveConstants.SLOW_SPEED_DECREMENT));
		driverController.rightTrigger()
				.onTrue(Commands.runOnce(() -> speedMultiplier = Math.min(1, speedMultiplier + SwerveConstants.FAST_SPEED_INCREMENT)))
				.onFalse(Commands.runOnce(() -> speedMultiplier -= SwerveConstants.FAST_SPEED_INCREMENT));
		
		// TODO: (Lukas) Drivers would like a button that when pressed rotates the robot to face
		// the source for pickup so they do not need to manually do this
		driverController.povLeft()
				.and(() -> checkAllianceColors(Alliance.Red))
				.whileTrue(drivetrain.driveCommand(() -> processJoystickVelocity(driverController.getLeftY()), () -> processJoystickVelocity(driverController.getLeftX()), () -> Math.cos(Units.degreesToRadians(-30)), () -> Math.sin(Units.degreesToRadians(-30))));
		
		driverController.povLeft()
				.and(() -> checkAllianceColors(Alliance.Blue))
				.whileTrue(drivetrain.driveCommand(() -> processJoystickVelocity(driverController.getLeftY()), () -> processJoystickVelocity(driverController.getLeftX()), () -> Math.cos(Units.degreesToRadians(150)), () -> Math.sin(Units.degreesToRadians(150))));
		
		driverController.povUp().onTrue(Commands.runOnce(() -> speedMultiplier = Math.min(1, speedMultiplier + SwerveConstants.PRECISE_INCREMENT)));
		driverController.povDown().onTrue(Commands.runOnce(() -> speedMultiplier = Math.max(.1, speedMultiplier - SwerveConstants.PRECISE_INCREMENT)));
		arm.setDefaultCommand(arm.ArmDefaultCommand(() -> Math.abs(operatorController.getRightY()) > OperatorConstants.JOYSTICK_DEADBAND ? -operatorController.getRightY() * ArmConstants.MAX_MANNUAL_ARM_SPEED : 0, () -> Math.abs(operatorController.getLeftY()) > OperatorConstants.JOYSTICK_DEADBAND ? -operatorController.getLeftY() * ElevatorConstants.MAX_MANUAL_SPEED : 0));

		operatorController.x().onTrue(arm.Stow());
		operatorController.y().whileTrue(head.StartOutake()).onFalse(head.StopIntake());
		operatorController.a().onTrue(MechanismCommands.IntakeGround(arm, head));
		operatorController.b().onTrue(MechanismCommands.IntakeSource(arm, head));
		operatorController.leftTrigger().onTrue(head.IdleShooter());
		operatorController.rightTrigger().onTrue(MechanismCommands.ShootSpeaker(arm, head, drivetrain));
		operatorController.leftBumper().onTrue(MechanismCommands.ShootAmp(arm, head));
		operatorController.rightBumper().onTrue(MechanismCommands.ShootSpeakerSubwoofer(arm, head));
		operatorController.start().onTrue(head.StopIntake());
		operatorController.back().onTrue(head.SpinDownShooter());

		operatorController.povUp().onTrue(Commands.runOnce(() -> climber.setSpeed(.2, .2), climber)).onFalse(Commands.runOnce(() -> climber.setSpeed(0, 0), climber));
		operatorController.povDown().onTrue(Commands.runOnce(() -> climber.setSpeed(-.2, -.2), climber)).onFalse(Commands.runOnce(() -> climber.setSpeed(0, 0), climber));
	}
	
	private boolean checkAllianceColors(Alliance checkAgainst) {
		if (DriverStation.getAlliance().isPresent()) {
			return DriverStation.getAlliance().get() == checkAgainst;
		}
		return false;
	}
	
	private double processJoystickVelocity(double joystickInput) {
		return /* checkAllianceColors(Alliance.Blue) ? */ (-MathUtil.applyDeadband(joystickInput, OperatorConstants.JOYSTICK_DEADBAND)) * speedMultiplier; // : MathUtil.applyDeadband(joystickInput, OperatorConstants.JOYSTICK_DEADBAND) * speedMultiplier;
	}
	
	private double processJoystickAngular(double joystickInput) {
		return checkAllianceColors(Alliance.Blue) ? Math.pow(-MathUtil.applyDeadband(joystickInput, OperatorConstants.JOYSTICK_DEADBAND), 3) : Math.pow(MathUtil.applyDeadband(joystickInput, OperatorConstants.JOYSTICK_DEADBAND), 3);
	}
	
	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return new PathPlannerAuto("Do Nothing");
	}
	
	public void setMotorBrake(boolean isBraked) {
		drivetrain.setMotorBrake(isBraked);
	}
}
