// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.LED;
import frc.robot.shuffleboard.DriverStationTab;
import frc.robot.shuffleboard.MotorTab;
import frc.robot.shuffleboard.LEDTab;
import frc.robot.shuffleboard.ShuffleboardInfo;
import frc.robot.shuffleboard.ShuffleboardTabBase;
import frc.robot.shuffleboard.SwerveTab;
import frc.robot.util.TunableNumber;

import java.util.ArrayList;

import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.commands.drivetrain.AbsoluteDriveDirectAngle;
import frc.robot.commands.drivetrain.LockWheelsState;
import frc.robot.commands.drivetrain.AbsoluteDriveAngularRotation;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
	LED led = new LED(SerialPort.Port.kMXP);

	// Replace with CommandPS4Controller or CommandJoystick if needed
	private final CommandXboxController driverController = new CommandXboxController(
			OperatorConstants.DRIVER_CONTROLLER_PORT);
	private double speedMultiplier = SwerveConstants.REGULAR_SPEED;
	private final Vision vision = new Vision();
	private final Drivetrain drivetrain = new Drivetrain(vision);

	//TODO: (Lukas) When using this command, I believe it is causing the bugs where the
	// robot will automatically rotate to an angle when the vision updates the robot at the
	// boot time, when switching back from the angular rotation drive. Could also be the cause
	// of robot moving strangely when within 4 meters of a tag. Either make changes to this 
	// command or look at the way YAGSL Example sets the drive commands in RobotContainer
	private final AbsoluteDriveDirectAngle absoluteDrive = (new AbsoluteDriveDirectAngle(drivetrain,
			() -> (-MathUtil.applyDeadband(driverController.getLeftY()* speedMultiplier, OperatorConstants.JOYSTICK_DEADBAND)),
			() -> (-MathUtil.applyDeadband(driverController.getLeftX()* speedMultiplier, OperatorConstants.JOYSTICK_DEADBAND)),
			() -> (-MathUtil.applyDeadband(driverController.getRightX()* speedMultiplier, OperatorConstants.JOYSTICK_DEADBAND)),
			() -> (-MathUtil.applyDeadband(driverController.getRightY()* speedMultiplier, OperatorConstants.JOYSTICK_DEADBAND))));
	//TODO: (Lukas) Do we want to use this command or just do it the way YAGSL Example code does it?
	private final AbsoluteDriveAngularRotation rotationDrive = (new AbsoluteDriveAngularRotation(drivetrain,
			() -> (-MathUtil.applyDeadband(driverController.getLeftY()* speedMultiplier, OperatorConstants.JOYSTICK_DEADBAND)),
			() -> (-MathUtil.applyDeadband(driverController.getLeftX()* speedMultiplier, OperatorConstants.JOYSTICK_DEADBAND)),
			() -> (-MathUtil.applyDeadband(driverController.getRightX()* speedMultiplier, OperatorConstants.JOYSTICK_DEADBAND))));


	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();

		new TunableNumber();
		shuffleboard = ShuffleboardInfo.getInstance();
		ArrayList<ShuffleboardTabBase> tabs = new ArrayList<>();
		// YOUR CODE HERE | | |
		// \/ \/ \/
		tabs.add(new DriverStationTab());


		tabs.add(MotorTab.getInstance());

		tabs.add(new LEDTab(led));
		tabs.add(new SwerveTab(drivetrain));
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
		//TODO: (Lukas) There seems to be a bug that if the robot is facing toward the driver station
		// rather than away from it, even if the pose is updated to have the correct angle
		// the joysticks do not correctly drive the robot forward. Everything is reversed. 
		drivetrain.setDefaultCommand(absoluteDrive);

		driverController.povDown().toggleOnTrue(new LockWheelsState(drivetrain));
		driverController.leftBumper().onTrue(new ScheduleCommand(rotationDrive))
				.onFalse(Commands.runOnce(() -> rotationDrive.cancel()));
		driverController.rightBumper()
				.onTrue(Commands.runOnce(
						() -> speedMultiplier = SwerveConstants.SLOW_SPEED))
				.onFalse(Commands.runOnce(
						() -> speedMultiplier = SwerveConstants.REGULAR_SPEED));
		driverController.rightTrigger()
				.onTrue(Commands.runOnce(
						() -> speedMultiplier = SwerveConstants.FAST_SPEED))
				.onFalse(Commands.runOnce(
						() -> speedMultiplier = SwerveConstants.REGULAR_SPEED));

		//TODO: (Lukas) Drivers would like a button that when pressed rotates the robot to face
		// the source for pickup so they do not need to manually do this

	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return new PathPlannerAuto("New Path auto");
	}

	public void setMotorBrake(boolean isBraked){
		drivetrain.setMotorBrake(isBraked);
	}
}
