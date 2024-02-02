// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.shuffleboard.DriverStationTab;
import frc.robot.shuffleboard.ShuffleboardInfo;
import frc.robot.shuffleboard.ShuffleboardTabBase;
import frc.robot.shuffleboard.SwerveTab;
import frc.robot.util.TunableNumber;

import java.util.ArrayList;

import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.commands.drivetrain.AbsoluteDrive;
import frc.robot.commands.drivetrain.LockWheelsState;
import frc.robot.commands.drivetrain.VelocityRotationDrive;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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

	// Replace with CommandPS4Controller or CommandJoystick if needed
	private final CommandXboxController driverController = new CommandXboxController(
			OperatorConstants.DRIVER_CONTROLLER_PORT);
	private final Drivetrain drivetrain = new Drivetrain();

	private final AbsoluteDrive absoluteDrive = (new AbsoluteDrive(drivetrain,
			() -> (-MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstants.JOYSTICK_DEADBAND)),
			() -> (-MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstants.JOYSTICK_DEADBAND)),
			() -> (-MathUtil.applyDeadband(driverController.getRightX(), OperatorConstants.JOYSTICK_DEADBAND)),
			() -> (-MathUtil.applyDeadband(driverController.getRightY(), OperatorConstants.JOYSTICK_DEADBAND))));

	private final VelocityRotationDrive rotationDrive = (new VelocityRotationDrive(drivetrain,
			() -> (-MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstants.JOYSTICK_DEADBAND)),
			() -> (-MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstants.JOYSTICK_DEADBAND)),
			() -> (-MathUtil.applyDeadband(driverController.getRightX(), OperatorConstants.JOYSTICK_DEADBAND))));

	//TODO: Use this instead
	   // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    // Command driveFieldOrientedAnglularVelocity = drivetrain.driveCommand(
			// () -> (-MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstants.JOYSTICK_DEADBAND)),
			// () -> (-MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstants.JOYSTICK_DEADBAND)),
			// () -> (-MathUtil.applyDeadband(driverController.getRightX(), OperatorConstants.JOYSTICK_DEADBAND))));

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();

		new TunableNumber();
		shuffleboard = ShuffleboardInfo.getInstance();
		ArrayList<ShuffleboardTabBase> tabs = new ArrayList<>();
		// YOUR CODE HERE |  |  |
		// 				 \/ \/ \/
		tabs.add(new DriverStationTab());

		tabs.add(new SwerveTab(null));
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
		drivetrain.setDefaultCommand(absoluteDrive);

		driverController.povDown().toggleOnTrue(new LockWheelsState(drivetrain));
		driverController.leftBumper().onTrue(new ScheduleCommand(rotationDrive)).onFalse(Commands.runOnce(() -> rotationDrive.cancel()));

		/*
		 * driverController.povLeft().onTrue(
		 * Commands.either(
		 * Commands.parallel(Commands.runOnce(() ->
		 * drivetrain.setDefaultCommand(absoluteDriveState))
		 * .andThen(new ScheduleCommand(absoluteDriveState)),
		 * Commands.print(drivetrain.getDefaultCommand().getName())),
		 * Commands.parallel(
		 * Commands.runOnce(() -> drivetrain.setDefaultCommand(fieldCentricDriveState),
		 * drivetrain)
		 * .andThen(new ScheduleCommand(fieldCentricDriveState)),
		 * Commands.print(drivetrain.getDefaultCommand().getName())),
		 * this::isFieldCentric));
		 */
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
}
