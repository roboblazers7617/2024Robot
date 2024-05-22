package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.Constants.ShootingConstants.ShootingPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Head;
import frc.robot.commands.HapticController;

public class MechanismCommands {
	private final Arm arm;
	private final Head head;
	private final HapticController driverController;
	private final HapticController operatorController;
	
	/**
	 * Creates a new MechanismCommands
	 * 
	 * @param arm
	 *                arm subsystem
	 * @param head
	 *                head subsystem
	 * @param driverController
	 *                haptic controller for the driver controller
	 * @param operatorController
	 *                haptic controller for the operator controller
	 */
	public MechanismCommands(Arm arm, Head head, HapticController driverController, HapticController operatorController) {
		this.arm = arm;
		this.head = head;
		this.driverController = driverController;
		this.operatorController = operatorController;
	}
	
	/**
	 * Intake from the source
	 * Finishes after piece has been intaken
	 * 
	 * @return {@link Command}
	 */
	public Command IntakeSource() {
		return head.SpinDownShooter()
				.andThen(() -> arm.setArmTarget(ArmConstants.SOURCE_ANGLE))
				.andThen(() -> arm.setElevatorTarget(ElevatorConstants.MAX_HEIGHT))
				.andThen(head.IntakePiece())
				.andThen(new ScheduleCommand(driverController.HapticTap(RumbleType.kBothRumble, 0.3, 0.3)))
				.andThen(new ScheduleCommand(operatorController.HapticTap(RumbleType.kBothRumble, 0.3, 0.3)));
	}
	
	/**
	 * Intake from the ground
	 * Finishes after piece has been intaken
	 * 
	 * @param stopShooter
	 *                should the shooter be stopped before intaking?
	 * @return {@link Command}
	 */
	public Command IntakeGround(boolean stopShooter) {
		return Commands.either(head.SpinDownShooter(), Commands.none(), () -> stopShooter)
				.andThen(() -> arm.setArmTarget(ArmConstants.FLOOR_PICKUP))
				.andThen(() -> arm.setElevatorTarget(ElevatorConstants.MAX_HEIGHT))
				.andThen(head.IntakePiece())
				.andThen(new ScheduleCommand(driverController.HapticTap(RumbleType.kBothRumble, 0.3, 0.3)))
				.andThen(new ScheduleCommand(operatorController.HapticTap(RumbleType.kBothRumble, 0.3, 0.3)));
	}
	
	/**
	 * Cancel shoot, intake, and stow
	 * 
	 * @return {@link Command}
	 */
	public Command StowStopIntakeAndShooter() {
		return arm.Stow()
				.andThen(head.StopIntake())
				.andThen(head.SpinDownShooter());
	}
	
	// THIS ISNT CODE DUPLICATION IT DOES A FUNDAMENTALLY DIFFERENT THING!!!!!
	// TODO: I see that this is different, but when would it be used? Should be renamed to better describe what it does. Why doesn't it stop the shooter? Will it be used for auto?
	public Command AutoStowAndStopIntake() {
		return arm.Stow()
				.andThen(head.StopIntake());
	}
	
	/**
	 * Prepare to shoot from a {@link ShootingPosition}
	 * Finishes when ready
	 * 
	 * @param position
	 *                position to shoot from
	 * @return {@link Command}
	 */
	public Command PrepareShoot(ShootingPosition position) {
		return arm.SetTargets(position)
				.andThen(head.SpinUpShooter(position))
				.andThen(new ScheduleCommand(operatorController.HapticTap(RumbleType.kBothRumble, 0.3, 0.3)));
	}
	
	/**
	 * Prepare to shoot from a distance
	 * Finishes when ready
	 * 
	 * @param distance
	 *                distance to shoot from
	 * @return {@link Command}
	 */
	public Command PrepareShoot(Supplier<Double> distance) {
		return arm.SetTargets(distance)
				.andThen(head.SpinUpShooter(ShootingConstants.VARIABLE_DISTANCE_SHOT))
				.andThen(new ScheduleCommand(operatorController.HapticTap(RumbleType.kBothRumble, 0.3, 0.3)));
	}
	
	public Command AutonomousPrepareShoot(Supplier<Double> distance) {
		return arm.SetTargetsAuto(distance)
				.andThen(head.SpinUpShooter(ShootingConstants.AUTO_SHOOT_SPEED));
	}
	
	/**
	 * Shoot a note
	 * Finishes after piece has been shot
	 * 
	 * @return {@link Command}
	 */
	public Command Shoot() {
		return Shoot(true);
	}
	
	/**
	 * Shoot a note
	 * Finishes after piece has been shot
	 * 
	 * @param stopShooter
	 *                should the shooter be stopped after shooting?
	 * @return {@link Command}
	 */
	public Command Shoot(boolean stopShooter) {
		return Commands.waitUntil(() -> arm.areArmAndElevatorAtTarget())
				.andThen(head.Shoot(stopShooter))
				.andThen(new ScheduleCommand(driverController.HapticTap(RumbleType.kBothRumble, 0.3, 0.3)))
				.andThen(new ScheduleCommand(operatorController.HapticTap(RumbleType.kBothRumble, 0.3, 0.3)));
	}
	
	public Command AutonomousShoot(ShootingPosition position) {
		return PrepareShoot(position)
				.andThen(Shoot(false));
	}
	
	public Command AutonomousShoot(Drivetrain drivetrain) {
		return AutonomousPrepareShoot(() -> drivetrain.getDistanceToSpeaker())
				.andThen(Shoot(false));
	}
}
