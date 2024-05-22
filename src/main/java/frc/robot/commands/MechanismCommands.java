package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
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

public class MechanismCommands {
	private final Arm arm;
	private final Head head;
	private final XboxController driverController;
	private final XboxController operatorController;
	
	/**
	 * Creates a new MechanismCommands
	 * 
	 * @param arm
	 *                arm subsystem
	 * @param head
	 *                head subsystem
	 * @param driverController
	 *                driver controller (for haptics)
	 * @param operatorController
	 *                operator controller (for haptics)
	 */
	public MechanismCommands(Arm arm, Head head, XboxController driverController, XboxController operatorController) {
		this.arm = arm;
		this.head = head;
		this.driverController = driverController;
		this.operatorController = operatorController;
	}
	
	/**
	 * will finish after piece has been intaken
	 * 
	 * @return {@link Command}
	 */
	public Command IntakeSource() {
		return head.SpinDownShooter()
				.andThen(() -> arm.setArmTarget(ArmConstants.SOURCE_ANGLE))
				.andThen(() -> arm.setElevatorTarget(ElevatorConstants.MAX_HEIGHT))
				.andThen(head.IntakePiece());
	}
	
	/**
	 * Intakes from the source and triggers haptics once done.
	 * will finish after piece has been intaken
	 * 
	 * @return {@link Command}
	 */
	public Command IntakeSourceWithHaptics() {
		return IntakeSource()
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(driverController, RumbleType.kBothRumble, 0.3, 0.3)))
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(operatorController, RumbleType.kBothRumble, 0.3, 0.3)));
	}
	
	/**
	 * will finish after piece has been intaken
	 * 
	 * @return {@link Command}
	 */
	public Command IntakeGround(boolean stopShooter) {
		return Commands.either(head.SpinDownShooter(), Commands.none(), () -> stopShooter)
				.andThen(() -> arm.setArmTarget(ArmConstants.FLOOR_PICKUP))
				.andThen(() -> arm.setElevatorTarget(ElevatorConstants.MAX_HEIGHT))
				.andThen(head.IntakePiece());
	}
	
	/**
	 * will finish after piece has been intaken
	 * 
	 * @return {@link Command}
	 */
	public Command IntakeGroundWithHaptics() {
		return IntakeGround(true)
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(driverController, RumbleType.kBothRumble, 0.3, 0.3)))
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(operatorController, RumbleType.kBothRumble, 0.3, 0.3)));
	}
	
	/**
	 * cancel shoot and intake and stow
	 * 
	 * @return {@link Command}
	 */
	public Command StowStopIntakeAndShooter() {
		return arm.Stow()
				.andThen(head.StopIntake())
				.andThen(head.SpinDownShooter());
	}
	
	// THIS ISNT CODE DUPLICATION IT DOES A FUNDAMENTALLY DIFFERENT THING!!!!!
	// TODO: I see that this is different, but when would it be used? Should be renamed to better describe
	// what it does. Why doesn't it stop the shooter? Will it be used for auto?
	public Command AutoStowAndStopIntake() {
		return arm.Stow()
				.andThen(head.StopIntake());
	}
	
	/**
	 * will finish when ready
	 * 
	 * @param position
	 *                position to shoot from
	 * @return {@link Command}
	 */
	public Command PrepareShoot(ShootingPosition position) {
		return arm.SetTargets(position)
				.andThen(head.SpinUpShooter(position));
	}
	
	/**
	 * will finish when ready
	 * 
	 * @param distance
	 *                distance to shoot from
	 * @return {@link Command}
	 */
	public Command PrepareShoot(Supplier<Double> distance) {
		return arm.SetTargets(distance)
				.andThen(head.SpinUpShooter(ShootingConstants.VARIABLE_DISTANCE_SHOT));
	}
	
	public Command AutonomousPrepareShoot(Supplier<Double> distance) {
		return arm.SetTargetsAuto(distance)
				.andThen(head.SpinUpShooter(ShootingConstants.AUTO_SHOOT_SPEED));
	}
	
	/**
	 * Prepare to shoot and trigger haptics when ready.
	 * will finish when ready
	 * 
	 * @param position
	 *                position to shoot from
	 * @return {@link Command}
	 */
	public Command PrepareShootWithHaptics(ShootingPosition position) {
		return PrepareShoot(position)
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(operatorController, RumbleType.kBothRumble, 0.3, 0.3)));
	}
	
	/**
	 * Prepare to shoot and trigger haptics when ready.
	 * will finish when ready
	 * 
	 * @param distance
	 *                distance to shoot from
	 * @return {@link Command}
	 */
	public Command PrepareShootWithHaptics(Supplier<Double> distance) {
		return PrepareShoot(distance)
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(operatorController, RumbleType.kBothRumble, 0.3, 0.3)));
	}
	
	/**
	 * will finish after piece has been shot
	 * 
	 * @return {@link Command}
	 */
	public Command Shoot() {
		return Shoot(true);
	}
	
	public Command Shoot(boolean stopShooter) {
		return Commands.waitUntil(() -> arm.areArmAndElevatorAtTarget())
				.andThen(head.Shoot(stopShooter));
	}
	
	public Command AutonomousShoot(ShootingPosition position) {
		return PrepareShoot(position)
				.andThen(Shoot(false));
	}
	
	public Command AutonomousShoot(Drivetrain drivetrain) {
		return AutonomousPrepareShoot(() -> drivetrain.getDistanceToSpeaker())
				.andThen(Shoot(false));
	}
	
	/**
	 * will finish after piece has been shot
	 * 
	 * @return {@link Command}
	 */
	public Command ShootWithHaptics() {
		return Shoot()
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(driverController, RumbleType.kBothRumble, 0.3, 0.3)))
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(operatorController, RumbleType.kBothRumble, 0.3, 0.3)));
	}
}
