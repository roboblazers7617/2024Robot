package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ShootingConstants.ShootingPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Head;

public class MechanismCommands {
	/**
	 * will finish after piece has been intaken
	 * 
	 * @param arm
	 *            subsystem
	 * @param head
	 *            subsystem
	 * @return Command
	 */
	public static Command IntakeSource(Arm arm, Head head) {
		return head.SpinDownShooter()
				.andThen(() -> arm.setArmTarget(ArmConstants.SOURCE_ANGLE))
				.andThen(() -> arm.setElevatorTarget(ElevatorConstants.MAX_HEIGHT))
				.andThen(head.IntakePiece());
	}
	
	/**
	 * will finish after piece has been intaken
	 * 
	 * @param driverController
	 *            for haptics
	 * @param operatorController
	 *            for haptics
	 * @param arm
	 *            subsystem
	 * @param head
	 *            subsystem
	 * @return Command
	 */
	public static Command IntakeSource(XboxController driverController, XboxController operatorController, Arm arm, Head head) {
		return IntakeSource(arm, head)
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(driverController, RumbleType.kBothRumble, 0.3, 0.3)))
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(operatorController, RumbleType.kBothRumble, 0.3, 0.3)));
	}
	
	/**
	 * will finish after piece has been intaken
	 * 
	 * @param arm
	 *            subsystem
	 * @param head
	 *            subsystem
	 * @return Command
	 */
	public static Command IntakeGround(Arm arm, Head head) {
		return head.SpinDownShooter()
				.andThen(() -> arm.setArmTarget(ArmConstants.FLOOR_PICKUP))
				.andThen(() -> arm.setElevatorTarget(ElevatorConstants.MAX_HEIGHT))
				.andThen(head.IntakePiece());
	}
	
	/**
	 * will finish after piece has been intaken
	 * 
	 * @param driverController
	 *            for haptics
	 * @param operatorController
	 *            for haptics
	 * @param arm
	 *            subsystem
	 * @param head
	 *            subsystem
	 * @return Command
	 */
	public static Command IntakeGround(XboxController driverController, XboxController operatorController, Arm arm, Head head) {
		return IntakeGround(arm, head)
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(driverController, RumbleType.kBothRumble, 0.3, 0.3)))
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(operatorController, RumbleType.kBothRumble, 0.3, 0.3)));
	}
	
	/**
	 * cancel shoot and intake and stow
	 * 
	 * @param arm
	 *            subsystem
	 * @param head
	 *            subsystem
	 * @return
	 */
	public static Command Stow(Arm arm, Head head) {
		return arm.Stow()
				.andThen(head.StopIntake())
				.andThen(head.SpinDownShooter());
	}
	//THIS ISNT CODE DUPLICATION IT DOES A FUNDAMENTALLY DIFFERENT THING!!!!!
	public static Command AutoStow(Arm arm, Head head) {
		return arm.Stow()
				.andThen(head.StopIntake());
	}

	
	/**
	 * will finish when ready
	 * @param arm
	 * @param head
	 * @param position
	 */
	public static Command PrepareShoot(Arm arm, Head head, ShootingPosition position) {
		return arm.SetTargets(position)
				.andThen(head.SpinUpShooter(position))
				.andThen(Commands.waitUntil(() -> head.isReadyToShoot()))
				.andThen(arm.WaitUntilArmAtTarget())
				.andThen(arm.WaitUntilElevatorAtTarget());
	}
	
	/**
	 * will finish when ready
	 * @param operatorController
	 * @param arm
	 * @param head
	 * @param position
	 * @return
	 */
	public static Command PrepareShoot(XboxController operatorController, Arm arm, Head head, ShootingPosition position) {
		return PrepareShoot(arm, head, position)
				.andThen(Commands.waitUntil(() -> head.isReadyToShoot()))
				.andThen(arm.WaitUntilArmAtTarget())
				.andThen(arm.WaitUntilElevatorAtTarget())
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(operatorController, RumbleType.kBothRumble, 0.3, 0.3)));
	}
	
	/**
	 * will finish after piece has been shot
	 * 
	 * @param arm
	 *            subsystem
	 * @param head
	 *            subsystem
	 * @param position
	 *            position to shoot from
	 * @return Command
	 */
	public static Command Shoot(Arm arm, Head head, ShootingPosition position) {
		return PrepareShoot(arm, head, position)
				.andThen(head.Shoot(position))
				.andThen(arm.Stow());
	}
	
	/**
	 * will finish after piece has been shot
	 * 
	 * @param driverController
	 *            for haptics
	 * @param operatorController
	 *            for haptics
	 * @param arm
	 *            subsystem
	 * @param head
	 *            subsystem
	 * @param position
	 *            position to shoot from
	 * @return Command
	 */
	public static Command Shoot(XboxController driverController, XboxController operatorController, Arm arm, Head head, ShootingPosition position) {
		return Shoot(arm, head, position)
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(driverController, RumbleType.kBothRumble, 0.3, 0.3)))
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(operatorController, RumbleType.kBothRumble, 0.3, 0.3)));
	}
	
	/**
	 * will wait until finished shooting to finish command
	 * 
	 * @param arm
	 *            subsystem
	 * @param head
	 *            subsystem
	 * @param distance
	 *            in meters
	 * @return Command
	 */
	public static Command Shoot(Arm arm, Head head, Double distance) {
		return Commands.runOnce(() -> arm.setArmTargetByDistance(distance))
				.andThen(() -> arm.setElevatorTarget(ElevatorConstants.MAX_HEIGHT))
				.andThen(arm.WaitUntilElevatorAtTarget())
				.andThen(arm.WaitUntilArmAtTarget())
				.andThen(head.Shoot(ShootingPosition.SUBWOOFER));
	}
	
	/**
	 * will wait until finished shooting to finish command
	 * 
	 * @param arm
	 *            subsystem
	 * @param head
	 *            subsystem
	 * @param distance
	 *            in meters
	 * @return Command
	 */
	public static Command Shoot(Arm arm, Head head, Supplier<Double> distance) {
		return Shoot(arm, head, distance.get());
	}
	
	/**
	 * will wait until finished shooting to finish command
	 * 
	 * @param driverController
	 *            for haptics
	 * @param operatorController
	 *            for haptics
	 * @param arm
	 *            subsystem
	 * @param head
	 *            subsystem
	 * @param distance
	 *            in meters
	 * @return Command
	 */
	public static Command Shoot(XboxController driverController, XboxController operatorController, Arm arm, Head head, Double distance) {
		return Shoot(arm, head, distance)
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(driverController, RumbleType.kBothRumble, 0.3, 0.3)))
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(operatorController, RumbleType.kBothRumble, 0.3, 0.3)));
	}
	
	/**
	 * will wait until finished shooting to finish command
	 * 
	 * @param driverController
	 *            for haptics
	 * @param operatorController
	 *            for haptics
	 * @param arm
	 *            subsystem
	 * @param head
	 *            subsystem
	 * @param distance
	 *            in meters
	 * @return Command
	 */
	public static Command Shoot(XboxController driverController, XboxController operatorController, Arm arm, Head head, Supplier<Double> distance) {
		return Shoot(driverController, operatorController, arm, head, distance.get());
	}
	
	/**
	 * will finish after piece has been shot
	 * 
	 * @param arm
	 *            subsystem
	 * @param head
	 *            subsystem
	 * @param drivetrain
	 *            subsystem
	 * @return Command
	 */
	public static Command Shoot(Arm arm, Head head, Drivetrain drivetrain) {
		return Shoot(arm, head, () -> drivetrain.getDistanceToSpeaker());
	}
	
	/**
	 * will finish after piece has been shot
	 * 
	 * @param driverController
	 *            for haptics
	 * @param operatorController
	 *            for haptics
	 * @param arm
	 *            subsystem
	 * @param head
	 *            subsystem
	 * @param drivetrain
	 *            subsystem
	 * @return Command
	 */
	public static Command Shoot(XboxController driverController, XboxController operatorController, Arm arm, Head head, Drivetrain drivetrain) {
		return Shoot(driverController, operatorController, arm, head, () -> drivetrain.getDistanceToSpeaker());
	}
}
