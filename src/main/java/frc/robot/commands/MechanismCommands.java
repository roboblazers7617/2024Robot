package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Head;

public class MechanismCommands {
	public static Command PrepareShootAmp(XboxController operatorController, Arm arm, Head head) {
		return new InstantCommand(() -> arm.setArmTarget(ArmConstants.AMP_ANGLE))
				.andThen(new InstantCommand(() -> arm.setElevatorTarget(ElevatorConstants.MAX_HEIGHT)))
				.andThen(head.SpinUpShooterForAmp())
				.andThen(Commands.waitUntil(() -> head.isReadyToShoot()))
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(operatorController, RumbleType.kBothRumble, 0.3, 0.3)));
	}
	
	public static Command ShootAmp(XboxController driverController, XboxController operatorController, Arm arm, Head head) {
		return arm.WaitUntilArmAtTarget()
				.andThen(arm.WaitUntilElevatorAtTarget())
				.andThen(head.ShootInAmp())
				.andThen(arm.Stow())
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(driverController, RumbleType.kBothRumble, 0.3, 0.3)))
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(operatorController, RumbleType.kBothRumble, 0.3, 0.3)));
	}
	
	/**
	 * will finish after piece has been shot
	 * 
	 * @param driverController
	 * @param operatorController
	 * @param arm
	 * @param head
	 * @param drivetrain
	 *            subsystem
	 * @return
	 */
	public static Command ShootSpeaker(XboxController driverController, XboxController operatorController, Arm arm, Head head, Drivetrain drivetrain) {
		return ShootSpeaker(driverController, operatorController, arm, head, () -> drivetrain.getDistanceToSpeaker());
	}
	
	/**
	 * will wait until finished shooting to finish command
	 * 
	 * @param driverController
	 * @param operatorController
	 * @param arm
	 * @param head
	 * @param distance
	 *            in meters
	 * @return
	 */
	public static Command ShootSpeaker(XboxController driverController, XboxController operatorController, Arm arm, Head head, Supplier<Double> distance) {
		return Commands.runOnce(() -> arm.setArmTargetByDistance(distance.get()))
				.andThen(() -> arm.setElevatorTarget(ElevatorConstants.MAX_HEIGHT))
				.andThen(arm.WaitUntilElevatorAtTarget())
				.andThen(arm.WaitUntilArmAtTarget())
				.andThen(head.ShootInSpeaker())
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(driverController, RumbleType.kBothRumble, 0.3, 0.3)))
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(operatorController, RumbleType.kBothRumble, 0.3, 0.3)));
	}
	
	/**
	 * will wait until finished shooting to finish command
	 * 
	 * @param driverController
	 * @param operatorController
	 * @param arm
	 * @param head
	 * @param distance
	 *            in meters
	 * @return
	 */
	public static Command ShootSpeaker(XboxController driverController, XboxController operatorController, Arm arm, Head head, double distance) {
		return Commands.runOnce(() -> arm.setArmTargetByDistance(distance))
				.andThen(() -> arm.setElevatorTarget(ElevatorConstants.MAX_HEIGHT))
				.andThen(arm.WaitUntilElevatorAtTarget())
				.andThen(arm.WaitUntilArmAtTarget())
				.andThen(head.ShootInSpeaker())
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(driverController, RumbleType.kBothRumble, 0.3, 0.3)))
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(operatorController, RumbleType.kBothRumble, 0.3, 0.3)));
	}
	
	/**
	 * will finish after piece has been shot
	 * 
	 * @param driverController
	 * @param operatorController
	 * @param arm
	 *            subsystem
	 * @param head
	 *            subsystem
	 * @return Command
	 */
	public static Command ShootSpeakerSubwoofer(XboxController driverController, XboxController operatorController, Arm arm, Head head) {
		return Commands.runOnce(() -> arm.setArmTargetByDistance(1.27))
				.andThen(() -> arm.setElevatorTarget(ElevatorConstants.MAX_HEIGHT))
				.andThen(head.SpinUpShooterForSpeaker())
				.andThen(arm.WaitUntilArmAtTarget())
				.andThen(arm.WaitUntilElevatorAtTarget())
				.andThen(head.ShootInSpeaker())
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(driverController, RumbleType.kBothRumble, 0.3, 0.3)))
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(operatorController, RumbleType.kBothRumble, 0.3, 0.3)));
	}
	
	public static Command PrepareShootSpeakerSubwoofer(XboxController driverController, XboxController operatorController, Arm arm, Head head) {
		return Commands.runOnce(() -> arm.setArmTargetByDistance(1.27))
				.andThen(() -> arm.setElevatorTarget(ElevatorConstants.MAX_HEIGHT))
				.andThen(head.SpinUpShooterForSpeaker());
	}
	
	/**
	 * will finish after piece has been shot
	 * 
	 * @param driverController
	 * @param operatorController
	 * @param arm
	 *            subsystem
	 * @param head
	 *            subsystem
	 * @return Command
	 */
	public static Command ShootSpeakerPodium(XboxController driverController, XboxController operatorController, Arm arm, Head head) {
		return Commands.runOnce(() -> arm.setArmTargetByDistance(Constants.PODIUM_DISTANCE)) // todo where should elevator be?
				.andThen(head.ShootInSpeaker())
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(driverController, RumbleType.kBothRumble, 0.3, 0.3)))
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(operatorController, RumbleType.kBothRumble, 0.3, 0.3)));
	}
	
	/**
	 * will finish after piece has been intaken
	 * 
	 * @param driverController
	 * @param operatorController
	 * @param arm
	 *            subsystem
	 * @param head
	 *            subsystem
	 * @return Command
	 */
	public static Command IntakeSource(XboxController driverController, XboxController operatorController, Arm arm, Head head) {
		return Commands.runOnce(() -> arm.setArmTarget(ArmConstants.SOURCE_ANGLE))
				.andThen(() -> arm.setElevatorTarget(ElevatorConstants.MAX_HEIGHT))
				.andThen(head.IntakePiece())
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(driverController, RumbleType.kBothRumble, 0.3, 0.3)))
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(operatorController, RumbleType.kBothRumble, 0.3, 0.3)));
	}
	
	public static Command IntakeGround(XboxController driverController, XboxController operatorController, Arm arm, Head head) {
		return Commands.runOnce(() -> arm.setArmTarget(ArmConstants.FLOOR_PICKUP))
				.andThen(() -> arm.setElevatorTarget(ElevatorConstants.MAX_HEIGHT))
				.andThen(head.IntakePiece())
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(driverController, RumbleType.kBothRumble, 0.3, 0.3)))
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(operatorController, RumbleType.kBothRumble, 0.3, 0.3)));
	}
}
