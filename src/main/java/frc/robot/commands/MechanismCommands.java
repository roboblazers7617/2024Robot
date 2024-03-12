package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
	public static Command ShootAmp(XboxController operatorController, Arm arm, Head head) {
		return new InstantCommand(() -> arm.setArmTarget(ArmConstants.AMP_ANGLE))
				.andThen(new InstantCommand(() -> arm.setElevatorTarget(ElevatorConstants.MAX_HEIGHT)))
				.andThen(head.SpinUpShooterForAmp())
				.andThen(Commands.waitUntil(() -> head.isReadyToShoot()))
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(operatorController, RumbleType.kBothRumble, 0.3, 0.3)));
	}
	
	public static Command ShootSpeaker(XboxController driverController, XboxController operatorController, Arm arm, Head head, Drivetrain drivetrain) {
		double distance = drivetrain.getDistanceToSpeaker();
		return Commands.runOnce(() -> arm.setArmTargetByDistance(distance))
				.andThen(head.ShootAtPosition(distance))
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(driverController, RumbleType.kBothRumble, 0.3, 0.3)))
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(operatorController, RumbleType.kBothRumble, 0.3, 0.3)));
	}
	
	public static Command ShootSpeakerSubwoofer(XboxController driverController, XboxController operatorController, Arm arm, Head head) {
		return Commands.runOnce(() -> arm.setArmTarget(ArmConstants.SPEAKER_SUBWOOFER_ANGLE))
				.andThen(() -> arm.setElevatorTarget(ElevatorConstants.MAX_HEIGHT))
				.andThen(head.SpinUpShooterAtPosition(0))
				.andThen(arm.WaitUntilArmAtTarget())
				.andThen(arm.WaitUntilElevatorAtTarget())
				.andThen(head.ShootAtPosition(0))
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(driverController, RumbleType.kBothRumble, 0.3, 0.3)))
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(operatorController, RumbleType.kBothRumble, 0.3, 0.3)));
	}
	
	public static Command ShootSpeakerPodium(XboxController driverController, XboxController operatorController, Arm arm, Head head) {
		return Commands.runOnce(() -> arm.setArmTargetByDistance(Constants.PODIUM_DISTANCE))
				.andThen(head.ShootAtPosition(Constants.PODIUM_DISTANCE))
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(driverController, RumbleType.kBothRumble, 0.3, 0.3)))
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(operatorController, RumbleType.kBothRumble, 0.3, 0.3)));
	}
	
	public static Command IntakeSource(XboxController driverController, XboxController operatorController, Arm arm, Head head) {
		return Commands.runOnce(() -> arm.setArmTarget(ArmConstants.SOURCE_ANGLE))
				.andThen(() -> arm.setElevatorTarget(ElevatorConstants.MAX_HEIGHT))
				.andThen(head.IntakePiece())
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(driverController, RumbleType.kBothRumble, 0.3, 0.3)))
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(operatorController, RumbleType.kBothRumble, 0.3, 0.3)));
	}
	
	public static Command IntakeGround(XboxController driverController, XboxController operatorController, Arm arm, Head head) {
		return Commands.runOnce(() -> arm.setArmTarget(ArmConstants.MIN_ANGLE))
				.andThen(() -> arm.setElevatorTarget(ElevatorConstants.MAX_HEIGHT))
				.andThen(head.IntakePiece())
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(driverController, RumbleType.kBothRumble, 0.3, 0.3)))
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(operatorController, RumbleType.kBothRumble, 0.3, 0.3)));
	}
}
