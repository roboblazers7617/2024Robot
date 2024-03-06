package frc.robot.commands;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Head;

public class MechanismCommands {
	public Command ShootAmp(Arm arm, Head head, double distance) {
		return new InstantCommand(() -> arm.setArmTarget(ArmConstants.MAX_ANGLE)).andThen(new InstantCommand(() -> arm.setElevatorTarget(ElevatorConstants.MAX_HEIGHT))).andThen(head.ShootInAmp());
	}
	
	public Command ShootSpeaker(Arm arm, Head head, double distance) {
		InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
		map.put(0.0, 0.0);
		return Commands.runOnce(() -> arm.setArmTarget(map.get(distance))).andThen(head.ShootAtPosition(distance));
	}
	
	public Command ShootSpeakerSubwoofer(Arm arm, Head head) {
		InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
		map.put(0.0, 0.0);
		return Commands.runOnce(() -> arm.setArmTarget(map.get(0.0))).andThen(head.ShootAtPosition(0));
	}
	
	public Command ShootSpeakerPodium(Arm arm, Head head) {
		InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
		map.put(0.0, 0.0);
		return Commands.runOnce(() -> arm.setArmTarget(map.get(Constants.PODIUM_DISTANCE))).andThen(head.ShootAtPosition(Constants.PODIUM_DISTANCE));
	}
	
	public Command IntakeSource(Arm arm, Head head) {
		return Commands.runOnce(() -> arm.setArmTarget(ArmConstants.SOURCE_ANGLE)).andThen(head.IntakePiece(true));
	}
	
	public Command IntakeGround(Arm arm, Head head) {
		return Commands.runOnce(() -> arm.setArmTarget(ArmConstants.MIN_ANGLE)).andThen(head.IntakePiece(false));
	}
}
