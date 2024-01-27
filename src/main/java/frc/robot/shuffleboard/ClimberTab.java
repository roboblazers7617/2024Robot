package frc.robot.shuffleboard;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Climber;

public class ClimberTab extends ShuffleboardTabBase {
	private final Climber climber;

	private final DoublePublisher rightAbsoluteEncoderPub;
	private final DoublePublisher leftAbsoluteEncoderPub;
	// private final DoublePublisher odometryYPub;
	// private final DoublePublisher odometryXPub;
	// private final DoublePublisher odometryAnglePub;

	public ClimberTab(Climber climber) {
		this.climber = climber;

		NetworkTableInstance inst = NetworkTableInstance.getDefault();

		NetworkTable networkTable = inst.getTable("logging/climber");

		rightAbsoluteEncoderPub = networkTable.getDoubleTopic("Right Absolute Encoder").publish();
		leftAbsoluteEncoderPub = networkTable.getDoubleTopic("Left Absolute Encoder").publish();

	}

	@Override
	public void update() {
		rightAbsoluteEncoderPub.set(climber.getRightAbsoluteEncoderPosition());
		leftAbsoluteEncoderPub.set(climber.getLeftAbsoluteEncoderPosition());
	}

	@Override
	public void activateShuffleboard() {
		ShuffleboardTab tab = Shuffleboard.getTab("climber");
		tab.add("raise climber", new InstantCommand(() -> climber.raiseClimber()));
		tab.add("lower climber", new InstantCommand(() -> climber.lowerClimber()));
	}

	@Override
	public String getNetworkTable() {
		return "climber";
	}

}
