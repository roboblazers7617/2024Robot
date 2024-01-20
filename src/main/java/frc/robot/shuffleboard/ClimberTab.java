package frc.robot.shuffleboard;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.Climber;

public class ClimberTab extends ShuffleboardTabBase {
	private final Climber climber;

	private final DoublePublisher rightEncoderPub;
	private final DoublePublisher leftEncoderPub;
	private final DoublePublisher rightAbsoluteEncoderPub;
	private final DoublePublisher leftAbsoluteEncoderPub;
	// private final DoublePublisher odometryYPub;
	// private final DoublePublisher odometryXPub;
	// private final DoublePublisher odometryAnglePub;

	public ClimberTab(Climber climber) {
		this.climber = climber;

		NetworkTableInstance inst = NetworkTableInstance.getDefault();

		NetworkTable networkTable = inst.getTable("logging/climber");

		rightEncoderPub = networkTable.getDoubleTopic("Right Encoder").publish();
		leftEncoderPub = networkTable.getDoubleTopic("Left Encoder").publish();
		rightAbsoluteEncoderPub = networkTable.getDoubleTopic("Right Absolute Encoder").publish();
		leftAbsoluteEncoderPub = networkTable.getDoubleTopic("Left Absolute Encoder").publish();

	}

	@Override
	public void update() {
		rightEncoderPub.set(climber.getRightEncoderPosition());
		leftEncoderPub.set(climber.getLeftEncoderPosition());
		rightAbsoluteEncoderPub.set(climber.getRightAbsoluteEncoderPosition());
		leftAbsoluteEncoderPub.set(climber.getLeftAbsoluteEncoderPosition());
	}

	@Override
	public void activateShuffleboard() {

	}

	@Override
	public String getNetworkTable() {
		return "climber";
	}

}
