package frc.robot.shuffleboard;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.Drivetrain;

public class SwerveTab extends ShuffleboardTabBase {
	private final Drivetrain swerveDrive;
	 private final DoublePublisher odometryYPub;
	 private final DoublePublisher odometryXPub;
	 private final DoublePublisher odometryAnglePub;
	private int number = 0;
	private final IntegerPublisher numPublisher;

	public SwerveTab(Drivetrain swerveDrive) {
		this.swerveDrive = swerveDrive;

		NetworkTableInstance inst = NetworkTableInstance.getDefault();

		NetworkTable networkTable = inst.getTable("logging/swerveDrive");

		 odometryXPub = networkTable.getDoubleTopic("X Odometry").publish();

		 odometryYPub = networkTable.getDoubleTopic("Y Odometry").publish();

		 odometryAnglePub = networkTable.getDoubleTopic("Angle Odometry").publish();

		numPublisher = networkTable.getIntegerTopic("number").publish();

	}

	@Override
	public void update() {
		// these functions have not been defined
		 odometryAnglePub.set(swerveDrive.getPose().getRotation().getDegrees());
		 odometryXPub.set(swerveDrive.getPose().getX());
		 odometryYPub.set(swerveDrive.getPose().getY());
		number += 1;
		numPublisher.set(number);
	}

	@Override
	public void activateShuffleboard() {

		// shuffleboardTab.add(this.swerveDrive);
	}

	@Override
	public String getNetworkTable() {
		return "swerveDrive";
	}

}
