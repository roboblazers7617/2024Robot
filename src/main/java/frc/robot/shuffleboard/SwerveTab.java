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
	
	public SwerveTab(Drivetrain swerveDrive) {
		this.swerveDrive = swerveDrive;

		NetworkTableInstance inst = NetworkTableInstance.getDefault();

		NetworkTable networkTable = inst.getTable("logging/swerveDrive");

		odometryXPub = networkTable.getDoubleTopic("X Odometry").publish();

		odometryYPub = networkTable.getDoubleTopic("Y Odometry").publish();

		odometryAnglePub = networkTable.getDoubleTopic("Angle Odometry").publish();


	}

	@Override
	public void update() {
		odometryAnglePub.set(swerveDrive.getPose().getRotation().getDegrees());
		odometryXPub.set(swerveDrive.getPose().getX());
		odometryYPub.set(swerveDrive.getPose().getY());
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
