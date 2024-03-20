package frc.robot.shuffleboard;

import edu.wpi.first.networktables.DoubleArrayPublisher;
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
	private final DoubleArrayPublisher posePublisher;
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
		
		posePublisher = networkTable.getDoubleArrayTopic("Pose").publish();
	}
	
	@Override
	public void update() {
		// these functions have not been defined
		odometryAnglePub.set(swerveDrive.getPose().getRotation().getDegrees());
		odometryXPub.set(swerveDrive.getPose().getX());
		odometryYPub.set(swerveDrive.getPose().getY());
		number += 1;
		numPublisher.set(number);
		double[] pose = new double[3];
		pose[0] = swerveDrive.getPose().getX();
		pose[1] = swerveDrive.getPose().getY();
		pose[2] = swerveDrive.getPose().getRotation().getDegrees();
		posePublisher.set(pose);
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
