package frc.robot.shuffleboard;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.Drivetrain;

public class SwerveTab extends ShuffleboardTabBase {
	private final Drivetrain swerveDrive;
	private final DoublePublisher odometryYPub;
	private final DoublePublisher odometryXPub;
	private final DoublePublisher odometryAnglePub;
	private final DoubleArrayPublisher posePublisher;
	
	public SwerveTab(Drivetrain swerveDrive) {
		this.swerveDrive = swerveDrive;
		
		NetworkTableInstance inst = NetworkTableInstance.getDefault();
		
		NetworkTable networkTable = inst.getTable("logging/swerveDrive");
		
		odometryXPub = networkTable.getDoubleTopic("X Odometry").publish();
		
		odometryYPub = networkTable.getDoubleTopic("Y Odometry").publish();
		
		odometryAnglePub = networkTable.getDoubleTopic("Angle Odometry").publish();
		
		posePublisher = networkTable.getDoubleArrayTopic("Pose").publish();
	}
	
	@Override
	public void update() {
		odometryAnglePub.set(swerveDrive.getPose().getRotation().getDegrees());
		odometryXPub.set(swerveDrive.getPose().getX());
		odometryYPub.set(swerveDrive.getPose().getY());
		double[] pose = new double[3];
		pose[0] = swerveDrive.getPose().getX();
		pose[1] = swerveDrive.getPose().getY();
		pose[2] = swerveDrive.getPose().getRotation().getDegrees();
		posePublisher.set(pose);
	}
	
	@Override
	public void activateShuffleboard() {
		ShuffleboardTab tab = Shuffleboard.getTab("swerveDrive");
		tab.add("X Odometry", 0.0).withPosition(0, 0);
		tab.add("Y Odometry", 0.0).withPosition(1, 0);
		tab.add("Angle Odometry", 0.0).withPosition(0, 1);
		tab.add("pose", new double[3]).withPosition(0, 3);


	}
	
	@Override
	public String getNetworkTable() {
		return "swerveDrive";
	}
}
