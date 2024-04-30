package frc.robot.shuffleboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.FieldHelpers.TagLocation;

public class TestCommandsTab extends ShuffleboardTabBase {
	
	private final Drivetrain drivetrain;
	private final String TITLE = "testCommands";
	
	
	public TestCommandsTab(Drivetrain drivetrain) {
		this.drivetrain = drivetrain;
		
		NetworkTableInstance inst = NetworkTableInstance.getDefault();		
		NetworkTable networkTable = inst.getTable("logging/" + TITLE);
		
	}
	
	@Override
	public void update() {
	}
	
	@Override
	public void activateShuffleboard() {
		ShuffleboardTab tab = Shuffleboard.getTab(TITLE);
	
		tab.add("Drive To Amp", drivetrain.DriveToLocation(TagLocation.AMP));
		tab.add("Drive To Source Close", drivetrain.DriveToLocation(TagLocation.SOURCE_CLOSE));
		tab.add("Drive To Source Far", drivetrain.DriveToLocation(TagLocation.SOURCE_FAR));
		tab.add("Drive To Speaker", drivetrain.DriveToLocation(TagLocation.SPEAKER));	
		tab.add("Reset Starting Pose to Speaker", drivetrain.ResetPose(TagLocation.SPEAKER));
		tab.add("Reset Starting Pose to Amp", drivetrain.ResetPose(TagLocation.AMP));
		tab.add("Reset Starting Pose to Source Close", drivetrain.ResetPose(TagLocation.SOURCE_CLOSE));

	}
	
	@Override
	public String getNetworkTable() {
		return TITLE;
	}
}
