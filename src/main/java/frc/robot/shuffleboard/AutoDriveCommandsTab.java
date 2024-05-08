package frc.robot.shuffleboard;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.FieldHelpers.TagLocation;

public class AutoDriveCommandsTab extends ShuffleboardTabBase {
	
	private final Drivetrain drivetrain;
	private final String TITLE = "AutoDriveCommands";
	private final Field2d field = new Field2d();
	private boolean addedCommands = false;
	private DoubleArrayPublisher currentPosePub;
	ShuffleboardTab tab = Shuffleboard.getTab(TITLE);
	
	
	public AutoDriveCommandsTab(Drivetrain drivetrain) {
		this.drivetrain = drivetrain;
		
		NetworkTableInstance inst = NetworkTableInstance.getDefault();		
		NetworkTable networkTable = inst.getTable("Shuffleboard/" + TITLE);

		currentPosePub = networkTable.getDoubleArrayTopic("Current Pose").publish();
		
	}
	
	@Override
	public void update() {
		field.setRobotPose(drivetrain.getPose());
		double[] pose = {drivetrain.getPose().getX(), drivetrain.getPose().getY(), drivetrain.getPose().getRotation().getDegrees()};
		currentPosePub.set(pose);
		
		if (AutoConstants.AUTOS_LOADED && !addedCommands){
			tab.add("Drive To Amp", drivetrain.DriveToLocation(TagLocation.AMP)).withPosition(0, 0);
			tab.add("Drive To Source Close", drivetrain.DriveToLocation(TagLocation.SOURCE_CLOSE)).withPosition(0, 1);
			tab.add("Drive To Source Far", drivetrain.DriveToLocation(TagLocation.SOURCE_FAR)).withPosition(0, 2);
			tab.add("Drive To Speaker", drivetrain.DriveToLocation(TagLocation.SPEAKER)).withPosition(0, 3);	
			addedCommands = true;
		}
	}
	
	@Override
	public void activateShuffleboard() {

		double[] pose = {drivetrain.getPose().getX(), drivetrain.getPose().getY(), drivetrain.getPose().getRotation().getDegrees()};

		tab.add("Reset Pose to Speaker", drivetrain.ResetPose(() ->TagLocation.SPEAKER)).withPosition(2, 0);
		tab.add("Reset Pose to Amp", drivetrain.ResetPose(() ->TagLocation.AMP)).withPosition(2, 1);
		tab.add("Reset Pose to Source Close", drivetrain.ResetPose(() ->TagLocation.SOURCE_CLOSE)).withPosition(2, 2);

		tab.add("Field", field)
		.withPosition(5, 0)
		.withSize(5,3);
		tab.add("Current Pose", pose ).withPosition(5, 4).withSize(2, 1);

	}
	
	@Override
	public String getNetworkTable() {
		return TITLE;
	}
}
