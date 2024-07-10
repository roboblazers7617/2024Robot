package frc.robot.shuffleboard;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ShootingConstants;
import frc.robot.Constants.ShootingConstants.ShootingPosition;
import frc.robot.subsystems.Head;

public class HeadTab extends ShuffleboardTabBase {
	private final Head head;
	
	private final BooleanPublisher noteWithinSensorPublisher;
	// private final BooleanPublisher noteInShooterPublisher;
	private final BooleanPublisher noteAcquiredPublisher;
	
	private final DoublePublisher shooterBottomSpeedPublisher;
	private final DoublePublisher shooterTopSpeedPublisher;
	private final DoublePublisher shooterSetPointPublisher;
	
	private final BooleanPublisher readyToShootPublisher;
	
	public HeadTab(Head head) {
		this.head = head;
		
		NetworkTableInstance inst = NetworkTableInstance.getDefault();
		
		NetworkTable networkTable = inst.getTable("Shuffleboard/Head");
		
		noteWithinSensorPublisher = networkTable.getBooleanTopic("Note Within Sensor").publish();
		noteAcquiredPublisher = networkTable.getBooleanTopic("Note Acquired").publish();
		
		shooterBottomSpeedPublisher = networkTable.getDoubleTopic("Bottom Speed").publish();
		shooterTopSpeedPublisher = networkTable.getDoubleTopic("Top Speed").publish();
		shooterSetPointPublisher = networkTable.getDoubleTopic("Setpoint").publish();
		
		readyToShootPublisher = networkTable.getBooleanTopic("Ready to Shoot").publish();
	}
	
	@Override
	public void update() {
		noteWithinSensorPublisher.set(head.isNoteWithinSensor());	
		shooterBottomSpeedPublisher.set(head.getShooterBottomSpeed());
		shooterTopSpeedPublisher.set(head.getShooterTopSpeed());
		shooterSetPointPublisher.set(head.getShooterSetPoint());
		
		readyToShootPublisher.set(head.isReadyToShoot());
	}
	
	@Override
	public void activateShuffleboard() {
		ShuffleboardTab tab = Shuffleboard.getTab("Head");
		// Intake
		tab.add("Note Within Sensor", false).withPosition(0, 0);
		
		tab.add("Intake", head.IntakePiece()).withPosition(0, 1);
		tab.add("Manual Intake", head.StartIntake()).withPosition(0, 2);
		tab.add("Outake", head.OutakePiece()).withPosition(2, 1);
		tab.add("Manual Outake", head.StartOutake()).withPosition(2, 2);
		
		tab.add("Stop Intake", head.StopIntake()).withPosition(2, 0);
		
		tab.add("Feeder Speed", 0.0).withPosition(5, 2);
		
		// Shooter
		tab.add("Top Speed", 0.0).withPosition(4, 0);
		tab.add("Bottom Speed", 0.0).withPosition(4, 2);
		tab.add("Setpoint", 0.0).withPosition(4, 1);
		
		tab.add("Ready to Shoot", false).withPosition(1, 0);
		
		tab.add("Spin Up (speaker)", head.SpinUpShooter(ShootingConstants.ShootingPosition.SUBWOOFER)).withPosition(5, 0);
		tab.add("Spin Down", head.SpinDownShooter()).withPosition(5, 1);
		//shooterLayout.add("Shoot (speaker)", head.Shoot(ShootingConstants.ShootingPosition.SUBWOOFER)).withPosition(2, 2);
	}
	
	@Override
	public String getNetworkTable() {
		return "Head";
	}
}
