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
		
		NetworkTable networkTable = inst.getTable("logging/Head");
		
		NetworkTable intakeNetworkTable = inst.getTable("logging/Head/Intake");
		noteWithinSensorPublisher = intakeNetworkTable.getBooleanTopic("Note Within Sensor").publish();
		noteAcquiredPublisher = intakeNetworkTable.getBooleanTopic("Note Acquired").publish();
		
		NetworkTable shooterNetworkTable = inst.getTable("logging/Head/Shooter");
		shooterBottomSpeedPublisher = shooterNetworkTable.getDoubleTopic("Bottom Speed").publish();
		shooterTopSpeedPublisher = shooterNetworkTable.getDoubleTopic("Top Speed").publish();
		shooterSetPointPublisher = shooterNetworkTable.getDoubleTopic("Setpoint").publish();
		
		readyToShootPublisher = shooterNetworkTable.getBooleanTopic("Ready to Shoot").publish();
	}
	
	@Override
	public void update() {
		noteWithinSensorPublisher.set(head.isNoteWithinSensor());
		// noteInShooterPublisher.set(head.isNoteInShooter());
		
		shooterBottomSpeedPublisher.set(head.getShooterBottomSpeed());
		shooterTopSpeedPublisher.set(head.getShooterTopSpeed());
		shooterSetPointPublisher.set(head.getShooterSetPoint());
		
		readyToShootPublisher.set(head.isReadyToShoot());
	}
	
	@Override
	public void activateShuffleboard() {
		ShuffleboardTab tab = Shuffleboard.getTab("Head");
		// Intake
		ShuffleboardLayout intakeLayout = tab.getLayout("Intake", BuiltInLayouts.kGrid).withSize(5, 2).withPosition(0, 0);
		intakeLayout.add("Note Within Sensor", false).withPosition(0, 0);
		
		intakeLayout.add("Intake", head.IntakePiece()).withPosition(1, 0);
		intakeLayout.add("Manual Intake", head.StartIntake()).withPosition(2, 0);
		intakeLayout.add("Outake", head.OutakePiece()).withPosition(3, 0);
		intakeLayout.add("Manual Outake", head.StartOutake()).withPosition(3, 1);
		
		intakeLayout.add("Stop Intake", head.StopIntake()).withPosition(4, 0);
		
		intakeLayout.add("Feeder Speed", 0.0).withPosition(1, 2);
		
		// Shooter
		ShuffleboardLayout shooterLayout = tab.getLayout("Shooter", BuiltInLayouts.kGrid).withSize(5, 2).withPosition(0, 2);
		shooterLayout.add("Top Speed", 0.0).withPosition(0, 0);
		shooterLayout.add("Bottom Speed", 0.0).withPosition(0, 1);
		shooterLayout.add("Setpoint", 0.0).withPosition(0, 2);
		
		shooterLayout.add("Ready to Shoot", false).withPosition(1, 0);
		
		shooterLayout.add("Spin Up (speaker)", head.SpinUpShooter(ShootingConstants.ShootingPosition.SUBWOOFER)).withPosition(2, 0);
		shooterLayout.add("Spin Down", head.SpinDownShooter()).withPosition(2, 1);
		// shooterLayout.add("Shoot (speaker)", head.Shoot(ShootingConstants.ShootingPosition.SUBWOOFER)).withPosition(2, 2);
	}
	
	@Override
	public String getNetworkTable() {
		return "Head";
	}
}
