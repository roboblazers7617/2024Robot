package frc.robot.shuffleboard;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
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
		
		NetworkTable intakeNetworkTable = inst.getTable("Shuffleboard/Head");
		noteWithinSensorPublisher = intakeNetworkTable.getBooleanTopic("Note Within Sensor").publish();
		noteAcquiredPublisher = intakeNetworkTable.getBooleanTopic("Note Acquired").publish();
		
		NetworkTable shooterNetworkTable = inst.getTable("Shuffleboard/Head");
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
		tab.add("Note Within Sensor", false).withPosition(0, 2).withSize(2,1);
		tab.add("Ready to Shoot", false).withPosition(0, 3).withSize(2,1);
		// Shooter
		tab.add("Top Speed", 0.0).withPosition(2, 0)
				.withWidget(BuiltInWidgets.kGraph).withSize(3, 2);
		tab.add("Bottom Speed", 0.0).withPosition(2, 2)
				.withWidget(BuiltInWidgets.kGraph).withSize(3, 2);
		tab.add("Setpoint", 0.0).withPosition(1, 0);
		// tab.add("Feeder Speed", 0.0).withPosition(0, 0);
		
		tab.add("Intake", head.IntakePiece()).withPosition(5, 0);
		tab.add("Manual Intake", head.StartIntake()).withPosition(5, 1);
		tab.add("Outake", head.OutakePiece()).withPosition(5, 2);
		tab.add("Manual Outake", head.StartOutake()).withPosition(5, 3);
		
		tab.add("Stop Intake", head.StopIntake()).withPosition(5, 4);
		tab.add("Spin Up 875", head.SpinUpShooter(875)).withPosition(7, 0);
		tab.add("Spin Up 3250", head.SpinUpShooter(3250)).withPosition(7, 1);
		tab.add("Spin Up 4150", head.SpinUpShooter(4150)).withPosition(7, 2);
		tab.add("Spin Up 4700", head.SpinUpShooter(4700)).withPosition(7, 3);
		tab.add("Spin Up 3500", head.SpinUpShooter(3500)).withPosition(7, 4);
		tab.add("Spin Up 4000", head.SpinUpShooter(4000)).withPosition(7, 5);

		tab.add("Spin Down", head.SpinDownShooter()).withPosition(9, 0);
		//shooterLayout.add("Shoot (speaker)", head.Shoot(ShootingConstants.ShootingPosition.SUBWOOFER)).withPosition(2, 2);
	}
	
	@Override
	public String getNetworkTable() {
		return "Head";
	}
}
