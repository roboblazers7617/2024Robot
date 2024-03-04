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
import frc.robot.subsystems.Head;
import frc.robot.util.TunableNumber;

public class HeadTab extends ShuffleboardTabBase {
	private final Head head;
	
	private final BooleanPublisher noteWithinHeadPublisher;
	private final BooleanPublisher noteAlignedPublisher;
	// private final BooleanPublisher noteInShooterPublisher;
	private final BooleanPublisher noteAcquiredPublisher;
	
	private final DoublePublisher shooterBottomSpeedPublisher;
	private final DoublePublisher shooterTopSpeedPublisher;
	private final DoublePublisher shooterSetPointPublisher;
	
	private final BooleanPublisher readyToShootPublisher;
	
	private TunableNumber shootingPosition;
	private final DoublePublisher shooterSpeedAtPositionPublisher;
	private TunableNumber shootingPositionSpeedTuning;
	
	private final DoublePublisher intakeBottomEncoderPublisher;
	private final DoublePublisher intakeTopEncoderPublisher;
	private final DoublePublisher shooterBottomEncoderPublisher;
	private final DoublePublisher shooterTopEncoderPublisher;
	
	public HeadTab(Head head) {
		this.head = head;
		
		NetworkTableInstance inst = NetworkTableInstance.getDefault();
		
		NetworkTable networkTable = inst.getTable("logging/Head");
		
		NetworkTable intakeNetworkTable = inst.getTable("logging/Head/Intake");
		noteWithinHeadPublisher = intakeNetworkTable.getBooleanTopic("Note Within Head").publish();
		noteAlignedPublisher = intakeNetworkTable.getBooleanTopic("Note Aligned").publish();
		// noteInShooterPublisher = intakeNetworkTable.getBooleanTopic("Note in Shooter").publish();
		noteAcquiredPublisher = intakeNetworkTable.getBooleanTopic("Note Acquired").publish();
		
		NetworkTable shooterNetworkTable = inst.getTable("logging/Head/Shooter");
		shooterBottomSpeedPublisher = shooterNetworkTable.getDoubleTopic("Bottom Speed").publish();
		shooterTopSpeedPublisher = shooterNetworkTable.getDoubleTopic("Top Speed").publish();
		shooterSetPointPublisher = shooterNetworkTable.getDoubleTopic("Setpoint").publish();
		
		readyToShootPublisher = shooterNetworkTable.getBooleanTopic("Ready to Shoot").publish();
		
		shooterSpeedAtPositionPublisher = shooterNetworkTable.getDoubleTopic("Speed at Position").publish();
		
		shootingPosition = new TunableNumber("Head/Shooter", "Position (tuning)", 0);
		shootingPositionSpeedTuning = new TunableNumber("Head/Shooter", "Speed (tuning)", 0);
		
		intakeBottomEncoderPublisher = networkTable.getDoubleTopic("Intake Encoder Bottom").publish();
		intakeTopEncoderPublisher = networkTable.getDoubleTopic("Intake Encoder Top").publish();
		shooterBottomEncoderPublisher = networkTable.getDoubleTopic("Shooter Encoder Bottom").publish();
		shooterTopEncoderPublisher = networkTable.getDoubleTopic("Shooter Encoder Top").publish();
	}
	
	@Override
	public void update() {
		noteWithinHeadPublisher.set(head.isNoteWithinHead());
		noteAlignedPublisher.set(head.isNoteAligned());
		// noteInShooterPublisher.set(head.isNoteInShooter());
		noteAcquiredPublisher.set(head.isNoteAcquired());
		
		shooterBottomSpeedPublisher.set(head.getShooterBottomSpeed());
		shooterTopSpeedPublisher.set(head.getShooterTopSpeed());
		shooterSetPointPublisher.set(head.getShooterSetPoint());
		
		readyToShootPublisher.set(head.isReadyToShoot());
		
		shooterSpeedAtPositionPublisher.set(head.getShooterSpeedAtPosition(shootingPosition.get()));
		
		intakeBottomEncoderPublisher.set(head.getIntakeEncoderBottom());
		intakeTopEncoderPublisher.set(head.getIntakeEncoderTop());
		shooterBottomEncoderPublisher.set(head.getShooterEncoderBottom());
		shooterTopEncoderPublisher.set(head.getShooterEncoderTop());
	}
	
	@Override
	public void activateShuffleboard() {
		ShuffleboardTab tab = Shuffleboard.getTab("Head");
		// Intake
		ShuffleboardLayout intakeLayout = tab.getLayout("Intake", BuiltInLayouts.kGrid).withSize(5, 2).withPosition(0, 0);
		intakeLayout.add("Note Within Head", false).withPosition(0, 0);
		intakeLayout.add("Note Aligned", false).withPosition(0, 1);
		intakeLayout.add("Note Acquired", false).withPosition(0, 2);
		
		intakeLayout.add("Intake from Ground", head.IntakePiece(false)).withPosition(1, 0);
		intakeLayout.add("Intake from Source", head.IntakePiece(true)).withPosition(2, 0);
		intakeLayout.add("Manual Intake from Ground", head.StartIntake(false)).withPosition(1, 1);
		intakeLayout.add("Manual Intake from Source", head.StartIntake(true)).withPosition(2, 1);
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
		
		shooterLayout.add("Spin Up", head.SpinUpShooter(() -> shootingPosition.get())).withPosition(2, 0);
		shooterLayout.add("Spin Down", head.SpinDownShooter()).withPosition(2, 1);
		shooterLayout.add("Shoot", head.Shoot()).withPosition(2, 2);
		
		shooterLayout.add("Position (tuning)", 0.0).withPosition(3, 0);
		shooterLayout.add("Speed at Position", 0.0).withPosition(3, 1);
		
		shooterLayout.add("Speed (tuning)", 0.0).withPosition(4, 0);
		shooterLayout.add("Write Speed (tuning)", new InstantCommand(() -> head.setShooterSpeedAtPosition(shootingPosition.get(), shootingPositionSpeedTuning.get()))).withPosition(4, 1);
	}
	
	@Override
	public String getNetworkTable() {
		return "Head";
	}
}
