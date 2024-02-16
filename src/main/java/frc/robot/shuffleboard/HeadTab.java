package frc.robot.shuffleboard;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Head;
import frc.robot.Constants.ShooterConstants;

public class HeadTab extends ShuffleboardTabBase {
	private final Head head;

	//TODO: (Max) Now that you have written the code for holding a note and ready to shoot, you should remove these sendable choosers as it should read
	//directly from the subsystem
	private SendableChooser<Boolean> holdingNote;
	private SendableChooser<Boolean> readyToShoot;

	private final BooleanPublisher noteAcquiredPublisher;
	private final BooleanPublisher noteAlignedPublisher;
	private final BooleanPublisher noteInShooterPublisher;
	private final DoublePublisher shooterSpeedPublisher;
	private final DoublePublisher shooterSetPointPublisher;
	private final BooleanPublisher shooterAtSpeedPublisher;
	private final BooleanPublisher readyToShootPublisher;
	
	public HeadTab(Head head) {
		this.head = head;

		NetworkTableInstance inst = NetworkTableInstance.getDefault();

		NetworkTable networkTable = inst.getTable("logging/Head");

		noteAcquiredPublisher = networkTable.getBooleanTopic("Note Acquired").publish();
		noteAlignedPublisher = networkTable.getBooleanTopic("Note Aligned").publish();
		noteInShooterPublisher = networkTable.getBooleanTopic("Note in Shooter").publish();
		shooterSpeedPublisher = networkTable.getDoubleTopic("Shooter Speed").publish();
		shooterSetPointPublisher = networkTable.getDoubleTopic("Shooter Setpoint").publish();
		shooterAtSpeedPublisher = networkTable.getBooleanTopic("Shooter At Speed").publish();
		readyToShootPublisher = networkTable.getBooleanTopic("Ready To Shoot").publish();
	}

	@Override
	public void update() {
		//TODO: (Max) Change these to read the actual values in the subsystem
		// these functions have not been defined
		//head.setIsNoteAcquired(holdingNote.getSelected());
		head.setIsReadyToShoot(readyToShoot.getSelected());

		noteAcquiredPublisher.set(head.isNoteAcquired());
		noteAlignedPublisher.set(head.isNoteAligned());
		noteInShooterPublisher.set(head.isNoteInShooter());
		shooterSpeedPublisher.set(head.getShooterSpeed());
		shooterSetPointPublisher.set(head.getShooterSetPoint());
		shooterAtSpeedPublisher.set(head.isShooterAtSpeed());
		readyToShootPublisher.set(head.isReadyToShoot());
	}

	@Override
	public void activateShuffleboard() {
		ShuffleboardTab tab = Shuffleboard.getTab("Head");
		tab.add("Intake from Ground", head.intakePiece(false));
		tab.add("Intake from Source", head.intakePiece(true));
		tab.add("Outake", head.outakePiece());

		tab.add("Spin Up (Subwoofer)", head.spinUpShooter(ShooterConstants.ShootingPosition.SUBWOOFER));
		tab.add("Spin Down", head.spinDownShooter());
		tab.add("Shoot", head.shoot());

		//holdingNote = new SendableChooser<>();
		//holdingNote.addOption("Holding Note", true);
		//holdingNote.setDefaultOption("Not Holding Note", false);

		//TODO: (Max) I don't think you need this any longer. correct? as was just for testing
		readyToShoot = new SendableChooser<>();
		readyToShoot.addOption("Ready To Shoot", true);
		readyToShoot.setDefaultOption("Not Ready To Shoot", false);

		//tab.add(holdingNote);
		tab.add(readyToShoot);
	}

	@Override
	public String getNetworkTable() {
		return "Head";
	}
}
