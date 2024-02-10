package frc.robot.shuffleboard;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;

public class LEDTab extends ShuffleboardTabBase {
	private final LED led;
	private final Intake intake;
	private final Shooter shooter;
	private SendableChooser<Boolean> holdingNote;
	private SendableChooser<Boolean> readyToShoot;

	public LEDTab(LED led, Intake intake, Shooter shooter) {
		this.led = led;
		this.intake = intake;
		this.shooter = shooter;

		NetworkTableInstance inst = NetworkTableInstance.getDefault();

		NetworkTable networkTable = inst.getTable("logging/LED");
	}

	@Override
	public void update() {
		// these functions have not been defined
		intake.setIsNoteAcquired(holdingNote.getSelected());
		shooter.setIsReadyToShoot(readyToShoot.getSelected());
	}

	@Override
	public void activateShuffleboard() {
		ShuffleboardTab tab = Shuffleboard.getTab("LED");
		tab.add("Disconnected Animation", new InstantCommand(() -> led.setDisconnectedAnimation()).ignoringDisable(true));
		tab.add("EStop Animation", new InstantCommand(() -> led.setEStopAnimation()).ignoringDisable(true));
		tab.add("Auto Disabled Animation", new InstantCommand(() -> led.setAutoDisabledAnimation()).ignoringDisable(true));
		tab.add("Auto Enabled Animation", new InstantCommand(() -> led.setAutoEnabledAnimation()).ignoringDisable(true));
		tab.add("Teleop Disabled Animation", new InstantCommand(() -> led.setTeleopDisabledAnimation()).ignoringDisable(true));
		tab.add("Teleop Enabled Animation", new InstantCommand(() -> led.setTeleopEnabledAnimation()).ignoringDisable(true));

		holdingNote = new SendableChooser<>();
		holdingNote.addOption("Holding Note", true);
		holdingNote.setDefaultOption("Not Holding Note", false);

		readyToShoot = new SendableChooser<>();
		readyToShoot.addOption("Ready To Shoot", true);
		readyToShoot.setDefaultOption("Not Ready To Shoot", false);

		tab.add(holdingNote);
		tab.add(readyToShoot);
	}

	@Override
	public String getNetworkTable() {
		return "LED";
	}

}
