package frc.robot.shuffleboard;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LED;

public class LEDTab extends ShuffleboardTabBase {
	private final LED led;
	// private final DoublePublisher odometryYPub;
	// private final DoublePublisher odometryXPub;
	// private final DoublePublisher odometryAnglePub;

	public LEDTab(LED led) {
		this.led = led;

		NetworkTableInstance inst = NetworkTableInstance.getDefault();

		NetworkTable networkTable = inst.getTable("logging/LED");
	}

	@Override
	public void update() {
		// these functions have not been defined
	}

	@Override
	public void activateShuffleboard() {
		ShuffleboardTab tab = Shuffleboard.getTab("LED");
		tab.add("Disconnected Animation", new InstantCommand(() -> led.setDisconnectedAnimation()));
		tab.add("EStop Animation", new InstantCommand(() -> led.setEStopAnimation()));
		tab.add("Idle Animation", new InstantCommand(() -> led.setIdleAnimation()));
		tab.add("Auto Color", new InstantCommand(() -> led.setAutoColor()));
		tab.add("Teleop Color", new InstantCommand(() -> led.setTeleopColor()));
	}

	@Override
	public String getNetworkTable() {
		return "LED";
	}

}
