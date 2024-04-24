package frc.robot.shuffleboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LED;

public class LEDTab extends ShuffleboardTabBase {
	private final LED led;

	public LEDTab(LED led) {
		this.led = led;

		NetworkTableInstance inst = NetworkTableInstance.getDefault();

		NetworkTable networkTable = inst.getTable("Shuffleboard/LED");
	}

	@Override
	public void update() {
		// these functions have not been defined
	}

	

}
