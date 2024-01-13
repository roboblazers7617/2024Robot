// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shuffleboard;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShuffleboardInfo extends SubsystemBase {
	ArrayList<ShuffleboardTabBase> tabs;

	private static ShuffleboardInfo instance;
	private String[] copyTables;

	public static ShuffleboardInfo getInstance() {
		if (instance == null) {
			instance = new ShuffleboardInfo();
		}
		return instance;
	}

	private ShuffleboardInfo() {

	}

	public void addTabs(ArrayList<ShuffleboardTabBase> tabs) {
		this.tabs = tabs;

		tabs.get(0).activateShuffleboard();
		copyTables = new String[tabs.size() - 1]; // subtract one because driverstation doesn't need to be copied
	}

	@Override
	public void periodic() {

		// This method will be called once per scheduler run
		// it will update every tab
		if (tabs != null) {
			for (int i = 0; i < tabs.size(); i++) {
				tabs.get(i).update();
			}
		}

		// copy over values from the logging network table to shuffleboard
		NetworkTableInstance inst = NetworkTableInstance.getDefault();
		// copyTables = new String[tabs.size()];
		copyTables[0] = "swerveDrive";
		for (String tab : copyTables) {
			if (tab != null) {

				for (String key : inst.getTable("logging/" + tab).getKeys()) {
					NetworkTableEntry sourceEntry = inst.getTable("logging/" + tab).getEntry(key);
					NetworkTableEntry destinationEntry = inst.getTable("Shuffleboard/" + tab).getEntry(key);
					destinationEntry.setValue(sourceEntry.getValue());
				}
			}
		}

	}
}
