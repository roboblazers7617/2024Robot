// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shuffleboard;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

public class ShuffleboardInfo extends SubsystemBase {
	ArrayList<ShuffleboardTabBase> tabs;

	private static ShuffleboardInfo instance;
	private String[] copyTables;
	private boolean isActivated = false;
	private Alert enabledAlert = new Alert("Enabled batter voltage is below " + Constants.ENABLED_BATTERY_WARNING_VOLTAGE, AlertType.ERROR);
	private Alert disabledAlert = new Alert("Disabled batter voltage is below " + Constants.DISABLED_BATTERY_WARNING_VOLTAGE, AlertType.ERROR);

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

	public void activateTabs() {
		if (isActivated == false) {
			isActivated = true;
			for (int i = 1; i < tabs.size(); i++) {
				if (tabs.get(i) != null) {
					tabs.get(i).activateShuffleboard();
					copyTables[i - 1] = tabs.get(i).getNetworkTable();
				}
			}
		}
	}

	@Override
	public void periodic() {
		// if robot is not connected to the field system, enable shuffleboard
		if (!DriverStation.isFMSAttached()) {
			activateTabs();
		}
		// This method will be called once per scheduler run
		// it will update every tab
		if (tabs != null) {
			for (int i = 0; i < tabs.size(); i++) {
				tabs.get(i).update();
			}
		}

		// copy over values from the logging network table to shuffleboard
		NetworkTableInstance inst = NetworkTableInstance.getDefault();
		for (String tab : copyTables) {
			if (tab != null) {
				for (String key : inst.getTable("logging/" + tab).getKeys()) {
					NetworkTableEntry sourceEntry = inst.getTable("logging/" + tab).getEntry(key);
					NetworkTableEntry destinationEntry = inst.getTable("Shuffleboard/" + tab).getEntry(key);
					destinationEntry.setValue(sourceEntry.getValue());

				}
			}
		}


		// create an Alert if the battery voltage is below BATTERY_WARNING_VOLTAGE
		enabledAlert.set(RobotController.getBatteryVoltage() < Constants.ENABLED_BATTERY_WARNING_VOLTAGE && DriverStation.isEnabled());
		disabledAlert.set(RobotController.getBatteryVoltage() < Constants.DISABLED_BATTERY_WARNING_VOLTAGE && !DriverStation.isEnabled());
		//batter voltage is displayed on the driver station tab

	}
}
