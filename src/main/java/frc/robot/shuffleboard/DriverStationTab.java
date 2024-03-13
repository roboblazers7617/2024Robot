// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shuffleboard;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

/** Add your docs here. */
public class DriverStationTab extends ShuffleboardTabBase {
	private int number = 0;
	private final IntegerPublisher numPublisher;
	private final DoublePublisher voltagePublisher;
	private final BooleanPublisher isDriverStationConnected;
	private final BooleanPublisher isBrownedOut;
	private Alert alert = new Alert("something is wrong", AlertType.ERROR);
	private SendableChooser<Command> autoChooser = new SendableChooser<>();
	
	
	public DriverStationTab(SendableChooser<Command> autoChooser) {
		this.autoChooser = autoChooser;

		NetworkTableInstance inst = NetworkTableInstance.getDefault();
		
		NetworkTable networkTable = inst.getTable("Shuffleboard/Driver Station");
		
		numPublisher = networkTable.getIntegerTopic("number").publish();
		voltagePublisher = networkTable.getDoubleTopic("Battery Voltage").publish();
		isDriverStationConnected = networkTable.getBooleanTopic("Is Driver Station Connected").publish();
		isBrownedOut = networkTable.getBooleanTopic("Is Browned Out").publish();
	}
	
	@Override
	public void update() {
		number += 1;
		numPublisher.set(number);
		voltagePublisher.set(RobotController.getBatteryVoltage());
		isDriverStationConnected.set(DriverStation.isDSAttached());
		isBrownedOut.set(RobotController.isBrownedOut());
	}
	
	@Override
	public void activateShuffleboard() {
		// this should be called immediately
		ShuffleboardTab tab = Shuffleboard.getTab("Driver Station");
		tab.add("activate tabs", new ActivateTabs());
		tab.add("start alert", new InstantCommand(() -> alert.set(true)).ignoringDisable(true));
		tab.add("end alert", new InstantCommand(() -> alert.set(false)).ignoringDisable(true));
		tab.add("Auto Path", autoChooser);
	}
	
	@Override
	public String getNetworkTable() {
		return null;
	}
}
