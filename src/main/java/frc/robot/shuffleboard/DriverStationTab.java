// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shuffleboard;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotContainer;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

/** Add your docs here. */
public class DriverStationTab extends ShuffleboardTabBase {
	private final DoublePublisher voltagePublisher;
	private final BooleanPublisher isDriverStationConnected;
	private final BooleanPublisher isBrownedOut;
	private final BooleanPublisher isButtonPushed;
	private final DigitalInput button;
	
	private SendableChooser<Command> autoChooser = new SendableChooser<>();
	
	private UsbCamera camera;
	
	public DriverStationTab(SendableChooser<Command> autoChooser, DigitalInput button) {
		this.autoChooser = autoChooser;
		
		NetworkTableInstance inst = NetworkTableInstance.getDefault();
		
		NetworkTable networkTable = inst.getTable("Shuffleboard/Driver Station");
		
		voltagePublisher = networkTable.getDoubleTopic("Battery Voltage").publish();
		isDriverStationConnected = networkTable.getBooleanTopic("Is Driver Station Connected").publish();
		isBrownedOut = networkTable.getBooleanTopic("Is Browned Out").publish();
		this.button = button;
		isButtonPushed = networkTable.getBooleanTopic("is brake button pushed").publish();
		
		// camera = CameraServer.startAutomaticCapture();
		/*
		 * if (camera.isConnected()) {
		 * camera.setResolution(480, 320);
		 * camera.setFPS(10);
		 * }
		 */
	}
	
	@Override
	public void update() {
		voltagePublisher.set(RobotController.getBatteryVoltage());
		isDriverStationConnected.set(DriverStation.isDSAttached());
		isBrownedOut.set(RobotController.isBrownedOut());
		isButtonPushed.set(button.get());
	}
	
	@Override
	public void activateShuffleboard() {
		// this should be called immediately
		ShuffleboardTab tab = Shuffleboard.getTab("Driver Station");
		tab.add("Auto Path", autoChooser);
	}
	
	@Override
	public String getNetworkTable() {
		return null;
	}
}
