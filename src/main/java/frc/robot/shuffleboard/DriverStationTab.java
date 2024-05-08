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
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.Alert;
import frc.robot.util.Logging;
import frc.robot.util.Alert.AlertType;

/** Add your docs here. */
public class DriverStationTab extends ShuffleboardTabBase {
	// private final DoublePublisher voltagePublisher;
	// private final BooleanPublisher isDriverStationConnected;
	// private final BooleanPublisher isBrownedOut;
	// private final BooleanPublisher isButtonPushed;
	// private final DigitalInput button;
	// private final Logging[] numbers = new Logging[1000]; // STRESS TEST CODE
	private double counter = 0;
	
	private SendableChooser<Command> autoChooser = new SendableChooser<>();
	
	private UsbCamera camera;
	
	public DriverStationTab(SendableChooser<Command> autoChooser, DigitalInput button) {
		this.autoChooser = autoChooser;
		
		NetworkTableInstance inst = NetworkTableInstance.getDefault();
		
		NetworkTable networkTable = inst.getTable("Shuffleboard/Driver Station");
		
		// voltagePublisher = networkTable.getDoubleTopic("Battery Voltage").publish();
		// isDriverStationConnected = networkTable.getBooleanTopic("Is Driver Station Connected").publish();
		// isBrownedOut = networkTable.getBooleanTopic("Is Browned Out").publish();
		// this.button = button;
		// isButtonPushed = networkTable.getBooleanTopic("is brake button pushed").publish();
		// for (int i = 0; i < numbers.length; i++) { // STRESS TEST CODE
		// 	numbers[i] = new Logging("Driver Station", "number: " + i, false); // STRESS TEST CODE
		// } // STRESS TEST CODE

		// create a command to set Constants.debugMode to true
		Shuffleboard.getTab("Driver Station").add(new InstantCommand(() -> Constants.debugMode = true).ignoringDisable(true));
		
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
		// voltagePublisher.set(RobotController.getBatteryVoltage());
		// isDriverStationConnected.set(DriverStation.isDSAttached());
		// isBrownedOut.set(RobotController.isBrownedOut());
		// isButtonPushed.set(button.get());
		// for (Logging number : numbers) { // STRESS TEST CODE
		// 	number.log(counter); // STRESS TEST CODE
		// } // STRESS TEST CODE
		// counter++; // STRESS TEST CODE
	}
	

}
