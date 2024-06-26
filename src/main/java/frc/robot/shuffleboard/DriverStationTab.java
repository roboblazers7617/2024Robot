// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shuffleboard;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.RobotStatus;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.FieldHelpers.TagLocation;

/** Add your docs here. */
public class DriverStationTab extends ShuffleboardTabBase {
	private final DoublePublisher voltagePublisher;
	private final BooleanPublisher isDriverStationConnected;
	private final BooleanPublisher isBrownedOut;
	private final BooleanPublisher isButtonPushed;
	private final BooleanPublisher areAutosLoaded;
	private final Field2d field = new Field2d();
	private final SendableChooser<TagLocation> locationChooser = new SendableChooser<>();
	private NetworkTableInstance inst = NetworkTableInstance.getDefault();
	private final String NAME = "Driver Station";
	private NetworkTable networkTable = inst.getTable("Shuffleboard/" + NAME);
	private ShuffleboardTab tab = Shuffleboard.getTab(NAME);
	private UsbCamera camera;
	private Drivetrain drivetrain;
	private final RobotStatus robotStatus;
	
	public DriverStationTab(Drivetrain drivetrain, RobotStatus robotStatus) {

		this.drivetrain = drivetrain;
		this.robotStatus = robotStatus;
		locationChooser.setDefaultOption(TagLocation.AMP.toString(), TagLocation.AMP);
		locationChooser.addOption(TagLocation.SOURCE_CLOSE.toString(), TagLocation.SOURCE_CLOSE);
		locationChooser.addOption(TagLocation.SPEAKER.toString(), TagLocation.SPEAKER);
		
		voltagePublisher = networkTable.getDoubleTopic("Battery Voltage").publish();
		isDriverStationConnected = networkTable.getBooleanTopic("Is Driver Station Connected").publish();
		isBrownedOut = networkTable.getBooleanTopic("Is Browned Out").publish();
		isButtonPushed = networkTable.getBooleanTopic("Arm Braked?").publish();
		areAutosLoaded = networkTable.getBooleanTopic("Autos Loaded?").publish();
		
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
		voltagePublisher.set(robotStatus.batteryVoltage());
		isDriverStationConnected.set(DriverStation.isDSAttached());
		isBrownedOut.set(robotStatus.isBrownedOut());
		isButtonPushed.set(robotStatus.isMechanismBraked());
		field.setRobotPose(drivetrain.getPose());
		areAutosLoaded.set(AutoConstants.AUTOS_LOADED);
	}
	
	@Override
	public void activateShuffleboard() {
		// this should be called immediately
		tab.add("Choose Pose Location", locationChooser)
			.withPosition(0, 0).withSize(2, 1);
		tab.add("Reset Pose", drivetrain.ResetPose(locationChooser::getSelected))
			.withPosition(2, 0).withSize(2, 1);
		tab.add("Field", field)
			.withPosition(0, 1)
			.withSize(5,3);
		tab.add("DS Connected?", DriverStation.isDSAttached()).withPosition(5, 0).withWidget(BuiltInWidgets.kBooleanBox);
		tab.add("Autos Loaded?", AutoConstants.AUTOS_LOADED).withPosition(6, 0).withWidget(BuiltInWidgets.kBooleanBox);
		tab.add("Arm Braked?", robotStatus.isMechanismBraked()).withPosition(5, 1).withWidget(BuiltInWidgets.kBooleanBox);
		tab.add("Battery Volts", robotStatus.batteryVoltage()).withPosition(5, 2);
		tab.add("CAN Errors ", robotStatus.numberCANErrors()).withPosition(5, 3).withWidget(BuiltInWidgets.kTextView);

	
	}
	
	@Override
	public String getNetworkTable() {
		return null;
	}

}
