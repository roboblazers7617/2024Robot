// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ButtonBox extends SubsystemBase {
	private final DoubleSubscriber rotationSubscriber;
	
	/** Creates a new ButtonBox. */
	public ButtonBox() {
		NetworkTableInstance inst = NetworkTableInstance.getDefault();
		NetworkTable networkTable = inst.getTable("controller");

		NetworkTable wheelTable = inst.getTable("controller/wheel");
		rotationSubscriber = wheelTable.getDoubleTopic("Rotation").subscribe(0);
	}
	
	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public double getWheelPosition() {
		return rotationSubscriber.get();
	}
}
