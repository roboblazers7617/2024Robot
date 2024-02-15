// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {
	/** Creates a new Pivot. */
	public Pivot() {}
	
	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
	
	public Command moveToPosition(PivotConstants.PivotPosition position) {
		return Commands.runOnce(() -> setPivotAngle(position.angle()), this).andThen(Commands.waitUntil(() -> isAtSetpoint()));
	}
	
	public boolean isAtSetpoint() {
		return false;
	}
	
	private void setPivotAngle(double angle) {
	}
}
