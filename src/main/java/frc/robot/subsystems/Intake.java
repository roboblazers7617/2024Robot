// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

	/** Creates a new Intake. */
	public Intake() {
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public Command intakePiece() {
		return Commands.runOnce(() -> setIntakeSpeed(IntakeConstants.INTAKE_SPEED), this)
				.andThen(Commands.waitUntil(() -> isNoteAcquired())).finallyDo(() -> setIntakeSpeed(0));
	}

	public Command outakePiece() {
		return Commands.runOnce(() -> setIntakeSpeed(IntakeConstants.OUTAKE_SPEED), this)
				.andThen(Commands.waitUntil(() -> isNoteAcquired())).finallyDo(() -> setIntakeSpeed(0));
	}

	public boolean isNoteAcquired() {
		return false;
	}

	private void setIntakeSpeed(double intakeSpeed) {
	}
}
