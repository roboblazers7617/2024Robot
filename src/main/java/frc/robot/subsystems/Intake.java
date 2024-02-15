// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
	private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.MOTOR_CAN_ID, MotorType.kBrushless);
	private final DigitalInput isNoteAcquired = new DigitalInput(IntakeConstants.SENSOR_DIO);
	
	private boolean noteAcquired = false;
	
	/** Creates a new Intake. */
	public Intake() {
		intakeMotor.setIdleMode(IdleMode.kBrake);
	}
	
	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
	
	public Command intakePiece() {
		return Commands.runOnce(() -> setIntakeSpeed(IntakeConstants.INTAKE_SPEED), this)
				.andThen(Commands.waitUntil(() -> isNoteAcquired()))
				.finallyDo(() -> setIntakeSpeed(0));
	}
	
	public Command outakePiece() {
		return Commands.runOnce(() -> setIntakeSpeed(IntakeConstants.OUTAKE_SPEED), this)
				.andThen(Commands.waitUntil(() -> isNoteAcquired()))
				.finallyDo(() -> setIntakeSpeed(0));
	}
	
	public boolean isNoteAcquired() {
		return noteAcquired;
	}
	
	public void setIsNoteAcquired(boolean isNoteAcquired) {
		noteAcquired = isNoteAcquired;
	}
	
	private void setIntakeSpeed(double intakeSpeed) {
		intakeMotor.set(intakeSpeed);
	}
}
