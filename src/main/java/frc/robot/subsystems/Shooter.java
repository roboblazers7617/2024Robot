// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

	private final CANSparkMax shooterMotor = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_ID, MotorType.kBrushless);
	private final SparkPIDController shooterController = shooterMotor.getPIDController();
	private final DigitalInput isNoteInShooter = new DigitalInput(ShooterConstants.SENSOR_DIO);


	/** Creates a new Shooter. */
	public Shooter() {
		shooterMotor.setIdleMode(IdleMode.kCoast);

	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public Command spinDown() {
		return Commands.runOnce(() -> setShooterSpeed(ShooterConstants.IDLE_SPEED), this)
				.andThen(Commands.waitUntil(() -> isAtSpeed()));
	}

	public Command spinUp(ShooterConstants.ShootingPosition position) {
		return Commands.runOnce(() -> setShooterSpeed(position.rpm()), this)
				.andThen(Commands.waitUntil(() -> isAtSpeed()));
	}

	public boolean isAtSpeed() {
		return false;
	}

	public boolean isNoteInShooter() {
		return isNoteInShooter.get();
	}

	private void setShooterSpeed(int rpm) {

	}
}
