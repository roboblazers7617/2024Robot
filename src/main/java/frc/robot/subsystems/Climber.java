// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
	private final CANSparkMax rightClimber = new CANSparkMax(Constants.ClimberConstants.RIGHT_CLIMBER_PORT, MotorType.kBrushless);
	private final RelativeEncoder rightClimberEncoder;
	
	private final CANSparkMax leftClimber = new CANSparkMax(Constants.ClimberConstants.LEFT_CLIMBER_PORT, MotorType.kBrushless);
	private final RelativeEncoder leftClimberEncoder;
	
	/** Creates a new Climber. */
	public Climber() {
		rightClimber.restoreFactoryDefaults();
		
		leftClimber.restoreFactoryDefaults();
		
		rightClimber.setIdleMode(IdleMode.kBrake);
		rightClimber.setInverted(true);
		
		leftClimber.setIdleMode(IdleMode.kBrake);
		
		leftClimberEncoder = leftClimber.getEncoder();
		rightClimberEncoder = rightClimber.getEncoder();
		leftClimberEncoder.setPosition(0.0);
		rightClimberEncoder.setPosition(0.0);
		
		// balanceController.setSetpoint(0.0);
		
		burnFlash();
	}
	
	private void burnFlash() {
		Timer.delay(0.005);
		leftClimber.burnFlash();
		Timer.delay(0.005);
		rightClimber.burnFlash();
		Timer.delay(0.005);
	}
	
	public void setSpeed(double leftSpeed, double rightSpeed) {
		// leftClimber.set(speed);
		rightClimber.set(rightSpeed);
		leftClimber.set(leftSpeed);
	}
	
	public double getSpeedRight() {
		return rightClimberEncoder.getVelocity();
	}
	
	public double getSpeedLeft() {
		return leftClimberEncoder.getVelocity();
	}
	
	public void setSpeedLeft(double leftSpeed) {
		leftClimber.set(leftSpeed);
	}
	
	public void setSpeedRight(double rightSpeed) {
		rightClimber.set(rightSpeed);
	}
	
	// public void resetClimberEncoders() {
	// leftClimberEncoder.setPosition(0.0);
	// rightClimberEncoder.setPosition(0.0);
	// }
	
	public double getPositionRightMotor() {
		return rightClimberEncoder.getPosition();
	}
	
	public double getPositionLeftMotor() {
		return leftClimberEncoder.getPosition();
	}
	
	@Override
	public void periodic() {}
	
	public CANSparkMax[] getMotors() {
		return new CANSparkMax[] { leftClimber, rightClimber };
	}
}
