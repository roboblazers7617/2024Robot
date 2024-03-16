// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class Head extends SubsystemBase {
	// Shooter
	private final CANSparkMax shooterMotorBottom = new CANSparkMax(ShooterConstants.MOTOR_BOTTOM_CAN_ID, MotorType.kBrushless);
	private final CANSparkMax shooterMotorTop = new CANSparkMax(ShooterConstants.MOTOR_TOP_CAN_ID, MotorType.kBrushless);
	private final RelativeEncoder shooterEncoderBottom = shooterMotorBottom.getEncoder();
	private final RelativeEncoder shooterEncoderTop = shooterMotorTop.getEncoder();
	private final SparkPIDController shooterControllerBottom = shooterMotorBottom.getPIDController();
	private final SparkPIDController shooterControllerTop = shooterMotorTop.getPIDController();
	
	private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.MOTOR_CAN_ID, MotorType.kBrushless);
	private final DigitalInput isNoteInSensor = new DigitalInput(IntakeConstants.NOTE_SENSOR_DIO);
	private boolean isNoteAcquired = false; // Since none of the sensors will be active when a note is intaken and aligned, this boolean is necessary to know if the robot has a note.
	
	private boolean shooterIdle = true; // Is the shooter set to the idle speed?
	private double shooterSetPoint = 0; // What speed should the shooter be spinning?
	
	/** Creates a new Head. */
	public Head() {
		// Shooter motor
		shooterMotorBottom.restoreFactoryDefaults();
		shooterMotorTop.restoreFactoryDefaults();
		
		shooterMotorBottom.setIdleMode(IdleMode.kCoast);
		shooterMotorTop.setIdleMode(IdleMode.kCoast);
		
		shooterMotorBottom.setInverted(false);
		shooterMotorTop.setInverted(false);
		
		shooterMotorBottom.setSmartCurrentLimit(40);
		shooterMotorTop.setSmartCurrentLimit(40);
		
		// Intake motor
		intakeMotor.restoreFactoryDefaults();
		intakeMotor.setIdleMode(IdleMode.kBrake);
		intakeMotor.setInverted(true);
		intakeMotor.setSmartCurrentLimit(40);
		
		// Shooter controller
		shooterControllerBottom.setP(ShooterConstants.BOTTOM_kP);
		shooterControllerTop.setP(ShooterConstants.TOP_kP);
		shooterControllerBottom.setI(ShooterConstants.BOTTOM_kI);
		shooterControllerTop.setI(ShooterConstants.TOP_kI);
		shooterControllerBottom.setD(ShooterConstants.BOTTOM_kD);
		shooterControllerTop.setD(ShooterConstants.TOP_kD);
		shooterControllerBottom.setOutputRange(ShooterConstants.BOTTOM_kMinOutput, ShooterConstants.BOTTOM_kMaxOutput);
		shooterControllerTop.setOutputRange(ShooterConstants.TOP_kMinOutput, ShooterConstants.TOP_kMaxOutput);
		
		// Burn motor controller configuration to flash
		Timer.delay(0.005);
		intakeMotor.burnFlash();
		Timer.delay(0.005);
		shooterMotorBottom.burnFlash();
		Timer.delay(0.005);
		shooterMotorTop.burnFlash();
	}
	
	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
	
	private void setIntakeSpeed(double intakeSpeed) {
		intakeMotor.set(intakeSpeed);
	}
	
	public Command StartIntake() {
		return Commands.runOnce(() -> {
			setIntakeSpeed(IntakeConstants.INTAKE_SPEED);
		}, this);
	}
	
	public Command StartOutake() {
		return Commands.runOnce(() -> {
			setIntakeSpeed(IntakeConstants.OUTAKE_SPEED);
		}, this);
	}
	
	public Command StopIntake() {
		return Commands.runOnce(() -> {
			setIntakeSpeed(0);
		}, this);
	}
	
	public Command IntakePiece() {
		return Commands.runOnce(() -> {
			setIntakeSpeed(IntakeConstants.INTAKE_SPEED);
		}, this)
				.andThen(Commands.waitUntil(() -> isNoteWithinSensor()))
				.finallyDo(() -> {
					isNoteAcquired = true;
					setIntakeSpeed(0);
				});
	}
	
	public Command OutakePiece() {
		return Commands.runOnce(() -> {
			setIntakeSpeed(IntakeConstants.OUTAKE_SPEED);
		}, this)
				.andThen(Commands.waitUntil(() -> !isNoteWithinSensor()))
				.andThen(Commands.waitSeconds(3))
				.finallyDo(() -> {
					isNoteAcquired = false;
					setIntakeSpeed(0);
				});
	}
	
	private void setShooterSpeed(double rpm) {
		shooterSetPoint = rpm;
		shooterControllerBottom.setReference(shooterSetPoint, ControlType.kVelocity);
		shooterControllerTop.setReference(shooterSetPoint, ControlType.kVelocity);
	}
	
	public double getShooterBottomSpeed() {
		return shooterEncoderBottom.getVelocity();
	}
	
	public double getShooterTopSpeed() {
		return shooterEncoderTop.getVelocity();
	}
	
	public double getShooterSetPoint() {
		return shooterSetPoint;
	}
	
	public Command SpinUpShooter(double rpm) {
		return Commands.runOnce(() -> {
			shooterIdle = false;
			setShooterSpeed(rpm);
		}, this);
	}
	
	public Command SpinUpShooterForSpeaker() {
		return SpinUpShooter(ShooterConstants.SPEAKER_SPEED);
	}
	
	public Command SpinUpShooterForAmp() {
		return SpinUpShooter(ShooterConstants.AMP_SPEED);
	}
	
	public Command SpinDownShooter() {
		return Commands.runOnce(() -> {
			shooterIdle = true;
			shooterMotorBottom.setVoltage(0);
			shooterMotorTop.setVoltage(0);
		}, this);
	}
	
	public boolean isReadyToShoot() {
		// @formatter:off
		return (
				(getShooterBottomSpeed() >= (shooterSetPoint * ShooterConstants.VELOCITY_MINIMUM)) &&
				(getShooterBottomSpeed() <= (shooterSetPoint * ShooterConstants.VELOCITY_MAXIMUM))
			) && (
				(getShooterTopSpeed() >= (shooterSetPoint * ShooterConstants.VELOCITY_MINIMUM)) &&
				(getShooterTopSpeed() <= (shooterSetPoint * ShooterConstants.VELOCITY_MAXIMUM))
			) && !shooterIdle;
		// @formatter:on
	}
	
	public Command Shoot(double rpm) {
		return SpinUpShooter(rpm)
				.andThen(Commands.waitUntil(() -> isReadyToShoot()))
				.andThen(Commands.waitSeconds(0.5))
				.andThen(Commands.runOnce(() -> {
					setIntakeSpeed(IntakeConstants.FEEDER_SPEED);
				}))
				.andThen(Commands.waitUntil(() -> isNoteWithinSensor()))
				.andThen(Commands.waitUntil(() -> !isNoteWithinSensor()))
				.andThen(Commands.waitSeconds(0.5))
				.andThen(SpinDownShooter())
				.finallyDo(() -> {
					isNoteAcquired = false;
					setIntakeSpeed(0);
				});
	}
	
	public Command ShootInSpeaker() {
		return Shoot(ShooterConstants.SPEAKER_SPEED);
	}
	
	public Command ShootInAmp() {
		return Shoot(ShooterConstants.AMP_SPEED);
	}
	
	public boolean isNoteWithinSensor() {
		return !isNoteInSensor.get();
	}
	
	// Does the intake have a note?
	public boolean isNoteAcquired() {
		return isNoteAcquired;
	}
	
	public Command ToggleBreakModes() {
		return new InstantCommand(() -> {
			if (intakeMotor.getIdleMode() == IdleMode.kBrake) {
				intakeMotor.setIdleMode(IdleMode.kCoast);
			} else {
				intakeMotor.setIdleMode(IdleMode.kBrake);
			}
		});
	}
}
