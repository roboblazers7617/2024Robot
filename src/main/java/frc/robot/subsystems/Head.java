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

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

public class Head extends SubsystemBase {
	// Temperature at which the overheat alert should be triggered
	private static final double MOTOR_OVERHEAT_TEMPERATURE = 85;
	
	// Shooter
	private final CANSparkMax shooterMotorBottom = new CANSparkMax(ShooterConstants.MOTOR_BOTTOM_CAN_ID, MotorType.kBrushless);
	private final CANSparkMax shooterMotorTop = new CANSparkMax(ShooterConstants.MOTOR_TOP_CAN_ID, MotorType.kBrushless);
	private final RelativeEncoder shooterEncoderBottom = shooterMotorBottom.getEncoder();
	private final RelativeEncoder shooterEncoderTop = shooterMotorTop.getEncoder();
	private final SparkPIDController shooterControllerBottom = shooterMotorBottom.getPIDController();
	private final SparkPIDController shooterControllerTop = shooterMotorTop.getPIDController();
	// private final DigitalInput isNoteInShooter = new
	// DigitalInput(ShooterConstants.NOTE_SHOT_SENSOR_DIO);
	private final InterpolatingDoubleTreeMap shooterInterpolationMap = new InterpolatingDoubleTreeMap();
	
	/*
	 * Two motor intake
	 * Always spin bottom motor forward, but reverse top motor to intake from source.
	 */
	private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.MOTOR_CAN_ID, MotorType.kBrushless);
	private final DigitalInput isNoteInSensor = new DigitalInput(IntakeConstants.NOTE_SENSOR_DIO);
	private boolean isNoteAcquired = false; // Since none of the sensors will be active when a note is intaken and aligned, this boolean is necessary to know if the robot has a note.
	
	private boolean shooterIdle = true; // Is the shooter set to the idle speed?
	private double shooterSetPoint = 0; // What speed should the shooter be spinning?
	
	private final Alert motorTemperatureAlert = new Alert("Intake motor too hot!", AlertType.ERROR);
	
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
		intakeMotor.setSmartCurrentLimit(20);
		
		// Shooter controller
		shooterControllerBottom.setP(ShooterConstants.BOTTOM_kP);
		shooterControllerTop.setP(ShooterConstants.TOP_kP);
		shooterControllerBottom.setI(ShooterConstants.BOTTOM_kI);
		shooterControllerTop.setI(ShooterConstants.TOP_kI);
		shooterControllerBottom.setD(ShooterConstants.BOTTOM_kD);
		shooterControllerTop.setD(ShooterConstants.TOP_kD);
		shooterControllerBottom.setOutputRange(ShooterConstants.BOTTOM_kMinOutput, ShooterConstants.BOTTOM_kMaxOutput);
		shooterControllerTop.setOutputRange(ShooterConstants.TOP_kMinOutput, ShooterConstants.TOP_kMaxOutput);
		
		// Shooter interpolation map
		shooterInterpolationMap.put(0.0, 6000.0);
		
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
		// Temperature alert
		if (intakeMotor.getMotorTemperature() > MOTOR_OVERHEAT_TEMPERATURE) {
			motorTemperatureAlert.set(true);
		} else {
			motorTemperatureAlert.set(false);
		}
	}
	
	public Double getIntakeEncoder() {
		return intakeMotor.getEncoder().getPosition();
	}
	
	public Double getShooterEncoderBottom() {
		return shooterMotorBottom.getEncoder().getPosition();
	}
	
	public Double getShooterEncoderTop() {
		return shooterMotorTop.getEncoder().getPosition();
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
	
	public double getShooterSpeedAtPosition(double positionMeters) {
		return shooterInterpolationMap.get(positionMeters);
	}
	
	public void setShooterSpeedAtPosition(double positionMeters, double rpm) {
		shooterInterpolationMap.put(positionMeters, rpm);
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
	
	public Command SpinUpShooterAtPosition(double positionMeters) {
		return SpinUpShooter(getShooterSpeedAtPosition(positionMeters));
	}
	
	public Command SpinUpShooterForAmp() {
		return SpinUpShooter(ShooterConstants.AMP_SPEED);
	}
	
	public Command IdleShooter() {
		return Commands.runOnce(() -> {
			shooterIdle = true;
			setShooterSpeed(getShooterSpeedAtPosition(0));
		});
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
				(getShooterBottomSpeed() >= (shooterSetPoint * ShooterConstants.VELOCITY_MINIMUM.get())) &&
				(getShooterBottomSpeed() <= (shooterSetPoint * ShooterConstants.VELOCITY_MAXIMUM.get()))
			) && (
				(getShooterTopSpeed() >= (shooterSetPoint * ShooterConstants.VELOCITY_MINIMUM.get())) &&
				(getShooterTopSpeed() <= (shooterSetPoint * ShooterConstants.VELOCITY_MAXIMUM.get()))
			) && !shooterIdle;
		// @formatter:on
	}
	
	public Command Shoot(double rpm) {
		return SpinUpShooter(rpm)
				.andThen(Commands.waitUntil(() -> isReadyToShoot()))
				.andThen(Commands.waitSeconds(0.5))
				.andThen(Commands.runOnce(() -> {
					setIntakeSpeed(IntakeConstants.FEEDER_SPEED.get());
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
	
	public Command ShootAtPosition(double position) {
		return Shoot(getShooterSpeedAtPosition(position));
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
}
