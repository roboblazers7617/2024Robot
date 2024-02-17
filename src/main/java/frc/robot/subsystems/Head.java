// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.Constants.IntakeConstants;

public class Head extends SubsystemBase {
	// Shooter
	private final CANSparkMax shooterMotorBottom = new CANSparkMax(ShooterConstants.MOTOR_BOTTOM_CAN_ID, MotorType.kBrushless);
	private final CANSparkMax shooterMotorTop = new CANSparkMax(ShooterConstants.MOTOR_TOP_CAN_ID, MotorType.kBrushless);
	private final RelativeEncoder shooterEncoderBottom = shooterMotorBottom.getEncoder();
	private final RelativeEncoder shooterEncoderTop = shooterMotorTop.getEncoder();
	private final SparkPIDController shooterControllerBottom = shooterMotorBottom.getPIDController();
	private final SparkPIDController shooterControllerTop = shooterMotorTop.getPIDController();
	// private final DigitalInput isNoteInShooter = new DigitalInput(ShooterConstants.NOTE_SHOT_SENSOR_DIO);
	
	/*
	 * Two motor intake
	 * Always spin bottom motor forward, but reverse top motor to intake from source.
	 */
	private final CANSparkMax intakeMotorBottom = new CANSparkMax(IntakeConstants.MOTOR_BOTTOM_CAN_ID, MotorType.kBrushless);
	private final CANSparkMax intakeMotorTop = new CANSparkMax(IntakeConstants.MOTOR_TOP_CAN_ID, MotorType.kBrushless);
	private final DigitalInput isNoteWithinHead = new DigitalInput(IntakeConstants.NOTE_POSSESSION_SENSOR_DIO);
	private final DigitalInput isNoteInPosition = new DigitalInput(IntakeConstants.NOTE_ALIGNMENT_SENSOR_DIO);
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
		
		shooterMotorBottom.setInverted(true);
		shooterMotorTop.setInverted(true);
		
		shooterMotorBottom.setSmartCurrentLimit(20);
		shooterMotorTop.setSmartCurrentLimit(20);
		
		// Intake motor
		intakeMotorBottom.restoreFactoryDefaults();
		intakeMotorTop.restoreFactoryDefaults();
		
		intakeMotorBottom.setIdleMode(IdleMode.kBrake);
		intakeMotorTop.setIdleMode(IdleMode.kBrake);
		
		intakeMotorBottom.setInverted(false);
		intakeMotorTop.setInverted(true);
		
		intakeMotorBottom.setSmartCurrentLimit(20);
		intakeMotorTop.setSmartCurrentLimit(20);
		
		// Shooter controller
		shooterControllerBottom.setP(ShooterConstants.kP);
		shooterControllerTop.setP(ShooterConstants.kP);
		shooterControllerBottom.setI(ShooterConstants.kI);
		shooterControllerTop.setI(ShooterConstants.kI);
		shooterControllerBottom.setD(ShooterConstants.kD);
		shooterControllerTop.setD(ShooterConstants.kD);
		shooterControllerBottom.setIZone(ShooterConstants.kIZone);
		shooterControllerTop.setIZone(ShooterConstants.kIZone);
		shooterControllerBottom.setFF(ShooterConstants.kFF);
		shooterControllerTop.setFF(ShooterConstants.kFF);
		shooterControllerBottom.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);
		shooterControllerTop.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);
	}
	
	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		// Temperature alert
		if ((intakeMotorBottom.getMotorTemperature() > 85) || (intakeMotorTop.getMotorTemperature() > 85)) {
			motorTemperatureAlert.set(true);
		} else {
			motorTemperatureAlert.set(false);
		}
	}
	
	private void setIntakeBottomSpeed(double intakeSpeed) {
		intakeMotorBottom.set(intakeSpeed);
	}
	
	private void setIntakeTopSpeed(double intakeSpeed) {
		intakeMotorTop.set(intakeSpeed);
	}
	
	private void setIntakeSpeeds(double intakeBottomSpeed, double intakeTopSpeed) {
		setIntakeBottomSpeed(intakeBottomSpeed);
		setIntakeTopSpeed(intakeTopSpeed);
	}
	
	public Command intakePiece(boolean isFromSource) {
		if (isFromSource) {
			return Commands.runOnce(() -> {
				setIntakeSpeeds(IntakeConstants.INTAKE_SPEED, -IntakeConstants.INTAKE_SPEED);
			}, this)
					.andThen(Commands.waitUntil(() -> isNoteWithinHead()))
					.andThen(Commands.runOnce(() -> {
						setIntakeSpeeds(-IntakeConstants.ALIGNMENT_SPEED, -IntakeConstants.ALIGNMENT_SPEED);
					}))
					.andThen(Commands.waitUntil(() -> isNoteAligned()))
					.finallyDo(() -> {
						isNoteAcquired = true;
						setIntakeSpeeds(0, 0);
					});
		} else {
			return Commands.runOnce(() -> {
				setIntakeSpeeds(IntakeConstants.INTAKE_SPEED, IntakeConstants.INTAKE_SPEED);
			}, this)
					.andThen(Commands.waitUntil(() -> isNoteWithinHead()))
					.andThen(Commands.runOnce(() -> {
						setIntakeSpeeds(-IntakeConstants.ALIGNMENT_SPEED, -IntakeConstants.ALIGNMENT_SPEED);
					}))
					.andThen(Commands.waitUntil(() -> isNoteAligned()))
					.finallyDo(() -> {
						isNoteAcquired = true;
						setIntakeSpeeds(0, 0);
					});
		}
	}
	
	public Command outakePiece() {
		return Commands.runOnce(() -> {
			isNoteAcquired = false;
			setIntakeSpeeds(IntakeConstants.OUTAKE_SPEED, IntakeConstants.OUTAKE_SPEED);
		}, this)
				.andThen(Commands.waitUntil(() -> !isNoteWithinHead()))
				.andThen(Commands.waitSeconds(1))
				.finallyDo(() -> {
					setIntakeSpeeds(0, 0);
				});
	}
	
	private void setShooterSpeed(int rpm) {
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
	
	public Command spinUpShooter(ShooterConstants.ShootingPosition position) {
		return Commands.runOnce(() -> {
			shooterIdle = false;
			setShooterSpeed(position.rpm());
		}, this);
	}
	
	public Command spinDownShooter() {
		return Commands.runOnce(() -> {
			shooterIdle = true;
			setShooterSpeed(ShooterConstants.IDLE_SPEED);
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
	
	public Command shoot() {
		return Commands.waitUntil(() -> isReadyToShoot())
				.andThen(Commands.runOnce(() -> {
					isNoteAcquired = false;
					setIntakeSpeeds(IntakeConstants.FEEDER_SPEED, IntakeConstants.FEEDER_SPEED);
				}))
				.andThen(Commands.waitUntil(() -> isNoteWithinHead()))
				.andThen(Commands.waitUntil(() -> !isNoteWithinHead()))
				.andThen(Commands.waitSeconds(0.5))
				.finallyDo(() -> {
					setIntakeSpeeds(0, 0);
					spinDownShooter();
				});
	}
	
	public boolean isNoteWithinHead() {
		// TODO: (Max) In 2022 we ran into some funkiness where this didn't work. We couldn't figure out why. If this doesn't work, you just need to do this...
		// boolean sensorVal = shooterSensor.get();
		// return !sensorVal;
		return isNoteWithinHead.get();
	}
	
	public boolean isNoteAligned() {
		// TODO: (Max) In 2022 we ran into some funkiness where this didn't work. We couldn't figure out why. If this doesn't work, you just need to do this...
		// boolean sensorVal = shooterSensor.get();
		// return !sensorVal;
		return !isNoteInPosition.get();
	}
	
	// Does the intake have a note?
	public boolean isNoteAcquired() {
		return isNoteAcquired;
	}
	
	// public boolean isNoteInShooter() {
	// // TODO: (Max) In 2022 we ran into some funkiness where this didn't work. We couldn't figure out why. If this doesn't work, you just need to do this...
	// // boolean sensorVal = shooterSensor.get();
	// // return !sensorVal;
	// return isNoteInShooter.get();
	// }
}
