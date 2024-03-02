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
		
		shooterMotorBottom.setInverted(false);
		shooterMotorTop.setInverted(false);
		
		shooterMotorBottom.setSmartCurrentLimit(40);
		shooterMotorTop.setSmartCurrentLimit(40);
		
		// Intake motor
		intakeMotorBottom.restoreFactoryDefaults();
		intakeMotorTop.restoreFactoryDefaults();
		
		intakeMotorBottom.setIdleMode(IdleMode.kBrake);
		intakeMotorTop.setIdleMode(IdleMode.kBrake);
		
		intakeMotorBottom.setInverted(true);
		intakeMotorTop.setInverted(false);
		
		intakeMotorBottom.setSmartCurrentLimit(20);
		intakeMotorTop.setSmartCurrentLimit(20);
		
		// Shooter controller
		shooterControllerBottom.setP(ShooterConstants.kP.get());
		shooterControllerTop.setP(ShooterConstants.kP.get());
		shooterControllerBottom.setI(ShooterConstants.kI.get());
		shooterControllerTop.setI(ShooterConstants.kI.get());
		shooterControllerBottom.setD(ShooterConstants.kD.get());
		shooterControllerTop.setD(ShooterConstants.kD.get());
		shooterControllerBottom.setOutputRange(ShooterConstants.kMinOutput.get(), ShooterConstants.kMaxOutput.get());
		shooterControllerTop.setOutputRange(ShooterConstants.kMinOutput.get(), ShooterConstants.kMaxOutput.get());
		
		// Shooter interpolation map
		shooterInterpolationMap.put(-1.0, 1000.0); // Amp
		shooterInterpolationMap.put(0.0, 7000.0);
	}
	
	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		// Temperature alert
		if ((intakeMotorBottom.getMotorTemperature() > MOTOR_OVERHEAT_TEMPERATURE) || (intakeMotorTop.getMotorTemperature() > MOTOR_OVERHEAT_TEMPERATURE)) {
			motorTemperatureAlert.set(true);
		} else {
			motorTemperatureAlert.set(false);
		}
		
		// Shooter controller
		shooterControllerBottom.setP(ShooterConstants.kP.get());
		shooterControllerTop.setP(ShooterConstants.kP.get());
		shooterControllerBottom.setI(ShooterConstants.kI.get());
		shooterControllerTop.setI(ShooterConstants.kI.get());
		shooterControllerBottom.setD(ShooterConstants.kD.get());
		shooterControllerTop.setD(ShooterConstants.kD.get());
		shooterControllerBottom.setOutputRange(ShooterConstants.kMinOutput.get(), ShooterConstants.kMaxOutput.get());
		shooterControllerTop.setOutputRange(ShooterConstants.kMinOutput.get(), ShooterConstants.kMaxOutput.get());
	}
	
	public Double getIntakeEncoderBottom() {
		return intakeMotorBottom.getEncoder().getPosition();
	}
	
	public Double getIntakeEncoderTop() {
		return intakeMotorTop.getEncoder().getPosition();
	}
	
	public Double getShooterEncoderBottom() {
		return shooterMotorBottom.getEncoder().getPosition();
	}
	
	public Double getShooterEncoderTop() {
		return shooterMotorTop.getEncoder().getPosition();
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
	
	public Command StartIntake(boolean isFromSource) {
		if (isFromSource) {
			return Commands.runOnce(() -> {
				setIntakeSpeeds(IntakeConstants.INTAKE_SPEED, -IntakeConstants.INTAKE_SPEED);
			}, this);
		} else {
			return Commands.runOnce(() -> {
				setIntakeSpeeds(IntakeConstants.INTAKE_SPEED, IntakeConstants.INTAKE_SPEED);
			}, this);
		}
	}
	
	public Command StartOutake() {
		return Commands.runOnce(() -> {
			setIntakeSpeeds(IntakeConstants.OUTAKE_SPEED, IntakeConstants.OUTAKE_SPEED);
		}, this);
	}
	
	public Command StopIntake() {
		return Commands.runOnce(() -> {
			setIntakeSpeeds(0, 0);
		}, this);
	}
	
	public Command IntakePiece(boolean isFromSource) {
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
	
	public Command OutakePiece() {
		return Commands.runOnce(() -> {
			setIntakeSpeeds(IntakeConstants.OUTAKE_SPEED, IntakeConstants.OUTAKE_SPEED);
		}, this)
				.andThen(Commands.waitUntil(() -> !isNoteWithinHead()))
				.andThen(Commands.waitSeconds(1))
				.finallyDo(() -> {
					isNoteAcquired = false;
					setIntakeSpeeds(0, 0);
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
	
	public Command SpinUpShooter(Supplier<Double> positionMeters) {
		return Commands.runOnce(() -> {
			shooterIdle = false;
			setShooterSpeed(getShooterSpeedAtPosition(positionMeters.get()));
		}, this);
	}
	
	public Command IdleShooter() {
		return Commands.runOnce(() -> {
			setShooterSpeed(getShooterSpeedAtPosition(0));
		});
	}
	
	public Command StartShooterTest(){
		return Commands.runOnce(() -> {
			setShooterSpeed(5000);
		});
	}

	public Command StopShooterTest(){
		return Commands.runOnce(() -> {
			setShooterSpeed(0);
		});
	}
	public Command SpinDownShooter() {
		return Commands.runOnce(() -> {
			shooterIdle = true;
			setShooterSpeed(ShooterConstants.IDLE_SPEED);
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
	
	public Command Shoot() {
		// TODO: (Max) don't you need to command the shooter to spin up?
		return Commands.waitUntil(() -> isReadyToShoot())
				.andThen(Commands.runOnce(() -> {
					setIntakeSpeeds(IntakeConstants.FEEDER_SPEED.get(), IntakeConstants.FEEDER_SPEED.get());
				}))
				.andThen(Commands.waitUntil(() -> isNoteWithinHead()))
				.andThen(Commands.waitUntil(() -> !isNoteWithinHead()))
				.andThen(Commands.waitSeconds(0.5))
				.finallyDo(() -> {
					isNoteAcquired = false;
					setIntakeSpeeds(0, 0);
					SpinDownShooter();
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
