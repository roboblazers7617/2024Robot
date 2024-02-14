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
import frc.robot.Constants.IntakeConstants;

public class Head extends SubsystemBase {
	// Shooter
	private final CANSparkMax shooterMotor = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_ID, MotorType.kBrushless);
	private final RelativeEncoder shooterEncoder = shooterMotor.getEncoder();
	private final SparkPIDController shooterController = shooterMotor.getPIDController();
	private final DigitalInput isNoteInShooter = new DigitalInput(ShooterConstants.SENSOR_DIO);

	/* 
	 * Two motor intake
	 * Always spin bottom motor forward, but reverse top motor to intake from source.
	 */
	private final CANSparkMax intakeMotorBottom = new CANSparkMax(IntakeConstants.MOTOR_BOTTOM_CAN_ID, MotorType.kBrushless);
	private final CANSparkMax intakeMotorTop = new CANSparkMax(IntakeConstants.MOTOR_TOP_CAN_ID, MotorType.kBrushless);
	private final DigitalInput isNoteAcquired = new DigitalInput(IntakeConstants.SENSOR_DIO);
	private final DigitalInput isNoteInPosition = new DigitalInput(IntakeConstants.POSITION_SENSOR_DIO);
	
	private boolean readyToShoot = false; // Is the shooter spun up to speed?
	private double shooterSetPoint = 0; // What speed should the shooter be spinning?

	/** Creates a new Head. */
	public Head() {
		shooterMotor.setIdleMode(IdleMode.kCoast);

		intakeMotorBottom.setIdleMode(IdleMode.kBrake);
		intakeMotorTop.setIdleMode(IdleMode.kBrake);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		shooterController.setReference(shooterSetPoint, ControlType.kVelocity);
	}

	private void setIntakeBottomSpeed(double intakeSpeed) {
		intakeMotorBottom.set(intakeSpeed);
	}

	private void setIntakeTopSpeed(double intakeSpeed) {
		intakeMotorTop.set(intakeSpeed);
	}

	public Command intakePiece(boolean isFromSource) {
		if (isFromSource) {
			return Commands.runOnce(() -> {setIntakeBottomSpeed(IntakeConstants.INTAKE_SPEED); setIntakeTopSpeed(-IntakeConstants.INTAKE_SPEED);}, this)
					.andThen(Commands.waitUntil(() -> isNoteAcquired()))
					.andThen(Commands.runOnce(() -> {setIntakeBottomSpeed(-IntakeConstants.ALIGNMENT_SPEED); setIntakeTopSpeed(-IntakeConstants.ALIGNMENT_SPEED);}))
					.andThen(Commands.waitUntil(() -> isNoteAligned()))
					.finallyDo(() -> {setIntakeBottomSpeed(0); setIntakeTopSpeed(0);});
		} else {
			return Commands.runOnce(() -> {setIntakeBottomSpeed(IntakeConstants.INTAKE_SPEED); setIntakeTopSpeed(IntakeConstants.INTAKE_SPEED);}, this)
					.andThen(Commands.waitUntil(() -> isNoteAcquired()))
					.andThen(Commands.runOnce(() -> {setIntakeBottomSpeed(-IntakeConstants.ALIGNMENT_SPEED); setIntakeTopSpeed(-IntakeConstants.ALIGNMENT_SPEED);}))
					.andThen(Commands.waitUntil(() -> isNoteAligned()))
					.finallyDo(() -> {setIntakeBottomSpeed(0); setIntakeTopSpeed(0);});
		}
	}

	public Command outakePiece() {
		return Commands.runOnce(() -> {setIntakeBottomSpeed(IntakeConstants.OUTAKE_SPEED); setIntakeTopSpeed(IntakeConstants.OUTAKE_SPEED);}, this)
				.andThen(Commands.waitUntil(() -> isNoteAcquired()))
				.finallyDo(() -> {setIntakeBottomSpeed(0); setIntakeTopSpeed(0);});
	}

	private void setShooterSpeed(int rpm) {
		shooterSetPoint = rpm;	
	}

	public double getShooterSpeed() {
		return shooterEncoder.getVelocity();
	}

	public double getShooterSetPoint() {
		return shooterSetPoint;
	}

	public Command spinUpShooter(ShooterConstants.ShootingPosition position) {
		return Commands.runOnce(() -> setShooterSpeed(position.rpm()), this)
				.andThen(Commands.waitUntil(() -> isShooterAtSpeed()))
				.finallyDo(() -> readyToShoot = true);
	}

	public Command spinDownShooter() {
		return Commands.runOnce(() -> {readyToShoot = false; setShooterSpeed(ShooterConstants.IDLE_SPEED);}, this)
				.andThen(Commands.waitUntil(() -> isShooterAtSpeed()));
	}

	public boolean isShooterAtSpeed() {
		return (getShooterSpeed() >= (shooterSetPoint - ShooterConstants.VELOCITY_MINIMUM)) && (getShooterSpeed() <= (shooterSetPoint + ShooterConstants.VELOCITY_MAXIMUM));
	}

	public boolean isReadyToShoot() {
		return readyToShoot;
	}

	public void setIsReadyToShoot(boolean isReadyToShoot) {
		readyToShoot = isReadyToShoot;
	}

	public Command shoot() {
		return Commands.waitUntil(() -> isReadyToShoot())
				.andThen(Commands.runOnce(() -> {setIntakeBottomSpeed(IntakeConstants.SHOOTING_INTAKE_SPEED); setIntakeTopSpeed(IntakeConstants.SHOOTING_INTAKE_SPEED);}))
				.andThen(Commands.waitUntil(() -> !isNoteInShooter()))
				.finallyDo(() -> {setIntakeBottomSpeed(0); setIntakeTopSpeed(0); spinDownShooter();});
	}

	public boolean isNoteAcquired() {
		return isNoteAcquired.get();
	}

	public boolean isNoteAligned() {
		return !isNoteInPosition.get();
	}
	
	public boolean isNoteInShooter() {
		return isNoteInShooter.get();
	}
}
