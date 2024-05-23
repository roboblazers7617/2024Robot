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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.Constants.ShootingConstants.ShootingPosition;

public class Head extends SubsystemBase {
	// Shooter
	private final CANSparkMax shooterMotorBottom = new CANSparkMax(ShooterConstants.MOTOR_BOTTOM_CAN_ID, MotorType.kBrushless);
	private final CANSparkMax shooterMotorTop = new CANSparkMax(ShooterConstants.MOTOR_TOP_CAN_ID, MotorType.kBrushless);
	private final RelativeEncoder shooterEncoderBottom = shooterMotorBottom.getEncoder();
	private final RelativeEncoder shooterEncoderTop = shooterMotorTop.getEncoder();
	private final SparkPIDController shooterControllerBottom = shooterMotorBottom.getPIDController();
	private final SparkPIDController shooterControllerTop = shooterMotorTop.getPIDController();
	
	private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.MOTOR_CAN_ID, MotorType.kBrushless);
	// TODO: Please rename isNoteInSensor as is very ambigious and doesn't say which sensor. If I understand this correctly should be something like isNoteInShootPosition
	private final DigitalInput isNoteInShootPosition = new DigitalInput(IntakeConstants.NOTE_SENSOR_DIO);
	private final DigitalInput isNoteInIntake = new DigitalInput(IntakeConstants.NOTE_ALIGNMENT_SENSOR_DIO);
	
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
				.andThen(Commands.waitUntil(() -> isNoteWithinAlignmentSensor()))
				.andThen(Commands.runOnce(() -> {
					setIntakeSpeed(IntakeConstants.ALIGMNMENT_SPEED);
				}))
				.andThen(Commands.waitUntil(() -> isNoteWithinSensor()))
				.andThen(() -> {
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
					setIntakeSpeed(0);
				});
	}
	
	private void setShooterSpeed(double rpm) {
		shooterSetPoint = rpm;
		shooterControllerBottom.setReference(shooterSetPoint, ControlType.kVelocity);
		shooterControllerTop.setReference(shooterSetPoint, ControlType.kVelocity);
	}
	
	public void stopShooter() {
		shooterSetPoint = 0.0;
		shooterMotorBottom.setVoltage(0);
		shooterMotorTop.setVoltage(0);
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
			setShooterSpeed(rpm);
		}, this);
	}
	
	public Command SpinUpShooter(ShootingPosition position) {
		return SpinUpShooter(position.rpm());
	}
	
	public Command SpinDownShooter() {
		return Commands.runOnce(() -> {
			shooterSetPoint = 0.0;
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
			);
		// @formatter:on
	}
	
	// TODO: Move the finallyDo to an andThen to avoid the delay that we saw yesterday with the intake
	// TODO: Why are you spinning down the shooter before stopping the intake? Intake should be first
	public Command Shoot() {
		return Shoot(true);
	}
	
	public Command Shoot(boolean stopShooter) {
		return Commands.waitUntil(() -> isReadyToShoot())
				
				.andThen(Commands.runOnce(() -> {
					setIntakeSpeed(IntakeConstants.FEEDER_SPEED);
				}))
				
				.andThen(Commands.waitUntil(() -> isNoteWithinSensor()))
				
				.andThen(Commands.waitUntil(() -> !isNoteWithinSensor()))
				
				.andThen(Commands.waitSeconds(0.2))
				
				.andThen(Commands.either(SpinDownShooter().andThen(() -> setIntakeSpeed(0.0)),
						
						Commands.none(), () -> stopShooter));
	}
	
	public boolean isNoteWithinSensor() {
		return !isNoteInShootPosition.get();
	}
	
	public boolean isNoteWithinAlignmentSensor() {
		return !isNoteInIntake.get();
	}
	
	public Command ToggleBreakModes() {
		return new InstantCommand(() -> {
			if (intakeMotor.getIdleMode() == IdleMode.kBrake) {
				intakeMotor.setIdleMode(IdleMode.kCoast);
			} else {
				intakeMotor.setIdleMode(IdleMode.kBrake);
			}
		}).ignoringDisable(true);
	}
	
	public Command EnableBrakeMode() {
		return new InstantCommand(() -> {
			intakeMotor.setIdleMode(IdleMode.kBrake);
		});
	}
}
