// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ShootingConstants.ShootingPosition;
import frc.robot.shuffleboard.MotorTab;
// import frc.robot.util.TunableNumber;

public class Arm extends SubsystemBase {
	// Arm
	/** this is the right arm motor */
	private final CANSparkMax leaderArmMotor = new CANSparkMax(ArmConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
	private final CANSparkMax followerArmMotor = new CANSparkMax(ArmConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
	
	private final SparkAbsoluteEncoder armAbsoluteEncoder = leaderArmMotor
			.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
	
	private final SparkPIDController armPIDController = leaderArmMotor.getPIDController();
	
	private ArmFeedforward extendedArmFeedForward = new ArmFeedforward(ArmConstants.EXTENDED_KS, ArmConstants.EXTENDED_KG, ArmConstants.EXTENDED_KV);
	private ArmFeedforward retractedArmFeedForward = new ArmFeedforward(ArmConstants.RETRACTED_KS, ArmConstants.RETRACTED_KG, ArmConstants.RETRACTED_KV);
	
	/** the current target for the arm, in degrees, it is within the total bounds of the arm but may not be a currently safe move */
	// of the arm so the arm doesn't try to move on boot-up
	private double armTarget;
	/** the last actual arm target */
	private double lastAcutalArmTarget;
	/** arm angle based on distance interpolation table */
	private final InterpolatingDoubleTreeMap armAngleBasedOnDistanceExtended = new InterpolatingDoubleTreeMap();
	private final InterpolatingDoubleTreeMap armAngleBasedOnDistanceRetracted = new InterpolatingDoubleTreeMap();
	
	// Elevator
	/** the right motor */
	private final CANSparkMax leaderElevatorMotor = new CANSparkMax(ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
	/** the left motor */
	private final CANSparkMax followerElevatorMotor = new CANSparkMax(ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
	
	private final RelativeEncoder elevatorEncoder = leaderElevatorMotor.getEncoder();
	
	private final SparkPIDController elevatorPIDController = leaderElevatorMotor.getPIDController();
	
	InterpolatingDoubleTreeMap elevatorKSTable = new InterpolatingDoubleTreeMap();
	InterpolatingDoubleTreeMap elevatorKGTable = new InterpolatingDoubleTreeMap();
	InterpolatingDoubleTreeMap elevatorKVTable = new InterpolatingDoubleTreeMap();
	
	private ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.KS, ElevatorConstants.KG, ElevatorConstants.KV);
	
	/** the current target for the elevator, it is within the total bounds of the arm but may not be a currently safe move */
	private double elevatorTarget;
	/** the last actual elevator target */
	private double lastAcutalElevatorTarget;
	
	/** this is used for the position setpoint, in degrees, for setVelocity() */
	private double dt, lastTime;
	private Timer time = new Timer();
	
	private final MotorTab motorTab = new MotorTab(4, "arm", 2);
	
	/** Creates a new Arm. */
	public Arm() {
		// setup arm motors
		
		leaderArmMotor.restoreFactoryDefaults();
		leaderArmMotor.setIdleMode(IdleMode.kBrake);
		leaderArmMotor.setSmartCurrentLimit(ArmConstants.MAX_AMPERAGE);
		leaderArmMotor.setInverted(true);
		
		followerArmMotor.restoreFactoryDefaults();
		followerArmMotor.setIdleMode(IdleMode.kBrake);
		followerArmMotor.setSmartCurrentLimit(ArmConstants.MAX_AMPERAGE);
		followerArmMotor.follow(leaderArmMotor, true);
		
		followerArmMotor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus0, 1000);
		followerArmMotor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus1, 1000);
		followerArmMotor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus3, 1000);
		followerArmMotor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus4, 1000);
		followerArmMotor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus5, 1000);
		followerArmMotor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus6, 1000);
		followerArmMotor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus7, 1000);
		followerArmMotor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus2, 1000);
		
		// setup the arm pid controller
		armPIDController.setP(ArmConstants.KP);
		armPIDController.setI(ArmConstants.KI);
		armPIDController.setD(ArmConstants.KD);
		armPIDController.setOutputRange(ArmConstants.kMinOutput, ArmConstants.kMaxOutput);
		armPIDController.setFeedbackDevice(armAbsoluteEncoder);
		armPIDController.setPositionPIDWrappingEnabled(false);
		
		// setup the arm encoder
		armAbsoluteEncoder.setPositionConversionFactor(ArmConstants.ABS_POSITION_CONVERSION_FACTOR);
		armAbsoluteEncoder.setVelocityConversionFactor(ArmConstants.ABS_VELOCITY_CONVERSION_FACTOR);
		armAbsoluteEncoder.setInverted(true);
		armAbsoluteEncoder.setZeroOffset(ArmConstants.ARM_OFFSET);
		
		armTarget = armAbsoluteEncoder.getPosition();
		
		armAngleBasedOnDistanceExtended.put(1.27, ShootingPosition.SUBWOOFER.arm_angle());
		armAngleBasedOnDistanceExtended.put(2.9, 33.5);
		armAngleBasedOnDistanceExtended.put(3.1, 36.0);
		
		// TODO: Add Treemap values
		armAngleBasedOnDistanceRetracted.put(1.96, 18.6);
		armAngleBasedOnDistanceRetracted.put(2.47, 27.0);
		armAngleBasedOnDistanceRetracted.put(2.92, 31.8);
		armAngleBasedOnDistanceRetracted.put(2.96, 30.9);
		armAngleBasedOnDistanceRetracted.put(3.51, 35.1);
		armAngleBasedOnDistanceRetracted.put(3.61, 32.7);
		armAngleBasedOnDistanceRetracted.put(4.11, 37.3);
		armAngleBasedOnDistanceRetracted.put(4.25, 38.3);
		armAngleBasedOnDistanceRetracted.put(4.29, 38.3);
		armAngleBasedOnDistanceRetracted.put(4.31, 37.6);
		
		followerElevatorMotor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus0, 1000);
		followerElevatorMotor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus1, 1000);
		followerElevatorMotor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus3, 1000);
		followerElevatorMotor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus4, 1000);
		followerElevatorMotor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus5, 1000);
		followerElevatorMotor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus6, 1000);
		followerElevatorMotor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus7, 1000);
		followerElevatorMotor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus2, 1000);
		// setup elevator motors
		leaderElevatorMotor.restoreFactoryDefaults();
		leaderElevatorMotor.setIdleMode(IdleMode.kBrake);
		leaderElevatorMotor.setSmartCurrentLimit(ElevatorConstants.MAX_AMPERAGE);
		leaderElevatorMotor.setInverted(true);
		
		followerElevatorMotor.restoreFactoryDefaults();
		followerElevatorMotor.setIdleMode(IdleMode.kBrake);
		followerElevatorMotor.setSmartCurrentLimit(ElevatorConstants.MAX_AMPERAGE);
		followerElevatorMotor.follow(leaderArmMotor, true);
		
		elevatorTarget = elevatorEncoder.getPosition();
		
		elevatorPIDController.setP(ElevatorConstants.KP);
		elevatorPIDController.setI(ElevatorConstants.KI);
		elevatorPIDController.setD(ElevatorConstants.KD);
		elevatorPIDController.setOutputRange(ElevatorConstants.kMinOutput, ElevatorConstants.kMaxOutput);
		
		time.reset();
		time.start();
		
		burnFlash();
	}
	
	private void burnFlash() {
		Timer.delay(0.005);
		leaderArmMotor.burnFlash();
		Timer.delay(0.005);
		followerArmMotor.burnFlash();
		Timer.delay(0.005);
		leaderElevatorMotor.burnFlash();
		Timer.delay(0.005);
		followerElevatorMotor.burnFlash();
		Timer.delay(0.005);
	}
	
	private ElevatorFeedforward getElevatorFeedforward() {
		return elevatorFeedforward;
	}
	
	private ArmFeedforward getArmFeedforward() {
		return elevatorEncoder.getPosition() > ElevatorConstants.MIN_ABOVE_PASS_HEIGHT || ElevatorConstants.KILL_IT_ALL ? extendedArmFeedForward : retractedArmFeedForward;
		// use just one feedforward for now, if we need 2, use line above
		// return extendedArmFeedForward;
	}
	
	// do something functions
	
	/**
	 * safely set the target angle for the arm
	 * 
	 * @param targetDegrees
	 *                the target angle for the arm in degrees
	 */
	public void setArmTarget(double targetDegrees) {
		// make sure the move can be done safely
		targetDegrees = MathUtil.clamp(targetDegrees, ArmConstants.MIN_ANGLE, ArmConstants.MAX_ANGLE);
		
		armTarget = targetDegrees;
	}
	
	/**
	 * sets the arm target based on the distance to the speaker and the interpolation table
	 * 
	 * @param distance
	 *                the distance to the speaker in meters
	 */
	public void setArmTargetByDistanceExtended(double distance) {
		armTarget = MathUtil.clamp(armAngleBasedOnDistanceExtended.get(distance), ArmConstants.MIN_ANGLE, ArmConstants.MAX_ANGLE);
	}
	
	public void setArmTargetByDistanceRetracted(double distance) {
		armTarget = MathUtil.clamp(armAngleBasedOnDistanceRetracted.get(distance), ArmConstants.MIN_ANGLE, ArmConstants.MAX_ANGLE);
	}
	
	public Command RaiseElevator() {
		return this.runOnce(() -> setElevatorTarget(ElevatorConstants.MAX_HEIGHT));
	}
	
	public Command lowerElevator() {
		return this.runOnce(() -> setElevatorTarget(ElevatorConstants.MIN_HEIGHT));
	}
	
	/**
	 * stows the arm and elevator
	 * 
	 * @return a command to stow the arm and elevator
	 */
	public Command Stow() {
		return this.runOnce(() -> {
			setArmTarget(ArmConstants.STOW_ANGLE);
			setElevatorTarget(ElevatorConstants.MIN_HEIGHT);
		});
	}
	
	/**
	 * sets the velocity for the arm by moving a position setpoint
	 * 
	 * @param velocityDegreesPerSec
	 *                the velocity for the arm in degrees per second
	 */
	public void setArmVelocity(double velocityDegreesPerSec) {
		armTarget = armTarget + velocityDegreesPerSec * dt;
		armTarget = MathUtil.clamp(armTarget, ArmConstants.MIN_ANGLE, ArmConstants.MAX_ANGLE);
	}
	
	/**
	 * safely set the target height for the elevator
	 * 
	 * @param target
	 *                the target height for the elevator in inches
	 */
	public void setElevatorTarget(double target) {
		// make sure the move can be done safely
		// if the target is greater than the max height, set the target to the max
		if (target > ElevatorConstants.MAX_HEIGHT) {
			target = ElevatorConstants.MAX_HEIGHT;
		}
		// if the target is less than the min height, set the target to the min height
		if (target < ElevatorConstants.MIN_HEIGHT) {
			target = ElevatorConstants.MIN_HEIGHT;
		}
		elevatorTarget = target;
	}
	
	public Command SetTargets(ShootingPosition position) {
		return Commands.runOnce(() -> {
			setArmTarget(position.arm_angle());
			setElevatorTarget(position.elevator_target());
		});
	}
	
	public Command SetTargets(Supplier<Double> distance) {
		return Commands.runOnce(() -> {
			// TODO: Remove me!
			System.out.println("Distance is " + distance.get());
			setArmTargetByDistanceRetracted(distance.get());
			setElevatorTarget(ElevatorConstants.MIN_HEIGHT);
		});
	}
	
	public Command SetTargetsAuto(Supplier<Double> distance) {
		return Commands.runOnce(() -> {
			setArmTargetByDistanceExtended(distance.get());
			setElevatorTarget(ElevatorConstants.MAX_HEIGHT);
		});
	}
	
	/**
	 * sets the velocity for the elevator by moving a position setpoint
	 * 
	 * @param velocityDegreesPerSec
	 *                the velocity for the elevator in degrees per second
	 */
	public void setElevatorVelocity(double velocityDegreesPerSec) {
		elevatorTarget = elevatorTarget + velocityDegreesPerSec * dt;
		elevatorTarget = MathUtil.clamp(elevatorTarget, ElevatorConstants.MIN_HEIGHT, ElevatorConstants.MAX_HEIGHT);
		// System.out.println("elevator target in volocity: " + elevatorTarget);
	}
	
	public Command ArmDefaultCommand(Supplier<Double> armVelocity, Supplier<Double> elevatorVelocity) {
		Command command = new RunCommand(() -> {
			setArmVelocity(armVelocity.get());
			setElevatorVelocity(elevatorVelocity.get());
		});
		command.addRequirements(this);
		
		return command;
	}
	
	public Command ToggleBrakeModes() {
		return this.runOnce(() -> {
			if (leaderArmMotor.getIdleMode() == IdleMode.kBrake) {
				leaderArmMotor.setIdleMode(IdleMode.kCoast);
				followerArmMotor.setIdleMode(IdleMode.kCoast);
			} else {
				leaderArmMotor.setIdleMode(IdleMode.kBrake);
				followerArmMotor.setIdleMode(IdleMode.kBrake);
			}
			if (leaderElevatorMotor.getIdleMode() == IdleMode.kBrake) {
				leaderElevatorMotor.setIdleMode(IdleMode.kCoast);
				followerElevatorMotor.setIdleMode(IdleMode.kCoast);
			} else {
				leaderElevatorMotor.setIdleMode(IdleMode.kBrake);
				followerElevatorMotor.setIdleMode(IdleMode.kBrake);
			}
		}).ignoringDisable(true);
	}
	
	public Command EnableBrakeMode() {
		return this.runOnce(() -> {
			leaderArmMotor.setIdleMode(IdleMode.kBrake);
			followerArmMotor.setIdleMode(IdleMode.kBrake);
			leaderElevatorMotor.setIdleMode(IdleMode.kBrake);
			followerElevatorMotor.setIdleMode(IdleMode.kBrake);
		}).ignoringDisable(true);
	}
	
	@Override
	public void periodic() {
		dt = time.get() - lastTime;
		lastTime = time.get();
		
		// Arm
		
		// current arm target will be the reference set by the PID controller, based on what is currently safe
		double currentArmTarget = armTarget;
		
		// if the arm is less than the threshold to go over the bumper
		if (currentArmTarget < ArmConstants.MIN_ABOVE_PASS_ANGLE) {
			if (elevatorEncoder.getPosition() < ElevatorConstants.MIN_ABOVE_PASS_HEIGHT) { // and the elevator is not
				// extended
				currentArmTarget = ArmConstants.MIN_ABOVE_PASS_ANGLE;
			}
		}
		if (lastAcutalArmTarget != currentArmTarget) {
			ArmFeedforward armFeedFoward = getArmFeedforward();
			double velocity = 0;
			if (Math.abs(currentArmTarget - armAbsoluteEncoder.getPosition()) > ArmConstants.ARM_VELOCITY_DEADBAND) {
				velocity = armAbsoluteEncoder.getVelocity();
			}
			double armFeedFowardValue = armFeedFoward.calculate(Units.degreesToRadians(currentArmTarget), velocity);
			// System.out.println("arm feed foward: " + armFeedFowardValue);
			
			armPIDController.setReference(currentArmTarget, CANSparkMax.ControlType.kPosition, 0, armFeedFowardValue, ArbFFUnits.kVoltage);
			lastAcutalArmTarget = currentArmTarget;
		}
		if (!ElevatorConstants.KILL_IT_ALL) {
			// Elevator
			// current elevator target will be the reference set by the PID controller, based on what is currently safe
			double currentElevatorTarget = elevatorTarget;
			// if the arm is less than the threshold to go over the bumper, than the elevator needs to stay on its current side of the bumper
			if (armAbsoluteEncoder.getPosition() < ArmConstants.MIN_ABOVE_PASS_ANGLE) {
				if (currentElevatorTarget < ElevatorConstants.MIN_ABOVE_PASS_HEIGHT && elevatorEncoder.getPosition() > ElevatorConstants.MIN_ABOVE_PASS_HEIGHT) {
					currentElevatorTarget = ElevatorConstants.MIN_ABOVE_PASS_HEIGHT;
				}
			}
			
			if (currentElevatorTarget != lastAcutalElevatorTarget) {
				double elevatorFeedFowardValue = getElevatorFeedforward().calculate(elevatorEncoder.getVelocity());
				elevatorPIDController.setReference(currentElevatorTarget, CANSparkMax.ControlType.kPosition, 0, elevatorFeedFowardValue, ArbFFUnits.kVoltage);
				lastAcutalElevatorTarget = currentElevatorTarget;
			}
			
			motorTab.update();
		}
	}
	
	// get info functions
	public double getArmAbsoluteEncoderPosition() {
		return armAbsoluteEncoder.getPosition();
	}
	
	public double getElevatorAbsoluteEncoderPosition() {
		return elevatorEncoder.getPosition();
	}
	
	public MotorTab getMotorTab() {
		return motorTab;
	}
	
	public boolean areArmAndElevatorAtTarget() {
		return (Math.abs(armTarget - armAbsoluteEncoder.getPosition()) < ArmConstants.ARM_AT_TARGET_DEADBAND) && (Math.abs(elevatorTarget - elevatorEncoder.getPosition()) < ElevatorConstants.ELEVATOR_AT_TARGET_DEADBAND || ElevatorConstants.KILL_IT_ALL);
	}
}
