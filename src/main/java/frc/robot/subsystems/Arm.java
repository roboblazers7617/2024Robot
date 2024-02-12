// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.shuffleboard.MotorTab;

public class Arm extends SubsystemBase {
	// Right motor and encoder
	private final CANSparkMax rightArmMotor = new CANSparkMax(ArmConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
	private final SparkAbsoluteEncoder rightAbsoluteEncoder = rightArmMotor
			.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
	private final SparkPIDController rightArmPIDController = rightArmMotor.getPIDController();

	// Left motor and encoder
	private final CANSparkMax leftArmMotor = new CANSparkMax(ArmConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
	private final SparkAbsoluteEncoder leftAbsoluteEncoder = leftArmMotor
			.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
	private final SparkPIDController leftArmPIDController = leftArmMotor.getPIDController();
	private double currentArmTarget = -1;

	private final ArmFeedforward armFeedFoward = new ArmFeedforward(ArmConstants.KS, ArmConstants.KG,
			ArmConstants.KV);

	// elevator motors and encoders
	// right motor
	private final CANSparkMax rightElevatorMotor = new CANSparkMax(ArmConstants.RIGHT_ELEVATOR_MOTOR_ID,
			MotorType.kBrushless);
	private final SparkAbsoluteEncoder rightElevatorAbsoluteEncoder = rightElevatorMotor
			.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
	private final SparkPIDController rightElevatorPIDController = rightElevatorMotor.getPIDController();

	// left motor
	private final CANSparkMax leftElevatorMotor = new CANSparkMax(ArmConstants.LEFT_ELEVATOR_MOTOR_ID,
			MotorType.kBrushless);
	private final SparkAbsoluteEncoder leftElevatorAbsoluteEncoder = leftElevatorMotor
			.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
	private final SparkPIDController leftElevatorPIDController = leftElevatorMotor.getPIDController();

	private final ElevatorFeedforward elevatorFeedforward;

	/** Creates a new Arm. */
	public Arm() {

		// setup arm motors

		rightArmMotor.restoreFactoryDefaults();
		rightArmMotor.setIdleMode(IdleMode.kBrake);
		rightArmMotor.setSmartCurrentLimit(20);

		leftArmMotor.restoreFactoryDefaults();
		leftArmMotor.setIdleMode(IdleMode.kBrake);
		leftArmMotor.setSmartCurrentLimit(20);
		leftArmMotor.setInverted(true);

		// setup arm pid controllers

		rightArmPIDController.setP(ArmConstants.KP);
		rightArmPIDController.setI(ArmConstants.KI);
		rightArmPIDController.setD(ArmConstants.KD);
		rightArmPIDController.setIZone(ArmConstants.kIz);
		rightArmPIDController.setOutputRange(ArmConstants.kMinOutput,
				ArmConstants.kMaxOutput);
		rightArmPIDController.setFeedbackDevice(rightAbsoluteEncoder);
		rightArmPIDController.setPositionPIDWrappingEnabled(true);

		leftArmPIDController.setP(ArmConstants.KP);
		leftArmPIDController.setI(ArmConstants.KI);
		leftArmPIDController.setD(ArmConstants.KD);
		leftArmPIDController.setIZone(ArmConstants.kIz);
		leftArmPIDController.setOutputRange(ArmConstants.kMinOutput, ArmConstants.kMaxOutput);
		leftArmPIDController.setFeedbackDevice(leftAbsoluteEncoder);
		leftArmPIDController.setPositionPIDWrappingEnabled(true);

		// setup arm encoders

		rightAbsoluteEncoder.setPositionConversionFactor(ArmConstants.ABS_POSITION_CONVERSION_FACTOR);
		rightAbsoluteEncoder.setVelocityConversionFactor(ArmConstants.ABS_VELOCITY_CONVERSION_FACTOR);
		rightAbsoluteEncoder.setInverted(false);

		leftAbsoluteEncoder.setPositionConversionFactor(ArmConstants.ABS_POSITION_CONVERSION_FACTOR);
		leftAbsoluteEncoder.setVelocityConversionFactor(ArmConstants.ABS_VELOCITY_CONVERSION_FACTOR);

		// setup elevator motors
		rightElevatorMotor.restoreFactoryDefaults();
		rightElevatorMotor.setIdleMode(IdleMode.kBrake);
		rightElevatorMotor.setSmartCurrentLimit(20);

		leftElevatorMotor.restoreFactoryDefaults();
		leftElevatorMotor.setIdleMode(IdleMode.kBrake);
		leftElevatorMotor.setSmartCurrentLimit(20);

		// setup elevator pid controllers
		rightElevatorPIDController.setP(ElevatorConstants.KP);
		rightElevatorPIDController.setI(ElevatorConstants.KI);
		rightElevatorPIDController.setD(ElevatorConstants.KD);
		rightElevatorPIDController.setIZone(ElevatorConstants.kIz);
		rightElevatorPIDController.setOutputRange(ElevatorConstants.kMinOutput,
				ElevatorConstants.kMaxOutput);
		rightElevatorPIDController.setFeedbackDevice(rightElevatorAbsoluteEncoder);

		leftElevatorPIDController.setP(ElevatorConstants.KP);
		leftElevatorPIDController.setI(ElevatorConstants.KI);
		leftElevatorPIDController.setD(ElevatorConstants.KD);
		leftElevatorPIDController.setIZone(ElevatorConstants.kIz);
		leftElevatorPIDController.setOutputRange(ElevatorConstants.kMinOutput,
				ElevatorConstants.kMaxOutput);
		leftElevatorPIDController.setFeedbackDevice(leftElevatorAbsoluteEncoder);
		

		leftAbsoluteEncoder.setInverted(true);

		elevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.KS, ElevatorConstants.KG, ElevatorConstants.KV);

		MotorTab.getInstance()
				.addMotor(new CANSparkMax[] { leftArmMotor, rightArmMotor, leftElevatorMotor, rightElevatorMotor });
	}

	// do something functions

	public void stopArm() {
		// disable();
		leftArmPIDController.setReference(leftAbsoluteEncoder.getPosition(), CANSparkMax.ControlType.kPosition, 0);
		rightArmPIDController.setReference(rightAbsoluteEncoder.getPosition(), CANSparkMax.ControlType.kPosition, 0);
	}

	public void setArmTarget(double target) {

		// make sure the move can be done safely
		// if the target is greater than the max height, set the target to the max
		// height
		if (target > ArmConstants.MAX_ANGLE) {
			target = ArmConstants.MAX_ANGLE;
		}
		// if the target is less than the min height, set the target to the min height
		if (target < 0) {
			target = 0;
		}

		// if the target is less than the MIN_ABOVE_PASS_ANGLE, make sure the arm is
		// extended, or retracted, if not, do nothing
		if (target < ArmConstants.MIN_ABOVE_PASS_ANGLE) {
			if (leftElevatorAbsoluteEncoder.getPosition() < ElevatorConstants.MIN_ABOVE_PASS_HEIGHT
					&& leftAbsoluteEncoder.getPosition() > ElevatorConstants.MAX_BELOW_PASS_HEIGHT) {
				return;
			}
		}

		double feedFowardValue = armFeedFoward.calculate(Units.degreesToRadians(target), 0);

		leftArmPIDController.setReference(target, CANSparkMax.ControlType.kPosition, 0,
				feedFowardValue, ArbFFUnits.kVoltage);
		rightArmPIDController.setReference(target, CANSparkMax.ControlType.kPosition, 0,
				feedFowardValue, ArbFFUnits.kVoltage);

	}

	public void setSelevatorTarget(double target){
		// make sure the move can be done safely
		// if the target is greater than the max height, set the target to the max
		if (target > ElevatorConstants.MAX_HEIGHT) {
			target = ElevatorConstants.MAX_HEIGHT;
		}
		// if the target is less than the min height, set the target to the min height
		if (target < ElevatorConstants.MIN_HEIGHT) {
			target = ElevatorConstants.MIN_HEIGHT;
		}
		// if the arm is below the MIN_ABOVE_PASS_ANGLE, abort
		if (leftAbsoluteEncoder.getPosition() < ArmConstants.MIN_ABOVE_PASS_ANGLE) {
			return;
		}

		double feedFowardValue = elevatorFeedforward.calculate(Units.degreesToRadians(target), 0);
		leftElevatorPIDController.setReference(target, CANSparkMax.ControlType.kPosition, 0,
				feedFowardValue, ArbFFUnits.kVoltage);
		rightElevatorPIDController.setReference(target, CANSparkMax.ControlType.kPosition, 0,
				feedFowardValue, ArbFFUnits.kVoltage);
	}

	@Override
	public void periodic() {

		if (ArmConstants.KP != leftArmPIDController.getP()) {
			leftArmPIDController.setP(ArmConstants.KP);
		}
		if (ArmConstants.KI != leftArmPIDController.getI()) {
			leftArmPIDController.setI(ArmConstants.KI);
		}
		if (ArmConstants.KD != leftArmPIDController.getD()) {
			leftArmPIDController.setD(ArmConstants.KD);
		}
	}

	// get info functions
	public double getRightAbsoluteEncoderPosition() {
		return rightAbsoluteEncoder.getPosition();
	}

	public double getLeftAbsoluteEncoderPosition() {
		return leftAbsoluteEncoder.getPosition();
	}

	public double getArmCurrentTarget() {
		return currentArmTarget;
	}

}
