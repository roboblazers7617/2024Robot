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
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
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
	private double currentTarget = -1;

	private final ArmFeedforward feedFoward = new ArmFeedforward(ArmConstants.KS, ArmConstants.KG,
			ArmConstants.KV);

	/** Creates a new Climber. */
	public Arm() {

		rightArmMotor.restoreFactoryDefaults();
		rightArmMotor.setIdleMode(IdleMode.kBrake);
		rightArmMotor.setSmartCurrentLimit(20);

		leftArmMotor.restoreFactoryDefaults();
		leftArmMotor.setIdleMode(IdleMode.kBrake);
		leftArmMotor.setSmartCurrentLimit(20);
		leftArmMotor.setInverted(true);

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

		rightAbsoluteEncoder.setPositionConversionFactor(ArmConstants.ABS_POSITION_CONVERSION_FACTOR);
		rightAbsoluteEncoder.setVelocityConversionFactor(ArmConstants.ABS_VELOCITY_CONVERSION_FACTOR);
		rightAbsoluteEncoder.setInverted(false);

		leftAbsoluteEncoder.setPositionConversionFactor(ArmConstants.ABS_POSITION_CONVERSION_FACTOR);
		leftAbsoluteEncoder.setVelocityConversionFactor(ArmConstants.ABS_VELOCITY_CONVERSION_FACTOR);
		leftAbsoluteEncoder.setInverted(true);

		MotorTab.getInstance().addMotor(new CANSparkMax[] { leftArmMotor });
	}

	// do something functions

	public void stopArm() {
		// disable();
		leftArmPIDController.setReference(leftAbsoluteEncoder.getPosition(), CANSparkMax.ControlType.kPosition, 0);
		rightArmPIDController.setReference(rightAbsoluteEncoder.getPosition(), CANSparkMax.ControlType.kPosition, 0);
	}

	public void setTarget(double target) {
		// the velocity setpoint should be zero
		double feedFowardValue = feedFoward.calculate(Units.degreesToRadians(target), 0);

		leftArmPIDController.setReference(target, CANSparkMax.ControlType.kPosition, 0,
				feedFowardValue, ArbFFUnits.kVoltage);
		rightArmPIDController.setReference(target, CANSparkMax.ControlType.kPosition, 0,
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

	public double getCurrentTarget() {
		return currentTarget;
	}

}
