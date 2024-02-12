// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
// import edu.wpi.first.math.controller.ArmFeedforward;
import frc.robot.Constants.ArmConstants;
import frc.robot.shuffleboard.MotorTab;

public class Arm extends SubsystemBase {
	// Right motor and encoder
	// private final CANSparkMax rightMotor = new
	// CANSparkMax(ClimberConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
	// private final AbsoluteEncoder rightAbsoluteEncoder = rightMotor
	// .getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
	// private final SparkPIDController rightPIDController =
	// rightMotor.getPIDController();

	// Left motor and encoder
	private final CANSparkMax leftMotor = new CANSparkMax(ArmConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
	private final SparkAbsoluteEncoder leftAbsoluteEncoder = leftMotor
			.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
	private final RelativeEncoder leftRelativeEncoder = leftMotor.getEncoder();
	private final SparkPIDController leftPIDController = leftMotor.getPIDController();
	private double currentTarget = -1;

	private final ArmFeedforward feedFoward = new ArmFeedforward(ArmConstants.KS, ArmConstants.KG,
			ArmConstants.KV);
	private final SysIdRoutine routine;

	/** Creates a new Climber. */
	public Arm() {
		// super(new TrapezoidProfile.Constraints(ArmConstants.MAX_VELOCITY,
		// ArmConstants.MAX_ACCELERATION), 0);
		// disable();

		// rightMotor.restoreFactoryDefaults();
		// rightMotor.setIdleMode(IdleMode.kBrake);
		// rightMotor.setSmartCurrentLimit(20);

		leftMotor.restoreFactoryDefaults();
		leftMotor.setIdleMode(IdleMode.kBrake);
		leftMotor.setSmartCurrentLimit(20);
		leftMotor.setInverted(true);

		// rightPIDController.setP(ClimberConstants.KP);
		// rightPIDController.setI(ClimberConstants.KI);
		// rightPIDController.setD(ClimberConstants.KD);
		// rightPIDController.setIZone(ClimberConstants.kIz);
		// rightPIDController.setOutputRange(ClimberConstants.kMinOutput,
		// ClimberConstants.kMaxOutput);
		// rightPIDController.setFeedbackDevice(rightAbsoluteEncoder);
		// rightPIDController.setSmartMotionAccelStrategy(SparkPIDController.AccelStrategy.kTrapezoidal,
		// ClimberConstants.SLOT_ID);
		// rightPIDController.setSmartMotionMinOutputVelocity(ClimberConstants.minVel,
		// ClimberConstants.SLOT_ID);
		// rightPIDController.setSmartMotionMaxVelocity(ClimberConstants.MAX_VELOCITY,
		// ClimberConstants.SLOT_ID);
		// rightPIDController.setSmartMotionAllowedClosedLoopError(ClimberConstants.MAX_ERROR,
		// ClimberConstants.SLOT_ID);
		// rightPIDController.setSmartMotionMaxAccel(ClimberConstants.MAX_ACCELERATION,
		// ClimberConstants.SLOT_ID);

		leftPIDController.setP(ArmConstants.KP);
		leftPIDController.setI(ArmConstants.KI);
		leftPIDController.setD(ArmConstants.KD);
		leftPIDController.setIZone(ArmConstants.kIz);
		leftPIDController.setOutputRange(ArmConstants.kMinOutput, ArmConstants.kMaxOutput);
		leftPIDController.setFeedbackDevice(leftAbsoluteEncoder);
		leftPIDController.setPositionPIDWrappingEnabled(true);

		// rightAbsoluteEncoder.setPositionConversionFactor(ClimberConstants.ABS_POSITION_CONVERSION_FACTOR);
		// rightAbsoluteEncoder.setVelocityConversionFactor(ClimberConstants.ABS_VELOCITY_CONVERSION_FACTOR);
		// // rightAbsoluteEncoder.setZeroOffset(ClimberConstants.OFFSET);
		// rightAbsoluteEncoder.setInverted(false);

		leftAbsoluteEncoder.setPositionConversionFactor(ArmConstants.ABS_POSITION_CONVERSION_FACTOR);
		leftAbsoluteEncoder.setVelocityConversionFactor(ArmConstants.ABS_VELOCITY_CONVERSION_FACTOR);
		// leftAbsoluteEncoder.setZeroOffset(ClimberConstants.OFFSET);
		leftAbsoluteEncoder.setInverted(true);

		leftRelativeEncoder.setPositionConversionFactor(ArmConstants.ABS_POSITION_CONVERSION_FACTOR);
		leftRelativeEncoder.setVelocityConversionFactor(ArmConstants.ABS_VELOCITY_CONVERSION_FACTOR);
		// leftRelativeEncoder.setInverted(true);

		routine = new SysIdRoutine(
				new SysIdRoutine.Config(),
				new SysIdRoutine.Mechanism(
						(Measure<Voltage> volts) -> {
							leftMotor.setVoltage(volts.in(Volts));
						},
						log -> {

							log.motor("arm")
									.voltage(
											m_appliedVoltage.mut_replace(
													leftMotor.get() * RobotController.getBatteryVoltage(), Volts))
									.angularPosition(
											m_distance.mut_replace(leftMotor.getEncoder().getPosition(), Degrees))
									.angularVelocity(
											m_velocity.mut_replace(leftMotor.getEncoder().getVelocity(),
													DegreesPerSecond));
						},
						this));

		MotorTab.getInstance().addMotor(new CANSparkMax[] { leftMotor });
	}

	// @Override
	// public void useState(TrapezoidProfile.State setpoint) {
	// leftPIDController.setReference(setpoint.position,
	// CANSparkBase.ControlType.kPosition);
	// currentTarget = setpoint.position;
	// if (setpoint.position > maxCurrentTarget) {
	// maxCurrentTarget = setpoint.position;
	// }
	// // System.out.println(setpoint.position);
	// // System.out.println(setpoint.velocity);

	// }

	// do something functions

	public void stopArm() {
		// disable();
		leftPIDController.setReference(leftAbsoluteEncoder.getPosition(), CANSparkMax.ControlType.kPosition, 0);
	}

	public void setTarget(double target) {
		// the velocity setpoint should be zero
		double feedFowardValue = feedFoward.calculate(Units.degreesToRadians(target), 0);

		leftPIDController.setReference(target, CANSparkMax.ControlType.kPosition, 0,
				feedFowardValue, ArbFFUnits.kVoltage);

		// setGoal(target);

		// setGoal(new TrapezoidProfile.State(target, 0));
	}

	@Override
	public void periodic() {

		if (ArmConstants.KP != leftPIDController.getP()) {
			leftPIDController.setP(ArmConstants.KP);
		}
		if (ArmConstants.KI != leftPIDController.getI()) {
			leftPIDController.setI(ArmConstants.KI);
		}
		if (ArmConstants.KD != leftPIDController.getD()) {
			leftPIDController.setD(ArmConstants.KD);
		}
		// This method will be called once per scheduler run
		System.out.println("KP: " + leftPIDController.getP() + " KI: " + leftPIDController.getI() + " KD: "
				+ leftPIDController.getD());
	}

	// get info functions
	public double getRightAbsoluteEncoderPosition() {
		// return rightAbsoluteEncoder.getPosition();
		return -1;
	}

	public double getLeftAbsoluteEncoderPosition() {
		return leftAbsoluteEncoder.getPosition();
	}

	public double getCurrentTarget() {
		return currentTarget;
	}

	// Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
	private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
	// Mutable holder for unit-safe linear distance values, persisted to avoid
	// reallocation.
	private final MutableMeasure<Angle> m_distance = mutable(Degrees.of(0));
	// Mutable holder for unit-safe linear velocity values, persisted to avoid
	// reallocation.
	private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(DegreesPerSecond.of(0));

	public Command SysidQuasistatic(SysIdRoutine.Direction direction) {
		return routine.quasistatic(direction);
	}

	public Command SysidDynamic(SysIdRoutine.Direction direction) {
		return routine.dynamic(direction);
	}

}
