package frc.robot.shuffleboard;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;

public class ArmTab extends ShuffleboardTabBase {
	private final Arm arm;

	private final DoublePublisher rightAbsoluteEncoderPub;
	private final DoublePublisher leftAbsoluteEncoderPub;
	private final DoublePublisher currentTarget;
	// private final DoublePublisher odometryYPub;
	// private final DoublePublisher odometryXPub;
	// private final DoublePublisher odometryAnglePub;

	public ArmTab(Arm arm) {
		this.arm = arm;

		NetworkTableInstance inst = NetworkTableInstance.getDefault();

		NetworkTable networkTable = inst.getTable("logging/arm");

		rightAbsoluteEncoderPub = networkTable.getDoubleTopic("Right Absolute Encoder").publish();
		leftAbsoluteEncoderPub = networkTable.getDoubleTopic("Left Absolute Encoder").publish();
		currentTarget = networkTable.getDoubleTopic("Current Target").publish();


	}

	@Override
	public void update() {
		rightAbsoluteEncoderPub.set(arm.getRightAbsoluteEncoderPosition());
		leftAbsoluteEncoderPub.set(arm.getLeftAbsoluteEncoderPosition());
		currentTarget.set(arm.getCurrentTarget());

	}

	@Override
	public void activateShuffleboard() {
		ShuffleboardTab tab = Shuffleboard.getTab("arm");
		tab.add("raise arm", new InstantCommand(() -> arm.raiseArm()).ignoringDisable(true));
		tab.add("lower arm", new InstantCommand(() -> arm.lowerArm()).ignoringDisable(true));
		tab.add("stop arm", new InstantCommand(() -> arm.stopArm()).ignoringDisable(true));
	}

	@Override
	public String getNetworkTable() {
		return "arm";
	}

}
