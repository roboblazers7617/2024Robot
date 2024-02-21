package frc.robot.shuffleboard;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.arm.RaiseArm;
import frc.robot.subsystems.Arm;

public class ArmTab extends ShuffleboardTabBase {
	private final Arm arm;

	private final DoublePublisher rightAbsoluteEncoderPub;
	//TODO: (Brandon) There will only be one absolute encoder
	// private final DoublePublisher odometryYPub;
	// private final DoublePublisher odometryXPub;
	// private final DoublePublisher odometryAnglePub;

	public ArmTab(Arm arm) {
		this.arm = arm;

		NetworkTableInstance inst = NetworkTableInstance.getDefault();

		NetworkTable networkTable = inst.getTable("logging/arm");

		rightAbsoluteEncoderPub = networkTable.getDoubleTopic("Right Absolute Encoder").publish();
		//TODO: (Brandon) You will need more data than this to debug. You will also need the motorEncoder of 
		//both arm motors so you can see which direction they are moving when the arm moves. 

		//TODO: (Brandon) You will need data about the elevator which is not added here. 
		// Think about what you need to tune values and make sure that motors are moving the way you expect


	}

	@Override
	public void update() {
		rightAbsoluteEncoderPub.set(arm.getRightAbsoluteEncoderPosition());

	}

	@Override
	public void activateShuffleboard() {
		ShuffleboardTab tab = Shuffleboard.getTab("arm");
		//TODO: (Brandon) Can you remove all the .ignoringDisable(true) as we don't want those to accidentally
		// get put back in the code as it could be an arm safety issue
		// tab.add("raise arm", new InstantCommand(() -> arm.raiseArm()).ignoringDisable(true));
		tab.add("raise arm ", new RaiseArm(arm, true));
		tab.add("add elevator feed foward values", arm.addElevatorFeedFowardValuesCommand());
		// tab.add("lower arm", new InstantCommand(() -> arm.lowerArm()).ignoringDisable(true));
		// tab.add("stop arm", new InstantCommand(() -> arm.stopArm()).ignoringDisable(true));
		// tab.add("foward run SysidQuasistatic", arm.SysidQuasistatic(Direction.kForward));
		// tab.add("backward run SysidQuasistatic", arm.SysidQuasistatic(Direction.kReverse));
	}

	@Override
	public String getNetworkTable() {
		return "arm";
	}

}