package frc.robot.shuffleboard;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import frc.robot.subsystems.Arm;

public class ArmTab extends ShuffleboardTabBase {
	private final Arm arm;
	
	private final DoublePublisher armAbsoluteEncoderPub;
	private final DoublePublisher elevatorPub;
	
	public ArmTab(Arm arm) {
		this.arm = arm;
		
		NetworkTableInstance inst = NetworkTableInstance.getDefault();
		
		NetworkTable networkTable = inst.getTable("logging/arm");
		
		armAbsoluteEncoderPub = networkTable.getDoubleTopic("Arm Absolute Encoder").publish();
		elevatorPub = networkTable.getDoubleTopic("Elevator").publish();
	}
	
	@Override
	public void update() {
		armAbsoluteEncoderPub.set(arm.getArmAbsoluteEncoderPosition());
		elevatorPub.set(arm.getElevatorAbsoluteEncoderPosition());
	}
	
	@Override
	public void activateShuffleboard() {
		ShuffleboardTab tab = Shuffleboard.getTab("arm");
		// TODO: (Brandon) Can you remove all the .ignoringDisable(true) as we don't want those to accidentally
		// get put back in the code as it could be an arm safety issue
		// tab.add("raise arm", new InstantCommand(() -> arm.raiseArm()).ignoringDisable(true));
		// tab.add("add elevator feed foward values", arm.addElevatorFeedFowardValuesCommand());
		// tab.add("generate new arm feed foward values", arm.generateNewArmFeedFoward());
		// tab.add("generate new elevator feed foward values", arm.generateNewElevatorFeedFoward());
		tab.add("arm subsystem", arm).withPosition(0, 0);
		tab.add("toggle brake modes", arm.ToggleBrakeModes()).withPosition(2, 0);
		tab.add("extend elevator", arm.RaiseElevator()).withPosition(2, 1);
		tab.add("retract elevator", arm.lowerElevator()).withPosition(2,2);
		tab.add("stow", arm.Stow()).withPosition(2, 3);
		// tab.add("stop arm", new InstantCommand(() -> arm.stopArm()).ignoringDisable(true));
		// tab.add("foward run SysidQuasistatic", arm.SysidQuasistatic(Direction.kForward));
		// tab.add("backward run SysidQuasistatic", arm.SysidQuasistatic(Direction.kReverse));
		arm.getMotorTab().activateShuffleboard();
	}
	
	@Override
	public String getNetworkTable() {
		return "arm";
	}
}
