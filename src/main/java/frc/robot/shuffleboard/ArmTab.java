package frc.robot.shuffleboard;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.Arm;

public class ArmTab extends ShuffleboardTabBase {
	private final Arm arm;
	
	private final DoublePublisher armAbsoluteEncoderPub;
	private final DoublePublisher elevatorPub;
	
	public ArmTab(Arm arm) {
		this.arm = arm;
		
		NetworkTableInstance inst = NetworkTableInstance.getDefault();
		
		NetworkTable networkTable = inst.getTable("Shuffleboard/arm");
		
		armAbsoluteEncoderPub = networkTable.getDoubleTopic("Arm Absolute Encoder").publish();
		elevatorPub = networkTable.getDoubleTopic("Elevator").publish();
		Shuffleboard.getTab("arm").add("toggle brake modes", arm.ToggleBrakeModes());
	}
	
	@Override
	public void update() {
		armAbsoluteEncoderPub.set(arm.getArmAbsoluteEncoderPosition());
		elevatorPub.set(arm.getElevatorAbsoluteEncoderPosition());
	}
	
		// tab.add("toggle brake modes", arm.ToggleBrakeModes());
		// tab.add("extend elevator", arm.RaiseElevator());
		// tab.add("retract elevator", arm.lowerElevator());
		// tab.add("stow", arm.Stow());
}
