package frc.robot.util;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.TestNumber;


/**
 * How to add your constant
 * <p> 1. Make sure to remove final from variable declaration
 * <p> 2. Add {@code private final IntegerEntry yourVariable}
 * <p> 3. Add {@code yourVariable = networkTable.getIntegerTopic("variable name on shuffleboard").getEntry(0);
		yourVariableName.set(Constants.YOUR_CONSTANT);}
 * <p> 4. Add {@code Constants.YOUR_CONSTANT = (int) YourVariable.get();}
 */
public class TunableNumber extends SubsystemBase{
	// How to add your constant
	// 1. Make sure to remove final from variable declaration
	// 2. declare 

	private final IntegerEntry number;
	private final DoubleEntry maxClimberHeight;
	public TunableNumber(){
		NetworkTableInstance inst = NetworkTableInstance.getDefault();
		NetworkTable networkTable = inst.getTable("Shuffleboard/Driver Station");

		// |  |  |
		// \/ \/ \/
		number = networkTable.getIntegerTopic("other number").getEntry(0);
		number.set(TestNumber.number);
		maxClimberHeight = networkTable.getDoubleTopic("max climber height").getEntry(0);
		maxClimberHeight.set(ArmConstants.MAX_HEIGHT);
		// /\ /\ /\
		//  |  |  |
	}

	@Override
	public void periodic() {
		TestNumber.number = (int) number.get();
		ArmConstants.MAX_HEIGHT = maxClimberHeight.get();
	}
}
