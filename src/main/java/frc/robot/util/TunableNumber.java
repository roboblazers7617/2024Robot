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
	// private final DoubleEntry armKS;
	// private final DoubleEntry armKG;
	// private final DoubleEntry armKV;
	private final DoubleEntry armKP;
	private final DoubleEntry armKI;
	private final DoubleEntry armKD;
	public TunableNumber(){
		NetworkTableInstance inst = NetworkTableInstance.getDefault();
		NetworkTable networkTable = inst.getTable("Shuffleboard/Driver Station");

		// |  |  |
		// \/ \/ \/
		number = networkTable.getIntegerTopic("other number").getEntry(0);
		number.set(TestNumber.number);
		maxClimberHeight = networkTable.getDoubleTopic("max climber height").getEntry(0);
		maxClimberHeight.set(ArmConstants.MAX_ANGLE);
		// armKS = networkTable.getDoubleTopic("arm KS").getEntry(0);
		// armKS.set(ArmConstants.KP);
		// armKG = networkTable.getDoubleTopic("arm KG").getEntry(0);
		// armKG.set(ArmConstants.KP);
		// armKV = networkTable.getDoubleTopic("arm KV").getEntry(0);
		// armKV.set(ArmConstants.KP);
		armKP = networkTable.getDoubleTopic("arm KP").getEntry(0);
		armKP.set(ArmConstants.KP);
		armKI = networkTable.getDoubleTopic("arm KI").getEntry(0);
		armKI.set(ArmConstants.KI);
		armKD = networkTable.getDoubleTopic("arm KD").getEntry(0);
		armKD.set(ArmConstants.KD);
		// /\ /\ /\
		//  |  |  |
	}

	@Override
	public void periodic() {
		//TODO: (Brandon) Let's talk through the TunableNumbers code to see if it can be made more efficient. 
		TestNumber.number = (int) number.get();
		ArmConstants.MAX_ANGLE = maxClimberHeight.get();
		// ArmConstants.KS = armKS.get();
		// ArmConstants.KG = armKG.get();
		// ArmConstants.KV = armKV.get();
		ArmConstants.KP = armKP.get();
		ArmConstants.KI = armKI.get();
		ArmConstants.KD = armKD.get();

	}
}
