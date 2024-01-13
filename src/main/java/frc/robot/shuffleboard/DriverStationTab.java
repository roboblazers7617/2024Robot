// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shuffleboard;


import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;


/** Add your docs here. */
public class DriverStationTab extends ShuffleboardTabBase {
	private int number = 0;
	private final IntegerPublisher numPublisher;

    public DriverStationTab() {
		NetworkTableInstance inst = NetworkTableInstance.getDefault();

        NetworkTable networkTable = inst.getTable("Shuffleboard/Driver Station");

		numPublisher = networkTable.getIntegerTopic("number").publish();
    }

    @Override
    public void update() {
		number += 1;
		numPublisher.set(number);
		
		
    }

	@Override
	public void activateShuffleboard(){
		
		//this should be called immediately
	}
}
