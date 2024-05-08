// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Head;

public class RobotStatus  {

	private final DigitalInput brakeToggleButton = new DigitalInput(Constants.BRAKE_TOGGLE_BUTTON_DIO);

  /** Creates a new RobotStatus. */
  public RobotStatus(Arm arm, Head head) {

	// Setup toggle button to control brake mode of arm and head mechanisms
	Trigger brakeToggleTrigger = new Trigger(() -> brakeToggleButton.get());
		brakeToggleTrigger.onTrue(arm.ToggleBrakeModes());
		brakeToggleTrigger.onTrue(head.ToggleBreakModes());
	
	// Ensure that when robot is enabled the brake mode for mechniasms is on
	Trigger enableTrigger = new Trigger(() -> DriverStation.isEnabled());
		enableTrigger.onTrue(Commands.runOnce(() -> {
			arm.EnableBrakeMode();
			head.EnableBrakeMode();
		}));
		
  }

  public boolean isMechanismBraked()
  {
	return brakeToggleButton.get();
  }

  public int numberCANErrors()
  {
	return RobotController.getCANStatus().receiveErrorCount + RobotController.getCANStatus().transmitErrorCount;
  }

  public double batteryVoltage()
  {
	return RobotController.getBatteryVoltage();
  }

  public boolean isBrownedOut()
  {
	return RobotController.isBrownedOut();
  }


}
