// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;

/** Add your docs here. */
public class AutoCreator  {

	private SendableChooser<Alliance> autoAllianceChooser = new SendableChooser<>();
	private ShuffleboardTab tab;
	private ShuffleboardTab tabTest;
	private boolean loadAutos = false;
	private RobotContainer robotContainer;
	private Drivetrain drivetrain;

	public AutoCreator  (RobotContainer robotContainer, Drivetrain drivetrain, String tabName) 
	{
		this.robotContainer = robotContainer;
		this.drivetrain = drivetrain;
		
		Trigger displayTrigger = new Trigger(() -> loadAutos);
		displayTrigger.onTrue(LoadAndDisplayAutos());
			
		tab = Shuffleboard.getTab(tabName);
		tabTest = Shuffleboard.getTab("Test");
		autoAllianceChooser.setDefaultOption("Blue", Alliance.Blue);
		autoAllianceChooser.addOption("Red", Alliance.Red);

		tab.add("Alliance Auto Color", autoAllianceChooser)
			.withSize(2, 1)
			.withPosition(0, 0)
			.withWidget(BuiltInWidgets.kComboBoxChooser);
		tab.add("Load Auto Paths",  ConfigureAutoBuilder().ignoringDisable(true))
			.withSize(2, 1)
			.withPosition(2, 0)
			.withWidget(BuiltInWidgets.kCommand);
	
	}

	private Command LoadAndDisplayAutos(){
		return Commands.runOnce(() ->
			{tab.add("Auto Selector: " + autoAllianceChooser.getSelected().toString(), AutoConstants.AUTO_CHOOSER)
				.withSize(2, 1)
				.withPosition(4, 0)
				.withWidget(BuiltInWidgets.kComboBoxChooser);
				AutoConstants.AUTOS_LOADED = true;
			}).ignoringDisable(true);
	}

	public Command ConfigureAutoBuilder()
	{
			return  Commands.runOnce(() -> {
			robotContainer.configureAutoBuilder(autoAllianceChooser.getSelected());
			System.out.println("***** Color is " + autoAllianceChooser.getSelected().toString());
			this.loadAutos = true;
			});
	}



}
