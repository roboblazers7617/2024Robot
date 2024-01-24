// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.LLColor;
import frc.LedStrip;
import frc.PredefinedColors;
import frc.Animations.RaceAnimation;
import frc.Animations.BlinkAnimation;
import frc.Animations.BounceAnimation;
import frc.Animations.SolidColorPattern;

public class LED extends SubsystemBase {
	private final LedStrip strip = new LedStrip(1, 180);
	private int currentMode = -1;

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		if (!DriverStation.isDSAttached() && currentMode != 0) {
			setDisconnectedAnimation();
			currentMode = 0;
		} else if (DriverStation.isEStopped() && DriverStation.isDSAttached() && currentMode != 1) {
			setEStopAnimation();
			currentMode = 1;
		} else if (DriverStation.isDisabled() && DriverStation.isDSAttached() && currentMode != 2) {
			setIdleAnimation();
			currentMode = 2;
		} else if (DriverStation.isAutonomousEnabled() && DriverStation.isDSAttached() && currentMode != 3) {
			setAutoColor();
			currentMode = 3;
		} else if (DriverStation.isTeleopEnabled() && DriverStation.isDSAttached() && currentMode != 4) {
			setTeleopColor();
			currentMode = 4;
		}
	}

	public void setDisconnectedAnimation() {
		strip.setAnimation(new BlinkAnimation(strip, PredefinedColors.kBlack, PredefinedColors.kBlue));
	}
	
	public void setEStopAnimation() {
		strip.setAnimation(new BounceAnimation(strip, PredefinedColors.kRed, PredefinedColors.kOrange, 10));
	}
	
	public void setIdleAnimation() {
		strip.setAnimation(new BounceAnimation(strip, strip.getAllianceColor(), PredefinedColors.kWhite, 10));
	}

	public void setAutoColor() {
		strip.setAnimation(new SolidColorPattern(strip, strip.getAllianceColor()));
	}

	public void setTeleopColor() {
		strip.setAnimation(new SolidColorPattern(strip, PredefinedColors.kYellow));
	}
}
