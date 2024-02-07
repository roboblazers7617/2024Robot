// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Drivetrain;
import java.util.List;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/**
 * An example command that uses an example subsystem.
 */
public class AbsoluteDriveAngularRotation extends Command {

	private final Drivetrain swerve;
	private final DoubleSupplier vX, vY, vTheta;
	private ChassisSpeeds speeds;
	private Translation2d translation;
	private final SwerveController controller;

	/**
	 * Used to drive a swerve robot in full field-centric mode. vX and vY supply
	 * translation inputs, where x is
	 * torwards/away from alliance wall and y is left/right. headingHorzontal and
	 * headingVertical are the Cartesian
	 * coordinates from which the robot's angle will be derived— they will be
	 * converted to a polar angle, which the robot
	 * will rotate to.
	 *
	 * @param swerve The swerve drivebase subsystem.
	 * @param vX     DoubleSupplier that supplies the x-translation
	 *               joystick input. Should be in the range -1
	 *               to 1 with deadband already accounted for. Positive X
	 *               is away from the alliance wall.
	 * @param vY     DoubleSupplier that supplies the y-translation
	 *               joystick input. Should be in the range -1
	 *               to 1 with deadband already accounted for. Positive Y
	 *               is towards the left wall when
	 *               looking through the driver station glass.
	 * @param vTheta The rotation speed.
	 */
	public AbsoluteDriveAngularRotation(Drivetrain swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier vTheta) {
		this.swerve = swerve;
		this.vX = vX;
		this.vY = vY;
		this.vTheta = vTheta;
		this.controller = swerve.getSwerveController();
		addRequirements(swerve);
	}

	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		speeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(), vTheta.getAsDouble());

		translation = SwerveController.getTranslation2d(speeds);

		translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
				SwerveConstants.LOOP_TIME, SwerveConstants.ROBOT_MASS, List.of(SwerveConstants.DRIVEBASE),
				swerve.getSwerveDriveConfiguration());
		
		swerve.drive(translation, speeds.omegaRadiansPerSecond, true);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}

}
