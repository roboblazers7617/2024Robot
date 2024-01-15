// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Drivetrain;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;
import frc.robot.Constants.SwerveConstants;
/**This is a state for mannual control of the robot,
 * where one joystick controls the the field-relative translation and the other controls the absolute heading.
 */

public class AbsoluteDriveState extends Command {
  /** Creates a new AbsoluteDriveState. */
  private final Drivetrain swerveDrive;
  private final Supplier<Double> vX, vY, headingHorizontal, headingVertical;
  private Trigger snapOut, snapIn, snapLeft, snapRight;
  private double snapY, snapX;
  private ChassisSpeeds speeds;
  private Translation2d translation;
  /**
   * @param swerveDrive The swerve drive to control.
   * @param vX The field-relative X velocity.
   * @param vY The field-relative Y velocity.
   * @param headingHorizontal The X component of the cartisian representation of the angle to face.
   * @param headingVertical The Y component of the cartisian representation of the angle to face.
   */
  public AbsoluteDriveState(Drivetrain swerveDrive, Supplier<Double> vX, Supplier<Double> vY, 
  Supplier<Double> headingHorizontal, Supplier<Double> headingVertical) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
    this.swerveDrive = swerveDrive;
    this.vX = vX;
    this.vY = vY;
    this.headingHorizontal = headingHorizontal;
    this.headingVertical = headingVertical;

    snapOut = new Trigger(()->false);
    snapIn = new Trigger(()->false);
    snapLeft = new Trigger(()->false);
    snapRight = new Trigger(()->false);
  }

  public AbsoluteDriveState(Drivetrain swerveDrive, Supplier<Double> vX, Supplier<Double> vY, Supplier<Double> headingHorizontal, Supplier<Double> headingVertical, 
  Trigger snapOut, Trigger snapIn, Trigger snapLeft, Trigger snapRight) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive); 
    this.swerveDrive = swerveDrive;
    this.vX = vX;
    this.vY = vY;
    this.headingHorizontal = headingHorizontal;
    this.headingVertical = headingVertical;

    this.snapOut = snapOut;
    this.snapIn = snapIn;
    this.snapLeft = snapLeft;
    this.snapRight = snapRight;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveDrive.getSwerveController().lastAngleScalar = swerveDrive.getHeading().getRadians();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(snapOut.getAsBoolean() == false && snapIn.getAsBoolean()==false && snapLeft.getAsBoolean()==false && snapRight.getAsBoolean()==false){
    speeds = swerveDrive.getTargetSpeeds(vX.get(),
    vY.get(),
    headingHorizontal.get(), headingVertical.get());

    translation = SwerveController.getTranslation2d(speeds);

    translation = SwerveMath.limitVelocity(translation, swerveDrive.getFieldVelocity(), swerveDrive.getPose(), 
    SwerveConstants.LOOP_TIME, SwerveConstants.ROBOT_MASS, List.of(SwerveConstants.DRIVEBASE), 
    swerveDrive.getSwerveDriveConfiguration());
    swerveDrive.drive(translation, speeds.omegaRadiansPerSecond, true, false);
  }
  else{
    snapX = 0;
    snapY = 0;
    if(snapOut.getAsBoolean()){
      snapY += 1;
    }
    if(snapIn.getAsBoolean()){
      snapY -= 1;
    }
    if(snapLeft.getAsBoolean()){
      snapX += 1;
    }
    if(snapRight.getAsBoolean()){
      snapX -= 1;
    }

    speeds = swerveDrive.getTargetSpeeds(vX.get(),
    vY.get(),
    snapX, snapY);

    translation = SwerveController.getTranslation2d(speeds);

    translation = SwerveMath.limitVelocity(translation, swerveDrive.getFieldVelocity(), swerveDrive.getPose(), 
    SwerveConstants.LOOP_TIME, SwerveConstants.ROBOT_MASS, List.of(SwerveConstants.DRIVEBASE), 
    swerveDrive.getSwerveDriveConfiguration());
    swerveDrive.drive(translation, speeds.omegaRadiansPerSecond, true, false);
  }


    
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
