// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
	private final PhotonCamera intakeCamera = new PhotonCamera("intake_camera");
	private final PhotonCamera shooterCamera = new PhotonCamera("shooter_camera");
	private PhotonPipelineResult intakeCamResult;
	private PhotonPipelineResult shooterCamResult;
	private AprilTagFieldLayout fieldLayout;
	private PhotonPoseEstimator intakeEstimator;
	private PhotonPoseEstimator shooterEstimator;
	private PhotonTrackedTarget intakeBestTag;
	private PhotonTrackedTarget shooterBestTag;

	/** Creates a new Vision. */
	public Vision() {
		try {
			fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
		} catch (IOException e) {
			fieldLayout = null;
		}
		 intakeEstimator  = new PhotonPoseEstimator(fieldLayout,
			PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, intakeCamera, VisionConstants.INTAKE_CAMERA_POSITION);
		shooterEstimator = new PhotonPoseEstimator(fieldLayout,
			PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, shooterCamera, VisionConstants.SHOOTER_CAMERA_POSITION);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		intakeCamResult = intakeCamera.getLatestResult();
		shooterCamResult = shooterCamera.getLatestResult();
	}
	
	public Optional<EstimatedRobotPose> updateOdometry() {
		intakeBestTag = intakeCamResult.getBestTarget();
		shooterBestTag = shooterCamResult.getBestTarget();
		if(intakeBestTag != null){
			if(shooterBestTag != null){
				if(intakeBestTag.getBestCameraToTarget().getTranslation().getNorm() < shooterBestTag.getBestCameraToTarget().getTranslation().getNorm()){
					if((intakeBestTag.getBestCameraToTarget().getTranslation().getNorm() < 4) && intakeBestTag.getPoseAmbiguity() < .2){
						return intakeEstimator.update();}
					else
						return Optional.empty();
				}
				else{
					if(shooterBestTag.getBestCameraToTarget().getTranslation().getNorm() < 4 && shooterBestTag.getPoseAmbiguity() < .2){
						return shooterEstimator.update();
					}
					else
						return Optional.empty();
				}
			}
			if(intakeBestTag.getBestCameraToTarget().getTranslation().getNorm() < 4){
				return intakeEstimator.update();}
		}
		else if(shooterBestTag != null){
			if(shooterBestTag.getBestCameraToTarget().getTranslation().getNorm() < 4){
				return shooterEstimator.update();}
			}
			return Optional.empty();
	}
}
