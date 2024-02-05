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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
	private final PhotonCamera intakeCamera = new PhotonCamera("intake_camera");
	//private final PhotonCamera shooterCamera = new PhotonCamera("shooter_camera");
	private PhotonPipelineResult intakeCamResult;
	private PhotonPipelineResult shooterCamResult;
	private AprilTagFieldLayout fieldLayout;
	private PhotonPoseEstimator intakeEstimator;
	private PhotonPoseEstimator shooterEstimator;
	private PhotonTrackedTarget intakeBestTag;
	//private Optional<PhotonTrackedTarget> intakeBestTag;
	//private Optional<PhotonTrackedTarget> shooterBestTag;

	/** Creates a new Vision. */
	public Vision() {
		try {
			fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
		} catch (IOException e) {
			fieldLayout = null;
		}
		 intakeEstimator  = new PhotonPoseEstimator(fieldLayout,
			PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, intakeCamera, VisionConstants.INTAKE_CAMERA_POSITION);
		//shooterEstimator =  = new PhotonPoseEstimator(fieldLayout,
		//	PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, shooterCamera, VisionConstants.SHOOTER_CAMERA_POSITION);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		intakeCamResult = intakeCamera.getLatestResult();
		shooterCamResult = new PhotonPipelineResult();//shooterCamera.getLatestResult();
	}

	// public Optional<EstimatedRobotPose> updateOdometry() {
	// 	intakeBestTag = Optional.ofNullable(intakeCamResult.getBestTarget());
	// 	if(intakeBestTag.isPresent()){
	// 		return intakeEstimator.update();
	// 	}
	// 	else{
	// 		return Optional.empty();
	// 	}
	// }
	
	public Optional<EstimatedRobotPose> updateOdometry() {
		intakeBestTag = intakeCamResult.getBestTarget();
		if(intakeBestTag != null){
			//assuming the get distance returns distance in meters
			if(intakeBestTag.getBestCameraToTarget().getTranslation().getNorm() < 2)
				return intakeEstimator.update();
		}
			return Optional.empty();
	}

	public Optional<Transform3d> findTag(int tagNumber) {
		for (PhotonTrackedTarget target : intakeCamResult.getTargets()) {
			if (target.getFiducialId() == tagNumber)
				return Optional
						.of(target.getBestCameraToTarget().plus(VisionConstants.INTAKE_CAMERA_POSITION.inverse()));
		}

		for (PhotonTrackedTarget target : shooterCamResult.getTargets()) {
			if (target.getFiducialId() == tagNumber)
				return Optional
						.of(target.getBestCameraToTarget().plus(VisionConstants.SHOOTER_CAMERA_POSITION.inverse()));
		}
		return Optional.empty();
	}
}
