package frc.robot;

import java.io.IOException;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;

public class Vision {

    private PhotonCamera camera;
    private PhotonPoseEstimator poseEstimator;

    private boolean is_enabled = true;
    private boolean can_enable = true;

    public Vision() {
        AprilTagFieldLayout aprilTagFieldLayout = null;
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            permenantlyDisable();
        }

        camera = new PhotonCamera(Constants.Vision.cameraName);
        poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, Constants.Vision.robotToCamera);
    }

     /**
     * @param estimatedRobotPose The current best guess at robot pose
     * @return A pair of the fused camera observations to a single Pose2d on the field, and the time
     *     of the observation. Assumes a planar field and the robot is always firmly on the ground
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (!is_enabled) {
            return Optional.empty();
        }
        poseEstimator.setReferencePose(prevEstimatedRobotPose);
        return poseEstimator.update();
    }

    public void enable() {
        if (can_enable) {
            is_enabled = true;
        }
    }

    //might want to do this while the robot is going up on the charge station so vision doesn't get confused
    public void disable() {
        is_enabled = false;
    }
    
    //use this to permantly disable vision if something goes completely wrong
    public void permenantlyDisable() {
        disable();
        can_enable = false;
    }
}
