package frc.robot;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class Vision {

    private PhotonCamera camera;
    private PhotonPoseEstimator poseEstimator;

    private static final Transform3d robotToCamera = new Transform3d(
                        new Translation3d(0.5, 0.0, 0.5),
                        new Rotation3d(0, 0,0)); //TODO input values

    public Vision() {
        AprilTagFieldLayout aprilTagFieldLayout = null;
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (Exception e) {
            //TODO: handle exception
        }

        // Forward Camera
        camera = new PhotonCamera(Constants.Vision.cameraName);

        poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotToCamera);
    }

     /**
     * @param estimatedRobotPose The current best guess at robot pose
     * @return A pair of the fused camera observations to a single Pose2d on the field, and the time
     *     of the observation. Assumes a planar field and the robot is always firmly on the ground
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {

        poseEstimator.setReferencePose(prevEstimatedRobotPose); // This is only necessary if strategy is CLOSEST_TO_REFERENCE_POSE
        
        return poseEstimator.update();
    }
    
}
