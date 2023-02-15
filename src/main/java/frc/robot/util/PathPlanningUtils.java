package frc.robot.util;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

public class PathPlanningUtils {
    
     /**
     * Transform a pose to the given alliance (blue alliance is default)
     * 
     * @param pose
     * @param alliance 
     * @return
     */
    public static Pose2d transformPoseForAlliance(Pose2d pose, DriverStation.Alliance alliance) {
        return (alliance == DriverStation.Alliance.Red) 
            ? new Pose2d(new Translation2d(pose.getX(), Constants.fieldWidthMeters - pose.getY()), pose.getRotation().times(-1))
            : pose;
    }

    /**
     * @param scoringNode node to score on
     * @param alliance current alliance
     * @return the PathPoint where the robot must be to score on given node
     */
    public static PathPoint getScoringNodePathPoint(int scoringNode, Alliance alliance) {
        // TODO implement
        return null;
    }
}
