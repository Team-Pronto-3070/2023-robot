package frc.robot.util;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

public class PathPlanningUtils {
    
     /**
     * Transform a pose to the given alliance (blue alliance is default)
     * 
     * @param pose pose to transform
     * @param alliance current alliance
     * @return the Pose2d in the coordinates of the given alliance
     */
    public static Pose2d transformPoseForAlliance(Pose2d pose, DriverStation.Alliance alliance) {
        return (alliance == DriverStation.Alliance.Red) 
            ? new Pose2d(transformTranslationForAlliance(pose.getTranslation(), alliance), pose.getRotation().times(-1))
            : pose;
    }

    /**
     * Transform a translation to the given alliance (blue alliance is default)
     * 
     * @param translation translation to transform
     * @param alliance current alliance
     * @return the Translation2d in the coordinates of the given alliance
     */
    public static Translation2d transformTranslationForAlliance(Translation2d translation, DriverStation.Alliance alliance) {
        return (alliance == DriverStation.Alliance.Red) 
            ? new Translation2d(translation.getX(), Constants.fieldWidthMeters - translation.getY())
            : translation;
    }

    /**
     * Gets the PathPoint for the robot to score on the target node
     * 
     * @param scoringNode node to score on
     * @param alliance current alliance
     * @return the PathPoint where the robot must be to score on given node
     */
    public static PathPoint getScoringNodePathPoint(int scoringNode, Alliance alliance) {
        return new PathPoint(
            getScoringNodeTranslation(scoringNode, alliance).plus(new Translation2d(Constants.robotLengthWithBumpers / 2, 0)), 
            new Rotation2d(Math.PI), 
            new Rotation2d(Math.PI));
    }

    /**
     * Gets the translation of the given node
     * 
     * @param scoringNode node to score on
     * @param alliance current alliance
     * @return the position of the node
     */
    public static Translation2d getScoringNodeTranslation(int scoringNode, Alliance alliance) {
        int slot = scoringNode % 9;
        int level = scoringNode / 9;

        return (alliance == Alliance.Blue)
            ? Grid.scoringNodeTranslations[level][slot]
            : transformTranslationForAlliance(Grid.scoringNodeTranslations[level][8 - slot], alliance);
    }

    private static class Grid {
        // X Layout
        private static final double outerX = Units.inchesToMeters(54.25);
        private static final double lowX = outerX - (Units.inchesToMeters(14.25) / 2.0); // Centered when under cube nodes
        private static final double midX = outerX - Units.inchesToMeters(22.75);
        private static final double highX = outerX - Units.inchesToMeters(39.75);

        // Y layout
        private static final int nodeRowCount = 9;
        private static final double nodeFirstY = Units.inchesToMeters(20.19);
        private static final double nodeSeparationY = Units.inchesToMeters(22.0);

        
        // nodes as translations from blue alliance origin
        // indexes are low to high, left to right
        private static final Translation2d[][] scoringNodeTranslations = {new Translation2d[nodeRowCount], new Translation2d[nodeRowCount], new Translation2d[nodeRowCount]};

        static {
            for (int i = 0; i < nodeRowCount; i++) {
                scoringNodeTranslations[0][8-i] = new Translation2d(lowX, nodeFirstY + nodeSeparationY * i);
                scoringNodeTranslations[1][8-i] = new Translation2d(midX, nodeFirstY + nodeSeparationY * i);
                scoringNodeTranslations[2][8-i] = new Translation2d(highX, nodeFirstY + nodeSeparationY * i);
            }
        }
    }
}
