package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.PathPlanningUtils;

public class AutoScoringTrajectoryCommand extends DriveToPointCommand {

    private static final Pose2d WALL_SIDE_MIDPOINT = new Pose2d(new Translation2d(2.32, 0.084), new Rotation2d(-0.593412));
    private static final Pose2d LOADING_ZONE_MIDPOINT = new Pose2d(new Translation2d(2.32, 4.53), new Rotation2d(0.558505));

    private int scoringNode;

    /**
     * Constructs an AutoScoringTrajectoryCommand that will create and follow a trajectory 
     * bringing the robot to the given scoring node
     * 
     * @param scoringNode target node
     * @param constraints PathConstraints for the trajectory (maximum velocity and maximum acceleration)
     * @param autoBuilder the autobuilder that will be used to follow the trajectory
     * @param swerve the swerve subsystem
     */
    public AutoScoringTrajectoryCommand(int scoringNode, PathConstraints constraints, SwerveAutoBuilder autoBuilder, SwerveSubsystem swerve) {
        super(PathPlanningUtils.getScoringNodePathPoint(scoringNode, DriverStation.getAlliance()), constraints, autoBuilder, swerve);
        this.scoringNode = scoringNode;
    }

    @Override
    public void initialize() {
        Pose2d currentPose = swerve.getPose();
        double x = currentPose.getX();
        double y = currentPose.getY();

        Pose2d midPoint = null;
        Alliance alliance = DriverStation.getAlliance();

        if (x > 2.416 && x < 4.909) { // check y position on field
            if (alliance == DriverStation.Alliance.Blue) {
                if ( y > 0 && y < 1.509) {
                    // wall side
                    if (scoringNode % 9 < 7) {
                        midPoint = WALL_SIDE_MIDPOINT;
                    }
                } else if (y > 4.035 && y < 5.353) {
                    // loading zone side
                    if (scoringNode % 9 > 1) {
                        midPoint = LOADING_ZONE_MIDPOINT;
                    }
                }
            } else { // Red
                if (y > Constants.fieldWidthMeters && y < Constants.fieldWidthMeters - 1.509) {
                    // wall side
                    if (scoringNode % 9 > 1) {
                        midPoint = WALL_SIDE_MIDPOINT;
                    }
                } else if (y > Constants.fieldWidthMeters - 4.035 && y < Constants.fieldWidthMeters - 5.353) {
                    // loading zone side
                    if (scoringNode % 9 < 7) {
                        midPoint = LOADING_ZONE_MIDPOINT;
                    }
                }
            }
        }
        
        if (midPoint != null) {
            Pose2d transformedPose = PathPlanningUtils.transformPoseForAlliance(midPoint, alliance);
            points.add(0, new PathPoint(transformedPose.getTranslation(), transformedPose.getRotation()));
        }

        super.initialize();
    }
}
