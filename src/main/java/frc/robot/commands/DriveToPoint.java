package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class DriveToPoint extends CommandBase {

    private final Swerve swerve;

    private Pose2d startPose;
    private final List<Pose2d> midpoints;
    private final Pose2d endPose;
    private final PathConstraints constraints;

    private PathPlannerTrajectory trajectory;
    private PPSwerveControllerCommand pathFollowingCommand;

    private final boolean useAllianceColor;


    /**
     * Constructs a DriveToPoint command that will create and follow a trajectory 
     * starting from the robots current pose, containing the midpoints finishing at the endPose
     * 
     * @param endPose the robots final pose
     * @param midpoints list of poses along the trajectory for the robot to follow
     * @param constraints PathConstraints for the trajectory
     * @param useAllianceColor Should the path states be automatically transformed based on alliance
   *     color? In order for this to work properly, you MUST create your path on the blue side of
   *     the field.
     * @param swerve
     */
    public DriveToPoint(Pose2d endPose, List<Pose2d> midpoints, PathConstraints constraints, boolean useAllianceColor, Swerve swerve) {
        this.swerve = swerve;
        this.midpoints = midpoints;
        this.endPose = endPose;
        this.constraints = constraints;
        this.useAllianceColor = useAllianceColor;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        startPose = swerve.getPose();

        trajectory = useAllianceColor 
                        ? PathPlannerTrajectory.transformTrajectoryForAlliance(getTrajectory(), DriverStation.getAlliance()) 
                        : getTrajectory();
        

        // Could put this in the drive subsystem (ex: swerve.followTrajectory(swerve, trajectory))
        pathFollowingCommand = new PPSwerveControllerCommand(
            trajectory,
            swerve::getPose,
            swerve.kinematics,
            new PIDController(Constants.Auto.TranslationPID.P, Constants.Auto.TranslationPID.I, Constants.Auto.TranslationPID.D),
            new PIDController(Constants.Auto.TranslationPID.P, Constants.Auto.TranslationPID.I, Constants.Auto.TranslationPID.D),
            new PIDController(Constants.Auto.RotationPID.P, Constants.Auto.RotationPID.I, Constants.Auto.RotationPID.D),
            swerve::setModuleStates,
            useAllianceColor,
            swerve);

        pathFollowingCommand.initialize();
    }

    

    /*
     * Returns the trajectory based on startPose, midpoints, and targetPose
     */
    private PathPlannerTrajectory getTrajectory() {
        List<PathPoint> pathPoints = new ArrayList<>();
        pathPoints.add(generatePathPointFromPose(startPose));

        for (Pose2d pose : midpoints) {
            pathPoints.add(generatePathPointFromPose(pose));
        }

        pathPoints.add(generatePathPointFromPose(endPose));

        return PathPlanner.generatePath(constraints, pathPoints);
    }

    /*
     * Turns a Pose2d into a PathPoint
     */
    private static PathPoint generatePathPointFromPose(Pose2d pose) {
        return new PathPoint(pose.getTranslation(), pose.getRotation());
    }


    @Override
    public void execute() {
        pathFollowingCommand.execute();
    }

    @Override
    public boolean isFinished() {
        return pathFollowingCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        pathFollowingCommand.end(interrupted);
    }
    
}
