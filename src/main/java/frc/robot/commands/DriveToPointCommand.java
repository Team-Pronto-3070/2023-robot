package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToPointCommand extends CommandBase {

    protected final List<PathPoint> points;
    private final PathConstraints constraints;
    private final SwerveAutoBuilder autoBuilder;
    private final SwerveSubsystem swerve;

    private Command pathFollowingCommand;


    /**
     * Constructs a DriveToPoint command that will create and follow a trajectory 
     * starting from the robot's current pose
     * 
     * @param points a list of PathPoints that the robot will pass through, excluding the starting point
     * @param constraints PathConstraints for the trajectory (maximum velocity and maximum acceleration)
     * @param autoBuilder the autobuilder that will be used to follow the trajectory
     * @param swerve the swerve subsystem
     */
    public DriveToPointCommand(List<PathPoint> points, PathConstraints constraints, SwerveAutoBuilder autoBuilder, SwerveSubsystem swerve) {
        this.points = points;
        this.constraints = constraints;
        this.autoBuilder = autoBuilder;
        this.swerve = swerve;

        addRequirements(swerve);
    }

    public DriveToPointCommand(PathPoint endPoint, PathConstraints constraints, SwerveAutoBuilder autoBuilder, SwerveSubsystem swerve) {
        this(List.of(endPoint), constraints, autoBuilder, swerve);
    }

    @Override
    public void initialize() {
        Pose2d startPose = swerve.getPose();
        ChassisSpeeds startSpeed = swerve.getChassisSpeeds();

        points.add(
            0,
            new PathPoint(
                startPose.getTranslation(),
                new Rotation2d(startSpeed.vxMetersPerSecond, startSpeed.vyMetersPerSecond).plus(swerve.getYaw()),
                startPose.getRotation(),
                Math.hypot(startSpeed.vxMetersPerSecond, startSpeed.vyMetersPerSecond)
            )
        );

        pathFollowingCommand = autoBuilder.followPath(PathPlanner.generatePath(constraints, points));
        pathFollowingCommand.initialize();
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
