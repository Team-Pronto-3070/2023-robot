package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoScoringTrajectoryCommand extends DriveToPointCommand {

    private DriveToPointCommand pathFollowingCommand;
    private SwerveSubsystem swerve;
    private Pose2d currentPose;

    public AutoScoringTrajectoryCommand(PathPoint endPoint, PathConstraints constraints, SwerveAutoBuilder autoBuilder, SwerveSubsystem swerve) {
        super(endPoint, constraints, autoBuilder, swerve);
    }

    @Override
    public void initialize() {
        currentPose = swerve.getPose();
        double x = currentPose.getX();
        double y = currentPose.getY();

        // TODO: doesnt work for first 2 nodes

        if (x > 2.416 && x < 4.909 && y > 0 && y < 1.509) {
            // wall side of blue community
            points.add(0, new PathPoint(new Translation2d(2.32, 0.084), new Rotation2d(-0.593412)));
        }
        if (x > 2.416 && x < 4.909 && y > 4.035 && y < 5.353) {

            // loading zone side of blue community
            points.add(0, new PathPoint(new Translation2d(2.32, 4.53), new Rotation2d(0.558505)));
        }

        super.initialize();
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
