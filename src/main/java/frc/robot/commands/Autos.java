package frc.robot.commands;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorArm.Position;
import frc.robot.subsystems.ElevatorArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import static frc.robot.commands.DriveCommands.driveToAngleCommand;

public class Autos {
    public final SwerveAutoBuilder autoBuilder;
    private final SendableChooser<Command> autoChooser;

    public Autos(SwerveSubsystem swerve, ElevatorArmSubsystem arm, IntakeSubsystem intake) {

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("armToGroundIntake", arm.goToTargetCommand(Position.L1CONE)); // TODO - does cone vs cube matter?
        eventMap.put("armToShelfIntake", arm.goToTargetCommand(Position.SHELF));
        eventMap.put("armToLevel1Cone", arm.goToTargetCommand(Position.L1CONE));
        eventMap.put("armToLevel1Cube", arm.goToTargetCommand(Position.L1CUBE));
        eventMap.put("armToLevel2Cone", arm.goToTargetCommand(Position.L2CONE));
        eventMap.put("armToLevel2Cube", arm.goToTargetCommand(Position.L2CUBE));
        //eventMap.put("armToLevel3Cone", arm.goToTargetCommand(Position.L3CONE));
        eventMap.put("armToLevel3Cone", arm.goToTargetCommand(Position.AUTOL3CONE));
        //eventMap.put("armToLevel3Cube", arm.goToTargetCommand(Position.L3CUBE));
        eventMap.put("armToLevel3Cube", arm.goToTargetCommand(Position.AUTOL3CUBE));
        eventMap.put("resetArm", arm.goToTargetCommand(Position.HOME));
        eventMap.put("scoreLevel1Cone", arm.goToTargetCommand(Position.L1CONE).andThen(intake.openCommand().withTimeout(1.0)));
        eventMap.put("scoreLevel1Cube", arm.goToTargetCommand(Position.L1CUBE).andThen(intake.openCommand().withTimeout(1.0)));
        eventMap.put("scoreLevel2Cone", arm.goToTargetCommand(Position.L2CONE).andThen(intake.openCommand().withTimeout(1.0)));
        eventMap.put("scoreLevel2Cube", arm.goToTargetCommand(Position.L2CUBE).andThen(intake.openCommand().withTimeout(1.0)));
        eventMap.put("scoreLevel3Cone", arm.goToTargetCommand(Position.AUTOL3CONE).andThen(arm.goToTargetCommand(Position.L3CONE)).andThen(intake.openCommand().withTimeout(1.0)));
        eventMap.put("scoreLevel3Cube", arm.goToTargetCommand(Position.AUTOL3CUBE).andThen(arm.goToTargetCommand(Position.L3CUBE)).andThen(intake.openCommand().withTimeout(1.0)));
        eventMap.put("intakeCone", intake.closeCommand());
        eventMap.put("intakeCube", intake.closeCommand());
        eventMap.put("autoBalance", DriveCommands.autoBalance(swerve));

        autoBuilder = new SwerveAutoBuilder(
            swerve::getPose, // Pose2d supplier
            swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
            new PIDConstants(Constants.Auto.TranslationPID.P, Constants.Auto.TranslationPID.I, Constants.Auto.TranslationPID.D), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(Constants.Auto.RotationPID.P, Constants.Auto.RotationPID.I, Constants.Auto.RotationPID.D), // PID constants to correct for rotation error (used to create the rotation controller)
            swerve::setChassisSpeeds, // chassis speeds consumer used to output to the drive subsystem
            eventMap,
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            swerve // The drive subsystem. Used to properly set the requirements of path following commands
        );

        autoChooser = new SendableChooser<Command>();
        autoChooser.setDefaultOption("none", new PrintCommand("no auto option selected"));
        autoChooser.addOption("test1", buildAuto("test1"));
        autoChooser.addOption("0 Piece Taxi Balance", buildAuto("0 Piece Taxi Balance"));
        autoChooser.addOption("1 Piece Taxi LZ", buildAuto("1 Piece Taxi LZ"));
        autoChooser.addOption("2 Piece Balance LZ", buildAuto("2 Piece Balance LZ"));
        autoChooser.addOption("2 Piece No Balance LZ", buildAuto("2 Piece No Balance LZ"));
        autoChooser.addOption("3 Piece No Balance LZ", buildAuto("3 Piece No Balance LZ"));
        autoChooser.addOption("1 Piece Taxi WALL", buildAuto("1 Piece Taxi WALL"));
        autoChooser.addOption("2 Piece Balance WALL", buildAuto("2 Piece Balance WALL"));
        autoChooser.addOption("2 Piece No Balance WALL", buildAuto("2 Piece No Balance WALL"));
        autoChooser.addOption("3 Piece No Balance WALL", buildAuto("3 Piece No Balance WALL"));
        autoChooser.addOption("1 Piece Taxi Balance CENTER", centerWithBalance(swerve, arm, intake));
        autoChooser.addOption("1 Piece Taxi No Balance CENTER", centerNoBalance(swerve, arm, intake));

        SmartDashboard.putData("auto chooser", autoChooser);
        PathPlannerServer.startServer(5811);
    }

    public Command buildAuto(String name) {
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup(name, new PathConstraints(Constants.Auto.maxVelocity, Constants.Auto.maxAcceleration)));
    }

    /**
     * Returns the autonomous command for when the robot starts in front of the center cube node 
     * Includes balancing on the charge station
     */
    public static Command centerWithBalance(SwerveSubsystem swerve, ElevatorArmSubsystem arm, IntakeSubsystem intake) {
        // NOTE: Could line up w/DriveToPointCommand before climbing to guarantee orientation
        return centerNoBalance(swerve, arm, intake).andThen(DriveCommands.autoBalance(swerve));
    }

    /**
     * Returns the autonomous command for when the robot starts in front of the center cube node 
     */
    public static Command centerNoBalance(SwerveSubsystem swerve, ElevatorArmSubsystem arm, IntakeSubsystem intake) {
        return sequence(
            swerve.runOnce(() -> swerve.resetOdometry(new Pose2d(1.81, 3.28, Rotation2d.fromDegrees(-180.0)))), 
            arm.goToTargetCommand(Position.AUTOL3CONE).withTimeout(5),
            arm.goToTargetCommand(Position.L3CONE).withTimeout(5),
            intake.openCommand().withTimeout(3),
            parallel(
                arm.goToTargetCommand(Position.HOME).withTimeout(4),
                intake.closeCommand(),
                sequence(
                    new InstantCommand(() -> SmartDashboard.putNumber("auto state", 1)),
                    driveToAngleCommand(swerve, 2.0, -12, false),
                    new InstantCommand(() -> SmartDashboard.putNumber("auto state", 2)),
                    driveToAngleCommand(swerve, 1.0, 12, true),
                    new InstantCommand(() -> SmartDashboard.putNumber("auto state", 3)),
                    driveToAngleCommand(swerve, 1.0, 1, false),
                    new InstantCommand(() -> SmartDashboard.putNumber("auto state", 4)),
                    swerve.run(() -> swerve.drive(1.0, 0, 0, true, false)).withTimeout(0.2),
                    new InstantCommand(() -> SmartDashboard.putNumber("auto state", 5)),
                    swerve.runOnce(swerve::stop),
                    waitSeconds(0.5)
                )
            ).withTimeout(10)
        );
    }

    public Command getSelectedAuto() {
        return autoChooser.getSelected();
    }
}
