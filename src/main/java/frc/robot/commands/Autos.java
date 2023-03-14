package frc.robot.commands;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorArm.Position;
import frc.robot.subsystems.ElevatorArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

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
        eventMap.put("armToLevel3Cone", arm.goToTargetCommand(Position.L3CONE));
        eventMap.put("armToLevel3Cube", arm.goToTargetCommand(Position.L3CUBE));
        eventMap.put("resetArm", arm.goToTargetCommand(Position.HOME));
        eventMap.put("scoreLevel1Cone", arm.goToTargetCommand(Position.L1CONE).andThen(intake.openCommand()));
        eventMap.put("scoreLevel1Cube", arm.goToTargetCommand(Position.L1CUBE).andThen(intake.openCommand()));
        eventMap.put("scoreLevel2Cone", arm.goToTargetCommand(Position.L2CONE).andThen(intake.openCommand()));
        eventMap.put("scoreLevel2Cube", arm.goToTargetCommand(Position.L2CUBE).andThen(intake.openCommand()));
        eventMap.put("scoreLevel3Cone", arm.goToTargetCommand(Position.L3CUBE).andThen(intake.openCommand()));
        eventMap.put("scoreLevel3Cube", arm.goToTargetCommand(Position.L3CUBE).andThen(intake.openCommand()));
        eventMap.put("intakeCone", intake.closeCommand());
        eventMap.put("intakeCube", intake.closeCommand());
        eventMap.put("autoBalance", DriveCommands.autoBalance(swerve));

        autoBuilder = new SwerveAutoBuilder(
            swerve::getPose, // Pose2d supplier
            swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
            new PIDConstants(Constants.Auto.TranslationPID.P, Constants.Auto.TranslationPID.I, Constants.Auto.TranslationPID.D), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
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
            swerve.runOnce(() -> swerve.resetOdometry(new Pose2d())), 
            arm.goToTargetCommand(Position.L3CUBE),
            intake.openCommand(),
            race(
                parallel(
                    swerve.run(() -> swerve.drive(0.2, 0, 0, true, true))),
                    arm.goToTargetCommand(Position.HOME),
                    intake.closeCommand()
                ),
                new WaitCommand(5)
            );
    }

    public Command getSelectedAuto() {
        return autoChooser.getSelected();
    }
}
