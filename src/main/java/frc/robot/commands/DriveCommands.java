package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.SwerveModuleState2;

public class DriveCommands extends CommandBase{

    /**
     * Command for driving onto charge station and balancing using PID control
     * 
     * Assumes robot is in position to ascend charge station by driving in x direction only
     * and is in the desired orientation for its ascent
     * 
     * @param swerve the swerve subsystem
     */
    private static Command autoBalancePID(SwerveSubsystem swerve) {

        return sequence(
            // drive onto ramp
            parallel(
                swerve.run(() -> swerve.drive(Constants.DriveCommands.AutoBalance.driveUpRampSpeed * ((swerve.getPose().getX() < 4) ? 1 : -1), 0, 0, true, false)),
                new WaitUntilCommand(() -> swerve.getPitch() >= Constants.DriveCommands.AutoBalance.onRampAngle)
            ),

            // balance
            new PIDCommand(
                new PIDController(Constants.DriveCommands.AutoBalance.PID.P, Constants.DriveCommands.AutoBalance.PID.I, Constants.DriveCommands.AutoBalance.PID.D), 
                swerve::getPitch,
                Constants.DriveCommands.AutoBalance.balanceSetpoint, 
                (output) -> swerve.drive(
                    MathUtil.clamp(-output, -Constants.DriveCommands.AutoBalance.driveUpRampSpeed, Constants.DriveCommands.AutoBalance.driveUpRampSpeed) * ((swerve.getPose().getX() < 4) ? 1 : -1), 
                    0, 
                    0, 
                    true, 
                    false), 
                swerve));
    }

    /**
     * Command for driving onto the charge station and balancing by driving up charge station at a static speed and stopping
     * 
     * Assumes robot is in position to ascend charge station by driving in x direction only
     * and is in the desired orientation for its ascent
     * 
     * @param swerve the swerve subsystem
     * @return
     */
    private static Command autoBalanceBangBang(SwerveSubsystem swerve) {
        SwerveModuleState2[] stopStates = {
            new SwerveModuleState2(0, new Rotation2d(Units.degreesToRadians(45)), 0), // front left
            new SwerveModuleState2(0, new Rotation2d(Units.degreesToRadians(-45)), 0), // front right
            new SwerveModuleState2(0, new Rotation2d(Units.degreesToRadians(-45)), 0), // rear left
            new SwerveModuleState2(0, new Rotation2d(Units.degreesToRadians(45)), 0), // rear right
        };

        return sequence(
            parallel(
                print("start auto balance"),
                //swerve.run(swerve::stop).asProxy(),
                swerve.run(
                    //() -> swerve.drive(Constants.DriveCommands.AutoBalance.driveUpRampSpeed * ((swerve.getPose().getX() < 4) ? 1 : -1), 0, 0, true, false)),
                    () -> swerve.drive(-0.2, 0, 0, true, false)),
                sequence(
                    new WaitUntilCommand(() -> swerve.getPitch() > Constants.DriveCommands.AutoBalance.onRampAngle),
                    new WaitUntilCommand(() -> swerve.getPitch() < Constants.DriveCommands.AutoBalance.stopAngle)
                )
            ).withTimeout(5),
            swerve.run(() -> swerve.setModuleStates(stopStates))
        );
    }

    private static CommandBase autoBalance2(SwerveSubsystem swerve) {
        return sequence(
            new InstantCommand(() -> SmartDashboard.putNumber("auto state", 6)),
            swerve.run(() -> swerve.drive(-2.0, 0, 0, true, false)).withTimeout(2),
            driveToAngleCommand(swerve, -2.0, 18, true),
            new InstantCommand(() -> SmartDashboard.putNumber("auto state", 7)),
            driveToAngleCommand(swerve, -0.6, 14.0, false),
            new InstantCommand(() -> SmartDashboard.putNumber("auto state", 8)),
            //waitSeconds(1),
            //driveToAngleCommand(swerve, 0.6, -2, true),
            new InstantCommand(() -> SmartDashboard.putNumber("auto state", 9)),
            swerve.runOnce(() -> swerve.setModuleStates(new SwerveModuleState2[] {
                new SwerveModuleState2(0, new Rotation2d(Units.degreesToRadians(45)), 0), // front left
                new SwerveModuleState2(0, new Rotation2d(Units.degreesToRadians(-45)), 0), // front right
                new SwerveModuleState2(0, new Rotation2d(Units.degreesToRadians(-45)), 0), // rear left
                new SwerveModuleState2(0, new Rotation2d(Units.degreesToRadians(45)), 0), // rear right
            }))
        ).withTimeout(10);
    }

    public static CommandBase autoBalance(SwerveSubsystem swerve) {
        //return autoBalanceBangBang(swerve);
        //return autoBalancePID(swerve);
        return autoBalance2(swerve);
    }

    public static Command driveToAngleCommand(SwerveSubsystem swerve, double speed, double angle, boolean increasing) {
        Debouncer debounce = new Debouncer(Constants.DriveCommands.AutoBalance.angleDebounceTime);
        return swerve.run(() -> swerve.drive(speed, 0, 0, true, false))
               .until(() -> debounce.calculate((swerve.getPitch() > angle) ^ !increasing));
    }

    private static final ProfiledPIDController xController = new ProfiledPIDController(Constants.Auto.TranslationPID.P, Constants.Auto.TranslationPID.I, Constants.Auto.TranslationPID.D,
                                                                    new TrapezoidProfile.Constraints(Constants.Auto.maxVelocity, Constants.Auto.maxAcceleration));
    private static final ProfiledPIDController yController = new ProfiledPIDController(Constants.Auto.TranslationPID.P, Constants.Auto.TranslationPID.I, Constants.Auto.TranslationPID.D,
                                                                    new TrapezoidProfile.Constraints(Constants.Auto.maxVelocity, Constants.Auto.maxAcceleration));
    private static final ProfiledPIDController thetaController = new ProfiledPIDController(Constants.Auto.RotationPID.P, Constants.Auto.RotationPID.I, Constants.Auto.RotationPID.D,
                                                                    new TrapezoidProfile.Constraints(1.0, 1.0));
    static {
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public static Command driveToPointPIDCommand(SwerveSubsystem swerve, Pose2d point) {
        return new InstantCommand(() -> {
                        Pose2d curPose = swerve.getPose();
                        ChassisSpeeds curSpeed = swerve.getChassisSpeeds();
                        xController.reset(new TrapezoidProfile.State(curPose.getX(), curSpeed.vxMetersPerSecond));
                        yController.reset(new TrapezoidProfile.State(curPose.getY(), curSpeed.vyMetersPerSecond));
                        thetaController.reset(new TrapezoidProfile.State(curPose.getRotation().getRadians(), curSpeed.omegaRadiansPerSecond));
                        xController.setGoal(point.getX());
                        yController.setGoal(point.getY());
                        thetaController.setGoal(point.getRotation().getRadians());
                    })
                .andThen(
                    swerve.run(() -> 
                        swerve.drive(
                            xController.calculate(swerve.getPose().getX()),
                            yController.calculate(swerve.getPose().getY()),
                            thetaController.calculate(swerve.getPose().getRotation().getRadians()),
                            true,
                            false
                        )
                    )
                );
    }
}
