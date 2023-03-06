// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ElevatorArm.Position;
import frc.robot.commands.AutoScoringTrajectoryCommand;
import frc.robot.commands.Autos;
import frc.robot.subsystems.ElevatorArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private final OI oi = new OI(Constants.OI.driverPort, Constants.OI.operatorPort);

  private Vision vision = null;

  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final ElevatorArmSubsystem elevatorArm = new ElevatorArmSubsystem();

  private final Autos autos = new Autos(swerve);

  private Position nextArmPosition;
  private int nextScoringSlot;

  public RobotContainer() {
    swerve.setDefaultCommand(swerve.run(() -> swerve.drive(
            MathUtil.applyDeadband(oi.drive_x.getAsDouble(), Constants.OI.deadband, Constants.Swerve.maxSpeed) * (oi.driveSlow.getAsBoolean() ? Constants.OI.slowSpeed : 1),
            MathUtil.applyDeadband(oi.drive_y.getAsDouble(), Constants.OI.deadband, Constants.Swerve.maxSpeed) * (oi.driveSlow.getAsBoolean() ? Constants.OI.slowSpeed : 1),
            MathUtil.applyDeadband(oi.drive_rot.getAsDouble(), Constants.OI.deadband, Constants.Swerve.maxAngularSpeed) * (oi.driveSlow.getAsBoolean() ? Constants.OI.slowSpeed : 1),
            true,
            true
        )));
    elevatorArm.setDefaultCommand(elevatorArm.run(elevatorArm::stop));

    configureBindings();
  }

  private void configureBindings() {
    //oi.closeIntakeButton.onTrue(intake.closeCommand());
    //oi.openIntakeButton.onTrue(intake.openCommand());

    //TODO - check for cones vs cubes for arm positions

    oi.armToNextTargetPositionButton.onTrue(elevatorArm.goToTargetCommand(nextArmPosition));
    oi.armToShelfIntakePositionButton.onTrue(elevatorArm.goToTargetCommand(Position.SHELF));
    oi.armToGroundIntakePositionButton.onTrue(elevatorArm.goToTargetCommand(Position.L1CONE)); 
    oi.armToHomePositionButton.onTrue(elevatorArm.goToTargetCommand(Position.HOME));

    oi.fullAutoScore.onTrue(
      new AutoScoringTrajectoryCommand(nextScoringSlot, new PathConstraints(4, 3), autos.autoBuilder, swerve)
      .alongWith(elevatorArm.goToTargetCommand(nextArmPosition))
      //.andThen(intake.openCommand())
    );
    oi.driveToScoringNodeButton.onTrue(new AutoScoringTrajectoryCommand(nextScoringSlot, new PathConstraints(4, 3), autos.autoBuilder, swerve)); //TODO determine path constraints
    oi.gyroResetButton.onTrue(swerve.runOnce(swerve::resetGyro));
    oi.interruptButton.onTrue(new InstantCommand(elevatorArm::stop, elevatorArm))
                      .onTrue(new InstantCommand(swerve::stop, swerve));

    oi.targetLvl1ArmPosition.onTrue(new InstantCommand(() -> nextArmPosition = Position.L1CONE));
    oi.targetLvl2ArmPosition.onTrue(new InstantCommand(() -> nextArmPosition = Position.L2CONE));
    oi.targetLvl3ArmPosition.onTrue(new InstantCommand(() -> nextArmPosition = Position.L3CONE));
    
    oi.manualArmButton.whileTrue(elevatorArm.manualMoveCommand(oi.manualArmVerticalPower, oi.manualArmElevatorPower));

    oi.targetSlot1.onTrue(new InstantCommand(() -> nextScoringSlot = 0));
    oi.targetSlot2.onTrue(new InstantCommand(() -> nextScoringSlot = 1));
    oi.targetSlot3.onTrue(new InstantCommand(() -> nextScoringSlot = 2));
    oi.targetSlot4.onTrue(new InstantCommand(() -> nextScoringSlot = 3));
    oi.targetSlot5.onTrue(new InstantCommand(() -> nextScoringSlot = 4));
    oi.targetSlot6.onTrue(new InstantCommand(() -> nextScoringSlot = 5));
    oi.targetSlot7.onTrue(new InstantCommand(() -> nextScoringSlot = 6));
    oi.targetSlot8.onTrue(new InstantCommand(() -> nextScoringSlot = 7));
    oi.targetSlot9.onTrue(new InstantCommand(() -> nextScoringSlot = 8));
  }

  public void initVision() {
    if (vision == null) {
      if (DriverStation.isFMSAttached()) {
        vision = new Vision();
      }
    }
  }

  public void periodic() {
    if (vision != null) {
      swerve.addPotentialVisionMeasurement(vision.getEstimatedGlobalPose(swerve.getPose()));
    }
  }

  public Command getAutonomousCommand() {
    return autos.getSelectedAuto();
  }
}
