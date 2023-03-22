// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.Constants.GameObject;
import frc.robot.Constants.ElevatorArm.Position;
import frc.robot.commands.AutoScoringTrajectoryCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.ElevatorArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private final OI oi = new OI(Constants.OI.driverPort, Constants.OI.operatorPort);

  private Vision vision = null;
  private boolean triedVision = false;

  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final ElevatorArmSubsystem elevatorArm = new ElevatorArmSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();

  private final Autos autos = new Autos(swerve, elevatorArm, intake);

  private Position nextArmPosition;
  private int nextScoringSlot;

  public RobotContainer() {
    ///*
    swerve.setDefaultCommand(swerve.run(() -> swerve.drive(
            Math.pow(MathUtil.applyDeadband(oi.drive_x.getAsDouble(), Constants.OI.deadband), 3) * Constants.Swerve.maxSpeed * (oi.driveSlow.getAsBoolean() ? Constants.OI.slowSpeed : 1),
            Math.pow(MathUtil.applyDeadband(oi.drive_y.getAsDouble(), Constants.OI.deadband), 3) * Constants.Swerve.maxSpeed * (oi.driveSlow.getAsBoolean() ? Constants.OI.slowSpeed : 1),
            Math.pow(MathUtil.applyDeadband(oi.drive_rot.getAsDouble(), Constants.OI.deadband), 3) * Constants.Swerve.maxAngularSpeed * (oi.driveSlow.getAsBoolean() ? Constants.OI.slowSpeed : 1),
            true,
            false
        )));
    //*/
    //swerve.setDefaultCommand(new TeleopDriveCommand(swerve, oi, false, true, false, true,
    //                        elevatorArm::getAngle, elevatorArm::getExtention, intake::getGameObject));
    elevatorArm.setDefaultCommand(elevatorArm.run(elevatorArm::stop));
    intake.setDefaultCommand(intake.run(intake::stop));

    nextArmPosition = Position.L3CONE;
    nextScoringSlot = 0;

    SmartDashboard.putData("vision off", new InstantCommand(() -> {vision = null; triedVision = true;}));
    SmartDashboard.putData("vision blue", new InstantCommand(() -> {vision = new Vision(DriverStation.Alliance.Blue); triedVision = true;}));
    SmartDashboard.putData("vision red", new InstantCommand(() -> {vision = new Vision(DriverStation.Alliance.Red); triedVision = true;}));

    configureBindings();
  }

  private void configureBindings() {
    oi.closeIntakeButton.onTrue(intake.closeCommand());
    //oi.openIntakeButton.onTrue(intake.openCommand());
    oi.openIntakeButton.whileTrue(intake.openCommand().andThen(intake.autoCloseCommand()));

    //TODO - check for cones vs cubes for arm positions

    oi.armToNextTargetPositionButton.onTrue(new ProxyCommand(() -> elevatorArm.goToTargetCommand(nextArmPosition)));
    oi.armToShelfIntakePositionButton.onTrue(elevatorArm.goToTargetCommand(Position.SHELF));
    oi.armToGroundIntakePositionButton.onTrue(elevatorArm.goToTargetCommand(Position.L1CONE)); 
    oi.armToHomePositionButton.onTrue(elevatorArm.goToTargetCommand(Position.HOME));

    oi.fullAutoScore.onTrue(
      new ProxyCommand(
        () -> sequence(
          new AutoScoringTrajectoryCommand(nextScoringSlot, new PathConstraints(4, 3), autos.autoBuilder, swerve)
          .alongWith(elevatorArm.goToTargetCommand(nextArmPosition))
        )
      )
    );
    oi.driveToScoringNodeButton.onTrue(new ProxyCommand(() -> new AutoScoringTrajectoryCommand(nextScoringSlot, new PathConstraints(4, 3), autos.autoBuilder, swerve))); //TODO determine path constraints
    oi.gyroResetButton.onTrue(swerve.runOnce(swerve::resetGyro));
    oi.interruptButton.onTrue(new InstantCommand(elevatorArm::stop, elevatorArm))
                      .onTrue(new InstantCommand(swerve::stop, swerve))
                      .onTrue(new InstantCommand(intake::stop, intake));

    //oi.targetLvl1ArmPosition.onTrue(new InstantCommand(() -> nextArmPosition = Position.L1CONE));
    //oi.targetLvl2ArmPosition.onTrue(new InstantCommand(() -> nextArmPosition = Position.L2CONE));
    //oi.targetLvl3ArmPosition.onTrue(new InstantCommand(() -> nextArmPosition = Position.L3CONE));
    oi.targetLvl1ArmPosition.onTrue(new ConditionalCommand(
          new InstantCommand(() -> nextArmPosition = Position.L1CONE),
          new InstantCommand(() -> nextArmPosition = Position.L1CUBE),
          () -> intake.getGameObject() == GameObject.CONE));
    oi.targetLvl2ArmPosition.onTrue(new ConditionalCommand(
          new InstantCommand(() -> nextArmPosition = Position.L2CONE),
          new InstantCommand(() -> nextArmPosition = Position.L2CUBE),
          () -> intake.getGameObject() == GameObject.CONE));
    oi.targetLvl3ArmPosition.onTrue(new ConditionalCommand(
          new InstantCommand(() -> nextArmPosition = Position.L3CONE),
          new InstantCommand(() -> nextArmPosition = Position.L3CUBE),
          () -> intake.getGameObject() == GameObject.CONE));
    
    oi.manualArmButton.whileTrue(elevatorArm.manualMoveCommand(oi.manualArmVerticalPower, oi.manualArmElevatorPower));

    oi.setGameObjectCone.onTrue(intake.runOnce(() -> intake.setGameObject(GameObject.CONE)));
    oi.setGameObjectCube.onTrue(intake.runOnce(() -> intake.setGameObject(GameObject.CUBE)));
    oi.setGameObjectNone.onTrue(intake.runOnce(() -> intake.setGameObject(GameObject.NONE)));

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
    SmartDashboard.putBoolean("vision on", vision != null);
    if (!triedVision) {
      if (vision == null) {
        //if (DriverStation.isFMSAttached() || true) { //TODO
        if (DriverStation.isDSAttached()) {
          vision = new Vision();
        }
      } else {
        Optional<EstimatedRobotPose> visionPose = vision.getEstimatedGlobalPose(swerve.getPose());
        if (visionPose.isPresent()) {
          triedVision = true;
          if (visionPose.get().estimatedPose.getX() > 3.0) {
            vision = null;
          }
        }
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
