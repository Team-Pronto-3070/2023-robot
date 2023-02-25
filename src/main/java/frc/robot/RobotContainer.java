// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ElevatorArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private final OI oi = new OI(Constants.OI.driverPort);

  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final ElevatorArmSubsystem elevatorArm = new ElevatorArmSubsystem();

  public RobotContainer() {
    swerve.setDefaultCommand(swerve.run(() -> swerve.drive(
            MathUtil.applyDeadband(oi.drive_x.getAsDouble(), Constants.OI.deadband, Constants.Swerve.maxSpeed),
            MathUtil.applyDeadband(oi.drive_y.getAsDouble(), Constants.OI.deadband, Constants.Swerve.maxSpeed),
            MathUtil.applyDeadband(oi.drive_rot.getAsDouble(), Constants.OI.deadband, Constants.Swerve.maxAngularSpeed),
            true,
            true
        )));

    configureBindings();
  }

  private void configureBindings() {
    oi.gyroResetButton.onTrue(swerve.runOnce(swerve::resetGryo));
    oi.elevatorArmReset.onTrue(elevatorArm.runOnce(elevatorArm::targetReset));
    oi.elevatorArmNextLevel.onTrue(elevatorArm.runOnce(elevatorArm::nextLevel));
    oi.elevatorArmRetract.onTrue(elevatorArm.runOnce(elevatorArm::targetRetract));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
