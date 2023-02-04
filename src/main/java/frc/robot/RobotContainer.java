// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  private final OI oi = new OI(Constants.OI.driverPort);
  private final Vision vision = new Vision();

  private final Swerve swerve = new Swerve();

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
  }

  public void periodic() {
    swerve.addPotentialVisionMeasurement(vision.getEstimatedGlobalPose(swerve.getPose()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
