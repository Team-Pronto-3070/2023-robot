// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Autos;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private final OI oi = new OI(Constants.OI.driverPort);
  private final Vision vision = new Vision();

  private final SwerveSubsystem swerve = new SwerveSubsystem();

  private final Autos autos = new Autos(swerve);

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
    return autos.getSelectedAuto();
  }
}
