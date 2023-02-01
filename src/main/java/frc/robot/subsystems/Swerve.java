
// Here is where the swerve subsystem is defined

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.ProntoSwerveModule;

public class Swerve extends SubsystemBase {

    private final ProntoSwerveModule frontLeft;
    private final ProntoSwerveModule frontRight;
    private final ProntoSwerveModule rearLeft;
    private final ProntoSwerveModule rearRight;

    private final ADIS16448_IMU gyro;

    public final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    
    public Swerve() {
        frontLeft = new ProntoSwerveModule(
            Constants.Swerve.FrontLeft.driveID,
            Constants.Swerve.FrontLeft.turnID,
            Constants.Swerve.FrontLeft.offset
        );

        frontRight = new ProntoSwerveModule(
            Constants.Swerve.FrontRight.driveID,
            Constants.Swerve.FrontRight.turnID,
            Constants.Swerve.FrontRight.offset
        );

        rearLeft = new ProntoSwerveModule(
            Constants.Swerve.RearLeft.driveID,
            Constants.Swerve.RearLeft.turnID,
            Constants.Swerve.RearLeft.offset
        );

        rearRight = new ProntoSwerveModule(
            Constants.Swerve.RearRight.driveID,
            Constants.Swerve.RearRight.turnID,
            Constants.Swerve.RearRight.offset
        );

        gyro = new ADIS16448_IMU();

        kinematics = new SwerveDriveKinematics(
            new Translation2d(Constants.Swerve.wheelBase / 2, Constants.Swerve.trackWidth / 2),
            new Translation2d(Constants.Swerve.wheelBase / 2, -Constants.Swerve.trackWidth / 2),
            new Translation2d(-Constants.Swerve.wheelBase / 2, Constants.Swerve.trackWidth / 2),
            new Translation2d(-Constants.Swerve.wheelBase / 2, -Constants.Swerve.trackWidth / 2)
        );

        odometry = new SwerveDriveOdometry(
            kinematics,
            getYaw(),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                rearLeft.getPosition(),
                rearRight.getPosition()
            }
        );
        
    }

    /**
     * 
     * @return the yaw of the swerve wheel
     */
    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(gyro.getAngle());
    }

    public void resetGryo() {
        gyro.reset();
    }

    /**
     * 
     * @return the position of the robot on the field as x and y in meters
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }


    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
            getYaw(),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                rearLeft.getPosition(),
                rearRight.getPosition()
            },
            pose
        );
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean isOpenLoop) {
        var swerveModuleStates = kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getYaw())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        frontLeft.setDesiredState(swerveModuleStates[0], isOpenLoop);
        frontRight.setDesiredState(swerveModuleStates[1], isOpenLoop);
        rearLeft.setDesiredState(swerveModuleStates[2], isOpenLoop);
        rearRight.setDesiredState(swerveModuleStates[3], isOpenLoop);
    }
    
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        frontLeft.setDesiredState(desiredStates[0], false);
        frontRight.setDesiredState(desiredStates[1], false);
        rearLeft.setDesiredState(desiredStates[2], false);
        rearRight.setDesiredState(desiredStates[3], false);
    }

    @Override
    public void periodic() {
        odometry.update(
            getYaw(),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                rearLeft.getPosition(),
                rearRight.getPosition()
            }
        );
    }
}
