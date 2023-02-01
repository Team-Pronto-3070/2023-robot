
// Here is the actual swerve module class and interface


package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

/**
 * Swerve module specific functions and support that don't belong in the subsystem class 
 */
public class ProntoSwerveModule {

    private final TalonFX driveMotor;
    private final CANSparkMax turnMotor;
    private final AbsoluteEncoder turningAbsoluteEncoder;

    private final SparkMaxPIDController turningPID;
    private final double chassisAngularOffset;
    private final SimpleMotorFeedforward driveFeedforward;

    private Rotation2d lastAngle;

    public ProntoSwerveModule(int driveMotorID, int turnMotorID, double encoderOffset) {
        driveMotor = new TalonFX(driveMotorID);
        driveMotor.configFactoryDefault();
        driveMotor.config_kP(0, Constants.Swerve.Drive.PID.P);
        driveMotor.config_kI(0, Constants.Swerve.Drive.PID.I);
        driveMotor.config_kD(0, Constants.Swerve.Drive.PID.D);
        driveMotor.config_kF(0, Constants.Swerve.Drive.PID.F);
        driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
            Constants.Swerve.Drive.enableCurrentLimit,
            Constants.Swerve.Drive.continuousCurrentLimit,
            Constants.Swerve.Drive.peakCurrentLimit,
            Constants.Swerve.Drive.peakCurrentDuration
        ));
        driveMotor.configOpenloopRamp(Constants.Swerve.Drive.openLoopRamp);
        driveMotor.configClosedloopRamp(Constants.Swerve.Drive.closedLoopRamp);
        driveMotor.setInverted(Constants.Swerve.Drive.motorInvert);
        driveMotor.setNeutralMode(Constants.Swerve.Drive.neutralMode);
        driveMotor.setSelectedSensorPosition(0);

        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
        turnMotor.restoreFactoryDefaults();
        turnMotor.setIdleMode(Constants.Swerve.Turn.idleMode);
        turnMotor.setSmartCurrentLimit(Constants.Swerve.Turn.currentLimit);

        turningAbsoluteEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);
        turningAbsoluteEncoder.setPositionConversionFactor(Constants.Swerve.Turn.encoderPositionFactor);
        turningAbsoluteEncoder.setVelocityConversionFactor(Constants.Swerve.Turn.encoderVelocityFactor);
        turningAbsoluteEncoder.setInverted(Constants.Swerve.Turn.encoderInvert);

        turningPID = turnMotor.getPIDController();
        turningPID.setFeedbackDevice(turningAbsoluteEncoder);
        turningPID.setPositionPIDWrappingEnabled(true);
        turningPID.setPositionPIDWrappingMinInput(0);
        turningPID.setPositionPIDWrappingMaxInput(Constants.Swerve.Turn.encoderPositionFactor);
        turningPID.setP(Constants.Swerve.Turn.PID.P);
        turningPID.setI(Constants.Swerve.Turn.PID.I);
        turningPID.setD(Constants.Swerve.Turn.PID.D);
        turningPID.setFF(Constants.Swerve.Turn.PID.F);
        turningPID.setOutputRange(Constants.Swerve.Turn.PID.minOutput, Constants.Swerve.Turn.PID.maxOutput);

        turnMotor.burnFlash();

        chassisAngularOffset = encoderOffset;
        lastAngle = new Rotation2d(chassisAngularOffset);

        driveFeedforward = new SimpleMotorFeedforward(
            Constants.Swerve.Drive.Feedforward.KS,
            Constants.Swerve.Drive.Feedforward.KV,
            Constants.Swerve.Drive.Feedforward.KA
        );
    }

    public void setDesiredState(SwerveModuleState rawDesiredState, boolean isOpenLoop) {
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(
            new SwerveModuleState(
                rawDesiredState.speedMetersPerSecond,
                rawDesiredState.angle.plus(new Rotation2d(chassisAngularOffset))
            ),
            new Rotation2d(turningAbsoluteEncoder.getPosition())
        );

        if(isOpenLoop){
            driveMotor.set(ControlMode.PercentOutput, optimizedDesiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed);
        } else {
            double wheelRPM = ((optimizedDesiredState.speedMetersPerSecond * 60) / Constants.Swerve.wheelCircumference);
            double motorRPM = wheelRPM * Constants.Swerve.gearRatio;
            double sensorCounts = motorRPM * (2048.0 / 600.0);
            driveMotor.set(ControlMode.Velocity, sensorCounts,
                           DemandType.ArbitraryFeedForward, driveFeedforward.calculate(optimizedDesiredState.speedMetersPerSecond));
        }

        //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        Rotation2d angle = (Math.abs(optimizedDesiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? 
                                lastAngle : optimizedDesiredState.angle;
        lastAngle = angle;
        turningPID.setReference(angle.getRadians(), CANSparkMax.ControlType.kPosition);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveMotor.getSelectedSensorPosition() * Constants.Swerve.wheelCircumference / (Constants.Swerve.gearRatio * 2048.0),
            new Rotation2d(turningAbsoluteEncoder.getPosition() - chassisAngularOffset)
        );
    }

    /**
     * 
     * @return motor speed & current motor rotation
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            ((driveMotor.getSelectedSensorVelocity() //raw falcon units
                * (600.0 / 2048.0) //motor RPM
                / Constants.Swerve.gearRatio) //wheel RPM
                * Constants.Swerve.wheelCircumference) //wheel surface speed in meters per minute
                / 60.0, //wheel surface speed in meters per second
            new Rotation2d(turningAbsoluteEncoder.getPosition() - chassisAngularOffset)
        );
    }
}