package frc.robot;

import java.util.function.Function;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.util.PicoColorSensor.RawColor;

public final class Constants {

    public static final double fieldWidthMeters = 8.02;
    public static final double robotLengthWithBumpers = Units.inchesToMeters(32.5);

    public static final class Auto {
        public static final double maxVelocity = 2.5; //meters per second
        public static final double maxAcceleration = 1.2; //meters per second squared
        public static final double velocityDeadband = 0.05; //meters per second
        public static final class TranslationPID {
            public static final double P = 2.0;
            public static final double I = 0.0;
            public static final double D = 0.0;
        }
        public static final class RotationPID {
            public static final double P = 1.5;
            public static final double I = 0.0;
            public static final double D = 0.0;
        }
    }
    public static final double loopTime = 0.02; // in seconds

    public static final class RobotBounds {
        public static final double maxHeight = Units.inchesToMeters(78.0);
        public static final double maxHorizontalExtention = Units.inchesToMeters(48.0);
        public static final double robotLength = Units.inchesToMeters(26);
    }

    public static final class OI {
        public static final int driverPort = 0;
        public static final int operatorPort = 1;
        public static final double deadband = 0.20;
        public static final double triggerDeadband = 0.5;

        public static final double slowSpeed = 0.5;

        public static final double maxManualExtensionSpeed = 0.8;
        public static final double maxManualRotationSpeed = 1.0;
    }

    public static final class Intake {
        public static final int ID = 13;
        public static final int leftSwitchPort = 4;
        public static final int rightSwitchPort = 3;
        public static final double closeVelocity = 1.0;
        public static final double openVelocity = -1.0;
        public static final boolean inverted = false;
        public static final double closeDuration = 2.0;
        public static final double openTimeout = 1.0;
        public static final TalonSRXConfiguration config = new TalonSRXConfiguration();
        public static final double currentDebounceTime = .2;
        public static final double closedCurrent = 14.5; // amps
        static {
            config.continuousCurrentLimit = 20; //amps
            config.peakCurrentDuration = 100; //miliseconds
            config.peakCurrentLimit = 25; //amps
        }
        public static final int distanceThreshold = 60; //arbitrary units, greater is closer
        public static final double distanceDebounceTime = 0.5; //seconds
        public static final Function<RawColor, Boolean> isCube = 
            (color) -> color.red <= color.green;
        public static final Function<RawColor, Boolean> isCone = 
            (color) -> color.red > color.green;
    }

    public static final class Swerve {
        public static final double wheelBase = Units.inchesToMeters(26.0 - 3.5); //distance between front and back wheels
        public static final double trackWidth = Units.inchesToMeters(24.0 - 3.5); //distance between left and right wheels
        public static final double wheelCircumference = Units.inchesToMeters(3) * Math.PI;

            // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
            // This changes the drive speed of the module (a pinion gear with more teeth will result in a
            // robot that drives faster).
        private static final int drivingMotorPinionTeeth = 13; 
        public static final double gearRatio = (45.0 * 22) / (drivingMotorPinionTeeth * 15);

        public static final double maxSpeed = Units.feetToMeters(15.87); //meters per second
        public static final double maxAcceleration = 1.19 * 9.81; // traction limited: COF*g (TODO: this cof is for blue nitrile on carpet)
        //public static final double maxAngularSpeed = 10.0 * maxSpeed / Math.hypot(wheelBase / 2.0, trackWidth / 2.0);
        public static final double maxAngularSpeed = 1.0 * maxSpeed / Math.hypot(wheelBase / 2.0, trackWidth / 2.0);

        //offsets are in radians
        public static final class FrontLeft {
            public static final int driveID = 4;
            public static final int turnID = 8;
            public static final double offset = -Math.PI / 2;
        }
        public static final class FrontRight {
            public static final int driveID = 3;
            public static final int turnID = 7;
            public static final double offset = 0.0;
        }
        public static final class RearLeft {
            public static final int driveID = 2;
            public static final int turnID = 6;
            public static final double offset = Math.PI;
        }
        public static final class RearRight {
            public static final int driveID = 1;
            public static final int turnID = 5;
            public static final double offset = Math.PI / 2;
        }
        public static final class Drive {
            public static final class PID {
                public static final double P = 0.1;
                public static final double I = 0.0;
                public static final double D = 0.0;
                public static final double F = 0.0;
            }
            public static final class Feedforward {
                public static final double KS = 0.0;
                //public static final double KV = 12 / maxSpeed;
                public static final double KV = 0.0;
                //public static final double KA = 12 / maxAcceleration;
                public static final double KA = 0.0;
            }
            public static final int continuousCurrentLimit = 35;
            public static final int peakCurrentLimit = 60;
            public static final double peakCurrentDuration = 0.1;
            public static final boolean enableCurrentLimit = true;

            public static final double openLoopRamp = 0.25;
            public static final double closedLoopRamp = 0.0;

            public static final boolean motorInvert = false;
            public static final NeutralMode neutralMode = NeutralMode.Brake;
        }

        public static final class Turn {
            public static final double encoderPositionFactor = 2 * Math.PI; //radians
            public static final double encoderVelocityFactor = 2 * Math.PI / 60.0; //radians per second
            public static final boolean encoderInvert = true;

            
            public static final double maxModuleAngularSpeed = //radians per second
                        Units.rotationsPerMinuteToRadiansPerSecond(
                            11000.0             //NEO550 free speed (rpm)
                            * 203.0 / 9424.0);  //gear ratio
            public static final double KV = 12.0 / maxModuleAngularSpeed; //volts * seconds / radians

            public static final class PID {
                public static final double P = 1.0;
                public static final double I = 0.0;
                public static final double D = 0.0;
                public static final double F = 0.0;
                public static final double minOutput = -1;
                public static final double maxOutput = 1;
            }
            public static final IdleMode idleMode = IdleMode.kBrake;
            public static final int currentLimit = 20;
        }
    }

    public static final class ElevatorArm {
        public static final double initialArmLength = 1.145180; // in meters
        public static final double maxExtention = Units.inchesToMeters(41.0 - 0.9); // in meters
        
        public static final class VerticalDrive {
            //base units are radians, so velocity is rad/s and accel is rad/s^2
            //public static final TrapezoidProfile.Constraints trapConstraints = new TrapezoidProfile.Constraints(1, 1); //TODO
            
            public static final int ID = 12;

            public static final Rotation2d tolerance = Rotation2d.fromDegrees(0.5);

            //TODO
            public static final boolean motorReversed = true;
            public static final boolean sensorPhase = false;

            public static final double absoluteEncoderOffset = 3219.0; // encoder units when arm is horizontal

            public static final TalonSRXConfiguration config = new TalonSRXConfiguration();
            static {
                config.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;

                //TODO: pid constants
                config.slot0 = new SlotConfiguration();
                config.slot0.kP = 30.0;
                config.slot0.kI = 0.0;
                config.slot0.kD = 0.0;
                //motion magic: "kF is multiplied by the runtime-calculated target and added to output" - presumambly calculated velocity

                config.continuousCurrentLimit = 35; //amps
                config.peakCurrentDuration = 100; //miliseconds
                config.peakCurrentLimit = 60; //amps

                config.forwardSoftLimitEnable = true; //TODO: set this to true once the threshold is filled in
                //config.forwardSoftLimitThreshold = 870.0;
                config.forwardSoftLimitThreshold = 912.0;
                config.reverseSoftLimitEnable = true; //TODO: set this to true once the threshold is filled in
                config.reverseSoftLimitThreshold = 77.0;

                //TODO
                config.motionCruiseVelocity = 2.0                    // rad/s                             max velocity in radians per second
                                              * (1 / (2 * Math.PI))  // * rot/rad -> rot/s                rotations per second
                                              * 4096.0               // * sensor units / rot -> units/s   sensor units per second
                                              * 10.0;                // * s/(100ms) -> units/100ms        sensor units per 100 miliseconds

                config.motionAcceleration =   1.5                    // rad/s^2                           max acceleration in radians per s^2
                                              * (1 / (2 * Math.PI))  // * rot/rad -> rot/s^2              rotations per second^2
                                              * 4096.0               // * sensor units / rot -> units/s^2 sensor units per second^2
                                              / 10.0;                // * s/(100ms) -> (units/100ms)/s    (sensor units / 100 miliseconds) / second

                config.motionCurveStrength = 3; //arbitary smoothing value
            }

            public static final double KS = 0.0;
            public static final double KV = 0.0;
            public static final double KA = 0.0;
            public static final double KG = 0.0;
        }
        public static final class ElevatorDrive {
            public static final int ID = 11;

            //base units are meters, so velocity is m/s and accel is m/s^2
            //public static final TrapezoidProfile.Constraints trapConstraints = new TrapezoidProfile.Constraints(1, 1); //TODO

            public static final double gearRatio = 16; // 16:1
            public static final double pulleyCircumference = 2 * Math.PI
                                                               * 0.023300; // radius in meters
            
            public static final double tolerance = Units.inchesToMeters(0.5); // meters

            //TODO
            public static final boolean motorReversed = false;
            public static final boolean sensorPhase = true;

            public static final TalonSRXConfiguration config = new TalonSRXConfiguration();
            static {
                config.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;

                //TODO: pid constants
                config.slot0 = new SlotConfiguration();
                config.slot0.kP = 0.5;
                config.slot0.kI = 0.0;
                config.slot0.kD = 0.0;
                //motion magic: "kF is multiplied by the runtime-calculated target and added to output" - presumambly calculated velocity

                config.continuousCurrentLimit = 35; //amps
                config.peakCurrentDuration = 100; //miliseconds
                config.peakCurrentLimit = 60; //amps

                config.forwardLimitSwitchNormal = LimitSwitchNormal.NormallyClosed;
                //config.reverseLimitSwitchNormal = LimitSwitchNormal.NormallyClosed;
                config.reverseLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
                config.clearPositionOnLimitR = true; //reset selected sensor position to 0 on falling edge of reverse limit switch

                //TODO
                config.motionCruiseVelocity = (0.45                  // m/s                               max velocity in m/s
                                              / pulleyCircumference) // *(m/rot)^-1 = rot/m -> rot/s      pulley rotations per second
                                              * gearRatio            // * unitless -> rot/s               sensor rotations per second
                                              * 4096.0               // * sensor units / rot -> units/s   sensor units per second
                                              / 10.0;                // * s/(100ms) -> units/100ms        sensor units / 100 miliseconds

                config.motionAcceleration =   (1.0                   // m/s^2                             max acceleration in m/s^2
                                              / pulleyCircumference) // *(m/rot)^-1 = rot/m -> rot/s^2    pulley rotations per second^2
                                              * gearRatio            // * unitless -> rot/s^2             sensor rotations per second^2
                                              * 4096.0               // * sensor units / rot -> units/s^2 sensor units per second^2
                                              / 10.0;                // * s/(100ms) -> (units/100ms)/s    (sensor units / 100 miliseconds) / second

                config.motionCurveStrength = 3; //arbitary smoothing value
            }
            public static final double KS = 0.0;
            public static final double KV = 0.0;
            public static final double KA = 0.0;
            public static final double KG = 0.0;
        }

        public static enum Position {
            //HOME (new Translation2d(Constants.ElevatorArm.initialArmLength, Rotation2d.fromDegrees(75))),
            HOME (new Translation2d(Constants.ElevatorArm.initialArmLength, Rotation2d.fromDegrees(80.5))),

            //L1CONE (new Translation2d(Constants.ElevatorArm.initialArmLength, Rotation2d.fromDegrees(5))),
            L1CONE (new Translation2d(Constants.ElevatorArm.initialArmLength, Rotation2d.fromDegrees(11))),
            L2CONE (new Translation2d(1.48, Rotation2d.fromDegrees(50.3))),
            //L3CONE (new Translation2d(1.9, Rotation2d.fromDegrees(35.6))),
            L3CONE (new Translation2d(1.9, Rotation2d.fromDegrees(46))),

            L1CUBE (new Translation2d(Constants.ElevatorArm.initialArmLength, Rotation2d.fromDegrees(11))),
            L2CUBE (new Translation2d(1.4, Rotation2d.fromDegrees(37))),
            L3CUBE (new Translation2d(1.9, Rotation2d.fromDegrees(38.5))),

            //SHELF (new Translation2d(Constants.ElevatorArm.initialArmLength, Rotation2d.fromDegrees(59.0))),
            SHELF (new Translation2d(1.165, Rotation2d.fromDegrees(59.0))),

            AUTOL3CUBE (new Translation2d(1.9, Rotation2d.fromDegrees(45.0))),
            AUTOL3CONE (new Translation2d(1.9, Rotation2d.fromDegrees(50.0)));

            public final Translation2d translation;
            private Position(Translation2d translation) {
                this.translation = translation;
            }
        }

    }

    public static enum GameObject {
        NONE (0),
        CUBE (0.653),
        CONE (0.071);

        private final double mass; //kilograms
        private GameObject(double mass) {
            this.mass = mass;
        }

        public double getMass() {
            return mass;
        }
    }

    public static final class MassProperties {
        public static final Translation3d elevatorCarriageCG = //relative to pivot point when fully retracted
            new Translation3d(
                Units.inchesToMeters(0.0),
                Units.inchesToMeters(0.0),
                Units.inchesToMeters(0.0)
            );
        public static final double elevatorCarriageMass = Units.lbsToKilograms(0.1);
        public static final Translation3d gameObjectLocation = //relative to pivot point when fully retracted
            new Translation3d(
                Units.inchesToMeters(0.0),
                Units.inchesToMeters(0.0),
                Units.inchesToMeters(0.0)
            );
        public static final Translation3d elevatorStaticCG = //relative to pivot point
            new Translation3d(
                Units.inchesToMeters(0.0),
                Units.inchesToMeters(0.0),
                Units.inchesToMeters(0.0)
            );
        public static final double elevatorStaticMass = Units.lbsToKilograms(0.1);
        public static final Translation3d pivotLocation = //location of the pivot point relative to the center of the robot
            new Translation3d(
                Units.inchesToMeters(8.5),
                Units.inchesToMeters(0.0),
                Units.inchesToMeters(8.0)
            );
        public static final Translation3d chassisCG = 
            new Translation3d(
                Units.inchesToMeters(0.0),
                Units.inchesToMeters(0.0),
                Units.inchesToMeters(24.0)
            );
        public static final double chassisMass = Units.lbsToKilograms(90.0);
    }

    public static final class Vision {
        public static final String cameraName = "OV9281_1";
        public static final Transform3d robotToCamera = new Transform3d(
            //new Translation3d(Units.inchesToMeters(4), Units.inchesToMeters(10), Units.inchesToMeters(6)),
            //new Translation3d(Units.inchesToMeters(4), Units.inchesToMeters(-11.5), Units.inchesToMeters(15)),
            new Translation3d(Units.inchesToMeters(13.0 - 8.5), Units.inchesToMeters(1.875 - 12.0), Units.inchesToMeters(9.75)),
            new Rotation3d(0, -Units.degreesToRadians(20), 0)
        );
    }

    public static final class DriveCommands {
        public static final class AutoBalance {
            public static final double driveUpRampSpeed = 1.0;
            public static final double onRampAngle = 12.0;
            public static final double stopAngle = 15.0;
            public static final double balanceSetpoint = 0.0;
            public static final double angleDebounceTime = 0.2;
            public static final class PID {
                public static final double P = 1.0;
                public static final double I = 0.0;
                public static final double D = 0.0;
            }
        }
    }
}
