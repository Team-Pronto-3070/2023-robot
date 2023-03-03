package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final double fieldWidthMeters = 8.02;

    public static final class Auto {
        public static final double maxVelocity = 4.0; //meters per second
        public static final double maxAcceleration = 3.0; //meters per second squared
        public static final double velocityDeadband = 0.05; //meters per second
        public static final class TranslationPID {
            public static final double P = 5.0;
            public static final double I = 0.0;
            public static final double D = 0.0;
        }
        public static final class RotationPID {
            public static final double P = 0.5;
            public static final double I = 0.0;
            public static final double D = 0.0;
        }
    }

    public static final class OI {
        public static final int driverPort = 0;
        public static final double deadband = 0.06;
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
        public static final double maxAngularSpeed = maxSpeed / Math.hypot(wheelBase / 2.0, trackWidth / 2.0);

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
                public static final double P = 0.05;
                public static final double I = 0.0;
                public static final double D = 0.0;
                public static final double F = 0.0;
            }
            public static final class Feedforward {
                public static final double KS = 0.0;
                public static final double KV = 12 / maxSpeed;
                public static final double KA = 12 / maxAcceleration;
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

    public static enum GameObject {
        NONE (0),
        CUBE (0.653),
        CONE (0.071);

        private final double mass;
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
        public static final double elevatorCarriageMass = Units.lbsToKilograms(0.0);
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
        public static final double elevatorStaticMass = Units.lbsToKilograms(0.0);
        public static final Translation3d pivotLocation = //location of the pivot point relative to the center of the robot
            new Translation3d(
                Units.inchesToMeters(0.0),
                Units.inchesToMeters(0.0),
                Units.inchesToMeters(0.0)
            );
        public static final Translation3d chassisCG = 
            new Translation3d(
                Units.inchesToMeters(0.0),
                Units.inchesToMeters(0.0),
                Units.inchesToMeters(0.0)
            );
        public static final double chassisMass = Units.lbsToKilograms(0.0);
    }

    public static final class Vision {
        public static final String cameraName = "";
        public static final Transform3d robotToCamera = new Transform3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0)
        );
    }

    public static final class DriveCommands {
        public static final class AutoBalance {
            public static final double driveUpRampSpeed = 0.5;
            public static final double onRampAngle = Units.degreesToRadians(6);
            public static final double stopAngle = Units.degreesToRadians(3);
            public static final double balanceSetpoint = 0.0;
            public static final class PID {
                public static final double P = 0.0;
                public static final double I = 0.0;
                public static final double D = 0.0;
            }
        }
    }
}
