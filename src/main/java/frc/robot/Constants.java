package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {
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

        public static final double maxSpeed = 4.0; //meters per second
        public static final double maxAngularSpeed = 2 * Math.PI;

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
            public static final class Feedforward { //TODO
                public static final double KS = 0.0;
                public static final double KV = 0.0;
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
        public static final double initialArmLength = 1.0;
        public static final double maxExtention = 1.0;
        public static final int extentionMotorID = 0;
        public static final int verticalDriveMotorID = 0;
        public static final Translation2d armOffset = new Translation2d();

        public static final class VerticalDrive {
            public static final double KS = 0.0;
            public static final double KV = 0.0;
            public static final double KA = 0.0;
            public static final double KG = 0.0;
        }
        public static final class ExtentionDrive {
            public static final double KS = 0.0;
            public static final double KV = 0.0;
            public static final double KA = 0.0;
            public static final double KG = 0.0;
        }

        public static final double lowerArmWeight = 0.0;
        public static final double upperArmWeight = 0.0;
        public static final double cubeWeight = 0.0;
        public static final double coneWeight = 0.0;

    }

    public static final class Manipulator {
        public static final double weight = 0.0;
    }
}
