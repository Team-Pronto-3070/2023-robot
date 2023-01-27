package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class OI {
        public static final int driverPort = 1;
        public static final double deadband = 0.06;
    }

    //TODO fill consts
    // need to fill consts so that simulation can work
    public static final class Swerve {
        public static final double wheelBase = 0.0; //distance between front and back wheels
        public static final double trackWidth = 0.0; //distance between left and right wheels
        public static final double wheelCircumference = Units.inchesToMeters(3) * Math.PI;

            // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
            // This changes the drive speed of the module (a pinion gear with more teeth will result in a
            // robot that drives faster).
        private static final int drivingMotorPinionTeeth = 14; 
        public static final double gearRatio = (45.0 * 22) / (drivingMotorPinionTeeth * 15);

        public static final double maxSpeed = 0.0; //meters per second
        public static final double maxAngularSpeed = 0.0;

        //offsets are in radians
        public static final class FrontLeft {
            public static final int driveID = 0;
            public static final int turnID = 0;
            public static final double offset = 0.0;
        }
        public static final class FrontRight {
            public static final int driveID = 0;
            public static final int turnID = 0;
            public static final double offset = 0.0;
        }
        public static final class RearLeft {
            public static final int driveID = 0;
            public static final int turnID = 0;
            public static final double offset = 0.0;
        }
        public static final class RearRight {
            public static final int driveID = 0;
            public static final int turnID = 0;
            public static final double offset = 0.0;
        }
        public static final class Drive { //TODO
            public static final class PID {
                public static final double P = 0.0;
                public static final double I = 0.0;
                public static final double D = 0.0;
                public static final double F = 0.0;
            }
            public static final class Feedforward {
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

        public static final class Turn { //TODO
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
}
