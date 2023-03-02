package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final double loopTime = 0.02; // in seconds
    public static final class RobotBounds {
        public static final double height = Units.inchesToMeters(78.0);
        public static final double horizontalPastBumper = Units.inchesToMeters(48.0);
        public static final double robotLength = Units.inchesToMeters(0.0); // TODO fill in constant
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
    public static final class ElevatorArm {
        public static final double initialArmLength = 1.0; // in meters
        public static final double maxExtention = 1.0; // in meters
        public static final Translation2d armOffset = new Translation2d();
        
        public static final class VerticalDrive {
            //base units are radians, so velocity is rad/s and accel is rad/s^2
            public static final TrapezoidProfile.Constraints trapConstraints = new TrapezoidProfile.Constraints(0.0, 0.0);
            
            public static final int ID = 0;

            public static final boolean motorReversed = false;
            public static final boolean sensorPhase = false;

            public static final TalonSRXConfiguration config = new TalonSRXConfiguration();
            static {
                config.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;

                //TODO: pid constants
                config.slot0 = new SlotConfiguration();
                config.slot0.kP = 0.0;
                config.slot0.kI = 0.0;
                config.slot0.kD = 0.0;
                //motion magic: "kF is multiplied by the runtime-calculated target and added to output" - presumambly calculated velocity

                config.continuousCurrentLimit = 35; //amps
                config.peakCurrentDuration = 100; //miliseconds
                config.peakCurrentLimit = 60; //amps

                config.forwardSoftLimitEnable = false; //TODO: set this to true once the threshold is filled in
                config.forwardSoftLimitThreshold = 0.0;
                config.reverseSoftLimitEnable = false; //TODO: set this to true once the threshold is filled in
                config.reverseSoftLimitThreshold = 0.0;

                config.motionCruiseVelocity = 0.0                    // rad/s                             max velocity in radians per second
                                              * (1 / (2 * Math.PI))  // * rot/rad -> rot/s                rotations per second
                                              * 4096.0               // * sensor units / rot -> units/s   sensor units per second
                                              * 10.0;                // * s/(100ms) -> units/100ms        sensor units per 100 miliseconds

                config.motionAcceleration =   0.0                    // rad/s^2                           max acceleration in radians per s^2
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
            public static final int ID = 0;

            //base units are radians, so velocity is rad/s and accel is rad/s^2
            public static final TrapezoidProfile.Constraints trapConstraints = new TrapezoidProfile.Constraints(0.0, 0.0);

            public static final double gearRatio = 16; // 16:1
            public static final double pulleyCircumference = 2 * Math.PI
                                                               * 0.023300; // radius in meters

            public static final boolean motorReversed = false;
            public static final boolean sensorPhase = false;

            public static final TalonSRXConfiguration config = new TalonSRXConfiguration();
            static {
                config.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;

                //TODO: pid constants
                config.slot0 = new SlotConfiguration();
                config.slot0.kP = 0.0;
                config.slot0.kI = 0.0;
                config.slot0.kD = 0.0;
                //motion magic: "kF is multiplied by the runtime-calculated target and added to output" - presumambly calculated velocity

                config.continuousCurrentLimit = 35; //amps
                config.peakCurrentDuration = 100; //miliseconds
                config.peakCurrentLimit = 60; //amps

                config.forwardLimitSwitchNormal = LimitSwitchNormal.NormallyClosed;
                config.reverseLimitSwitchNormal = LimitSwitchNormal.NormallyClosed;
                config.clearPositionOnLimitR = true; //reset selected sensor position to 0 on falling edge of reverse limit switch

                config.motionCruiseVelocity = (0.0                   // m/s                               max velocity in m/s
                                              / pulleyCircumference) // *(m/rot)^-1 = rot/m -> rot/s      pulley rotations per second
                                              * gearRatio            // * unitless -> rot/s               sensor rotations per second
                                              * 4096.0               // * sensor units / rot -> units/s   sensor units per second
                                              / 10.0;                // * s/(100ms) -> units/100ms        sensor units / 100 miliseconds

                config.motionAcceleration =   (0.0                   // m/s^2                             max acceleration in m/s^2
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
            HOME (new Translation2d()),

            L1CONE (new Translation2d()),
            L2CONE (new Translation2d()),
            L3CONE (new Translation2d()),

            L1CUBE (new Translation2d()),
            L2CUBE (new Translation2d()),
            L3CUBE (new Translation2d());

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
}
