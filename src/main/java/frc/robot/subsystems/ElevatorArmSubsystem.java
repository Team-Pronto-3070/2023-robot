package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.fasterxml.jackson.databind.deser.std.ContainerDeserializerBase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ArmFeedforward;
import frc.robot.Constants;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;



public class ElevatorArmSubsystem extends SubsystemBase {

    enum Load {
        None,
        Cube,
        Cone,
    }

    enum Level {
        Home,
        Lv1,
        Lv2,
        Lv3
    }

    private Level lastLevel = Level.Home;

    private final WPI_TalonSRX verticalTalon;
    private final WPI_TalonSRX elevatorTalon;


    private Rotation2d target_angle = new Rotation2d();
    private double target_extention = 0.0;
    private TrapezoidProfile verticalTProfile;
    private TrapezoidProfile elevatorTProfile;

    public ElevatorArmSubsystem() {
        verticalTalon = new WPI_TalonSRX(Constants.ElevatorArm.VerticalDrive.verticalTalonID);
        verticalTalon.configFactoryDefault();
        verticalTalon.setNeutralMode(NeutralMode.Brake);
        
        elevatorTalon = new WPI_TalonSRX(Constants.ElevatorArm.ElevatorDrive.elevatorTalonID);
        elevatorTalon.configFactoryDefault();
        elevatorTalon.setNeutralMode(NeutralMode.Brake);

    }

    /**
     * the coordanates are relative to the robot's center
     * @param cartesian
     */
    private void setTargetFromPolar(Translation2d cartesian) {
        target_angle = cartesian.getAngle();
        target_extention = cartesian.getDistance(Constants.ElevatorArm.armOffset);
    }

    /**
     * 
     * @return the TrapezoidProfile for the vertical motor
     */
    private TrapezoidProfile calcVerticalTrapezoidProfile(Rotation2d target_angle) {
        double targetPos = target_angle.getRadians();
    
        //TODO fill in the constants
        return new TrapezoidProfile(
            new TrapezoidProfile.Constraints(Constants.ElevatorArm.VerticalDrive.maxVelocity, Constants.ElevatorArm.VerticalDrive.maxAcceleration),
            new TrapezoidProfile.State(targetPos, 0.0),
            new TrapezoidProfile.State(getAngle().getRadians(), getVerticalDriveVel())
        );
    }

    /**
     * 
     * @return the TrapezoidProfile for the elevator motor
     */
    private TrapezoidProfile calcElevatorTrapezoidProfile(double target_extention) {

        //TODO fill in the constants
        return new TrapezoidProfile(
            new TrapezoidProfile.Constraints(Constants.ElevatorArm.ElevatorDrive.maxVelocity, Constants.ElevatorArm.ElevatorDrive.maxAcceleration),
            new TrapezoidProfile.State(target_extention, 0.0),
            new TrapezoidProfile.State(getExtention(), getElevatorDriveVel())
        );
    }

    /**
     * sets the output of the vertical and elevator motors
     * to the values calculated by verticalTProfile and 
     * elevatorTProfile
     * 
     * @param deltaT - delta time in seconds
     */
    public void move() {
        verticalTalon.set(
            ControlMode.Velocity,
            verticalTProfile.calculate(Constants.loopDelay).velocity
        );
        elevatorTalon.set(
            ControlMode.Velocity,
            elevatorTProfile.calculate(Constants.loopDelay).velocity
        );
    }

    public void setTarget(Translation2d target) {
        setTargetFromPolar(target); // calculate the polar coords
        verticalTProfile = calcVerticalTrapezoidProfile(target_angle); // calculate the vertical drive TrapezoidProfile
        elevatorTProfile = calcElevatorTrapezoidProfile(target_extention); // calculate the elevator drive TrapezoidProfile
    }

    /**
     * 
     * @return Rotation2d - the angle of the arm
     */
    private Rotation2d getAngle() {
        return new Rotation2d(
            Units.rotationsToRadians( // convert rotations to radians
                verticalTalon.getSelectedSensorPosition() // raw sensor units
                / 2048.0 // revolutions
        ));
    }

    /**
     * 
     * @return double - the extention of the elevator in meters
     */
    private double getExtention() {
        return elevatorTalon.getSelectedSensorPosition() // raw sensor units
            / 2048.0 // revolutions before gear ratio
            / Constants.ElevatorArm.ElevatorDrive.gearRatio // final revolutions
            * Constants.ElevatorArm.ElevatorDrive.wheelCircumference // extention distance in meters
            + Constants.ElevatorArm.initialArmLength; // full arm length
    }

    /**
     * @param sAngle
     * @return raw sensor units for TalonSRX of vertical
     */
    private double rawFromAngle(Rotation2d sAngle) {
        return sAngle.getRotations() // total revolutions
            * 2048.0; // raw sensor units
    }

    /**
     * @param sExtention
     * @return raw sensor units for TalonSRX of extention
     */
    private double rawFromExtention(double sExtention) {
        return sExtention // full arm length
            - Constants.ElevatorArm.initialArmLength // extention distance in meters
            / Constants.ElevatorArm.ElevatorDrive.wheelCircumference// final revolutions
            * Constants.ElevatorArm.ElevatorDrive.gearRatio // revolutions before gear ratio
            * 2048.0; // raw sensor units
    }

    /**
     * 
     * @return velocity of the vertical drive motor in radians/s
     */
    private double getVerticalDriveVel() {
        return Units.rotationsToRadians( // radians per second
        verticalTalon.getSelectedSensorVelocity() // raw talon units
        * (10.0 / 2048.0) // motor revolutions per second
        );
    }

    /**
     * 
     * @return velocity of the vertical drive motor in radians/s
     */
    private double getElevatorDriveVel() {
        return Units.rotationsToRadians( // radians per second
        elevatorTalon.getSelectedSensorVelocity() // raw talon units
        * (10.0 / 2048.0) // motor revolutions per second
        );
    }

    /**
     * cycles from home -> Lv3 -> Lv2 -> Lv1 -> Retracted
     */
    public void nextLevel() {

        switch (lastLevel) {
            case Home:
                targetLv3();
                break;
            
            case Lv3:
                targetLv2();
                break;
            
            case Lv2:
                targetLv1();
                break;
            
            case Lv1:
                targetRetract();
                break;
            
            default:
                break;
        }

        
    }

    /**
     * sets the target to the retracted state
     */
    public void targetHome() {
        lastLevel = Level.Home;
        setTarget(Constants.ElevatorArm.Positions.home);
    }

    /**
     * sets the target to the Lv1 (ground) state
     */
    public void targetLv1() {
        lastLevel = Level.Lv1;
        setTarget(Constants.ElevatorArm.Positions.level1);
    }

    /**
     * sets the target to the Lv2 state
     */
    public void targetLv2() {
        lastLevel = Level.Lv2;
        setTarget(Constants.ElevatorArm.Positions.level2);
    }

    /**
     * sets the target to the Lv3 state
     */
    public void targetLv3() {
        lastLevel = Level.Lv3;
        setTarget(Constants.ElevatorArm.Positions.level3);
    }

    /**
     * sets the target to retract the elevator all the way
     */
    public void targetRetract() {
        target_extention = Constants.ElevatorArm.initialArmLength; // set the extention to fully retracted
        verticalTProfile = calcVerticalTrapezoidProfile(target_angle); // calculate the vertical drive TrapezoidProfile
        elevatorTProfile = calcElevatorTrapezoidProfile(target_extention); // calculate the elevator drive TrapezoidProfile
    }

    





}
