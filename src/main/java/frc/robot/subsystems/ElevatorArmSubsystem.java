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
        Retracted,
        Lv1,
        Lv2,
        Lv3
    }

    Level lastLevel = Level.Retracted;
    double extention = 0.0;
    Rotation2d angle = new Rotation2d();
    Load load = Load.None;

    private final WPI_TalonSRX verticalTalon;
    private final WPI_TalonSRX elevatorTalon;


    Rotation2d target_angle = new Rotation2d();
    double target_extention = 0.0;
    TrapezoidProfile verticalTProfile;
    TrapezoidProfile elevatorTProfile;

    public ElevatorArmSubsystem() {
        verticalTalon = new WPI_TalonSRX(Constants.ElevatorArm.VerticalDrive.verticalTalonID);
        elevatorTalon = new WPI_TalonSRX(Constants.ElevatorArm.ElevatorDrive.elevatorTalonID);

        verticalTalon.configFactoryDefault();
        elevatorTalon.configFactoryDefault();

        verticalTalon.setNeutralMode(NeutralMode.Brake);
        elevatorTalon.setNeutralMode(NeutralMode.Brake);


    }

    /**
     * the coordanates are relative to the robot's center
     * @param cartesian
     */
    private void calculateFromPolar(Translation2d cartesian) {
        target_angle = cartesian.getAngle();
        target_extention = cartesian.getDistance(Constants.ElevatorArm.armOffset);
    }

    /**
     * 
     * @return the TrapezoidProfile for the vertical motor
     */
    private void calcVerticalTrapezoidProfile() {
        double targetPos = target_angle.getRadians();
    
        //TODO fill in the constants
        verticalTProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(0.0, 0.0),
            new TrapezoidProfile.State(targetPos, 0.0),
            new TrapezoidProfile.State(getAngle().getRadians(), 0.0)
        );
    }

    /**
     * 
     * @return the TrapezoidProfile for the elevator motor
     */
    private void calcElevatorTrapezoidProfile() {
        double targetPos = target_extention;

        //TODO fill in the constants
        elevatorTProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(0.0, 0.0),
            new TrapezoidProfile.State(targetPos, 0.0),
            new TrapezoidProfile.State(getExtention(), 0.0)
        );
    }

    /**
     * sets the output of the vertical and elevator motors
     * to the values calculated by verticalTProfile and 
     * elevatorTProfile
     * 
     * @param deltaT - delta time in seconds
     */
    private void move(double deltaT) {
        verticalTalon.set(
            ControlMode.Velocity,
            verticalTProfile.calculate(deltaT).velocity
        );
        elevatorTalon.set(
            ControlMode.Velocity,
            elevatorTProfile.calculate(deltaT).velocity
        );
    }

    @Override
    public void periodic() {
        move(0.02);
    }

    public void setTarget(Translation2d target) {
        calculateFromPolar(target); // calculate the polar coords
        calcVerticalTrapezoidProfile(); // calculate the vertical drive TrapezoidProfile
        calcElevatorTrapezoidProfile(); // calculate the elevator drive TrapezoidProfile
    }

    /**
     * 
     * @return Rotation2d - the angle of the arm
     */
    private Rotation2d getAngle() {
        return new Rotation2d(
            Units.rotationsToRadians( // convert rotations to radians
                verticalTalon.getSelectedSensorPosition() // raw sensor units
                / 4096.0 // revolutions before gear ratio
                / Constants.ElevatorArm.VerticalDrive.gearRatio // final revolutions
        ));
    }

    /**
     * 
     * @return double - the extention of the elevator in meters
     */
    private double getExtention() {
        return elevatorTalon.getSelectedSensorPosition() // raw sensor units
            / 4096.0 // revolutions before gear ratio
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
            * Constants.ElevatorArm.VerticalDrive.gearRatio // revolutions before gear ratio
            * 4096.0; // raw sensor units
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
            * 4096.0; // raw sensor units
    }

    public void nextLevel() {

        switch (lastLevel) {
            case Retracted:
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
    public void targetReset() {
        lastLevel = Level.Retracted;
        setTarget(Constants.ElevatorArm.Positions.reset);
    }

    /**
     * sets the target to the retracted state
     */
    public void targetLv1() {
        setTarget(Constants.ElevatorArm.Positions.level1);
    }

    /**
     * sets the target to the retracted state
     */
    public void targetLv2() {
        setTarget(Constants.ElevatorArm.Positions.level2);
    }

    /**
     * sets the target to the retracted state
     */
    public void targetLv3() {
        setTarget(Constants.ElevatorArm.Positions.level3);
    }

    /**
     * sets the target to retract the elevator all the way
     */
    public void targetRetract() {
        target_extention = Constants.ElevatorArm.initialArmLength;
        calcVerticalTrapezoidProfile(); // calculate the vertical drive TrapezoidProfile
        calcElevatorTrapezoidProfile(); // calculate the elevator drive TrapezoidProfile
    }

    





    /**
     * Calculates the KG of the vertical drive.
     * This is not super straight forward as this
     * changes with the extention of the arm and the load
     */
    private double getVerticalKG() {
        // TODO replace with actual calculation
        return Constants.ElevatorArm.lowerArmWeight + (Constants.ElevatorArm.upperArmWeight * extention);
    }

    private ArmFeedforward getVerticalFeedForward() {
        return new ArmFeedforward(
            Constants.ElevatorArm.VerticalDrive.KS,
            getVerticalKG(),
            Constants.ElevatorArm.VerticalDrive.KV
        );
    }

    public Translation3d getCenterOfMass() {
        double lw = 0.0;
        if (load == Load.Cone) {lw = Constants.ElevatorArm.coneWeight;}
        if (load == Load.Cube) {lw = Constants.ElevatorArm.cubeWeight;}

        Translation2d cm = new Translation2d(((
                (Constants.ElevatorArm.maxExtention / 2) * Constants.ElevatorArm.upperArmWeight) + 
                ((Constants.ElevatorArm.initialArmLength / 2) * Constants.ElevatorArm.lowerArmWeight) + 
                ((extention) * lw)) / 
                (Constants.ElevatorArm.upperArmWeight + Constants.ElevatorArm.lowerArmWeight + lw), angle);
        
        double y = cm.getY();
        double z = cm.getX();

        return new Translation3d( 0.0, y, z);
    }

    public void setDesiredState(Translation2d desiredPos) {

    }

}
