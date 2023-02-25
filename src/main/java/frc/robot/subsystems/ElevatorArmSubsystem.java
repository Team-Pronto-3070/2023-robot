package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.fasterxml.jackson.databind.deser.std.ContainerDeserializerBase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ArmFeedforward;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;



public class ElevatorArmSubsystem extends SubsystemBase {

    enum Load {
        None,
        Cube,
        Cone,
    }

    double extention = 0.0;
    Rotation2d angle = new Rotation2d();
    Load load = Load.None;

    private final WPI_TalonSRX verticalTalon;
    private final WPI_TalonSRX elevatorTalon;


    Rotation2d target_angle = new Rotation2d();
    double target_extention = 0.0;

    public ElevatorArmSubsystem() {
        verticalTalon = new WPI_TalonSRX(Constants.ElevatorArm.verticalTalonID);
        elevatorTalon = new WPI_TalonSRX(Constants.ElevatorArm.elevatorTalonID);

        verticalTalon.configFactoryDefault();
        elevatorTalon.configFactoryDefault();

        verticalTalon.setNeutralMode(NeutralMode.Brake);
        elevatorTalon.setNeutralMode(NeutralMode.Brake);


    }

    /**
     * the coordanates are absolute (not relative to arm base)
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
    private TrapezoidProfile getVerticalTrapezoidProfile() {
        //TODO fill in the constants
        return new TrapezoidProfile(
            new TrapezoidProfile.Constraints(0.0, 0.0),
            new TrapezoidProfile.State(0.0, 0.0),
            new TrapezoidProfile.State(0.0, 0.0)
        );
    }

    /**
     * 
     * @return the TrapezoidProfile for the elevator motor
     */
    private TrapezoidProfile getElevatorTrapezoidProfile() {
        //TODO fill in the constants
        return new TrapezoidProfile(
            new TrapezoidProfile.Constraints(0.0, 0.0),
            new TrapezoidProfile.State(0.0, 0.0),
            new TrapezoidProfile.State(0.0, 0.0)
        );
    }

    /**
     * sets the output of the vertical and elevator motors
     * to the values calculated by the TrapezoidProfile's
     * 
     * @param verticalTProfile
     * @param elevatorTProfile
     * @param deltaT
     */
    private void move(TrapezoidProfile verticalTProfile, TrapezoidProfile elevatorTProfile, double deltaT) {
        

        verticalTalon.set(
            ControlMode.Velocity,
            verticalTProfile.calculate(deltaT).velocity
        );
        elevatorTalon.set(
            ControlMode.Velocity,
            elevatorTProfile.calculate(deltaT).velocity
        );
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
