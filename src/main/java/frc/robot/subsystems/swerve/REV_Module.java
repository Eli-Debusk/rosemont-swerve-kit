package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.montylib.Chassis.Side;
import frc.montylib.hardware.NEOVortex;
import frc.montylib.hardware.NEOv1;
import frc.montylib.swerve.vendor.SDS.MK4i;
import frc.montylib.swerve.vendor.SDS.MK4i.*;

/** Class to handle the hardware of a SDS Mark 4 Inverted (MK4i) Swerve Module using REV Motors */
public class REV_Module {
    /* 
    * Allowed Motors for the Module are:
    * NEO Brushless Motor V1.1 (CANSparkMax MC)
    * NEO Vortex (CANSparkFlex MC) 
    */

    //Motor Devices
    private NEOVortex driveVortex, pivotVortex = null;
    private NEOv1 driveNEOv1, pivotNEOv1 = null;

    //Encoder Devices
    private RelativeEncoder driveEncoder, pivotEncoder = null;
    private CANcoder absoluteEncoder = null;

    //Pivot Motion PID Controller
    public PIDController pivotController = null;

    //Module Configuration Variables
    public boolean moduleEnabled = false;
    private Motors motor = null;
    private double maxMechanicalSpeedMetersPerSecond = 0.0;

    /** Construcs a REV_Module with default values - NOT FOR ROBOT USE */
    public REV_Module() {}

    /**
     * Constructs new REV_Module
     * @param motor the motor devices used by the module
     * @param abs_encoder the absolute encoder device used by the module
     * @param location the location of the module instance on the robot
     * @param side_reversed the desired side of the chassis which the propulsion will be reversed
     * @param gear_ratio the gear ratio of the propulsion relation
     * @param pivot_controller_constants the PID Constants for the Pivot Motion PID Controller
     */
    public REV_Module(Motors motor, AbsoluteEncoder abs_encoder, ModuleLocation location, Side side_reversed, GearRatio gear_ratio, PIDConstants pivot_controller_constants) {
        
        //Stores configuration values
        int[] canIDs = MK4i.getCanIDs(location);
        this.motor = motor;

        /*
         * Checks the motor device type:
         * 
         * if the motor device is NEO_VORTEX then it will define the NEOVortex objects
         * if the motor device is NEO_V1 then it will define the NEOv1 objects
         * if the motor device is neither NEO_VORTEX or NEO_V1 then it will report an error and disable the module's processes -
         * involving the motors.
         */
        if (motor == Motors.NEO_VORTEX) {

            //Defining the NEOVortex objects and Configuring the direction of the drive motor
            driveVortex = new NEOVortex(canIDs[0]);
            pivotVortex = new NEOVortex(canIDs[1]);
            driveVortex.setInverted(MK4i.getDriveMotorReversed(location, side_reversed));

            /*
            * Defining the relative encoders as the encoders of the NEOVortex's encoders
            * having a seperate RelativeEncoder object allows for us to change the conversion factors to get our desired units
            */
            driveEncoder = driveVortex.getEncoder();
            pivotEncoder = pivotVortex.getEncoder();

            // Enabling the module's processes involving the motors
            moduleEnabled = true;

        } else if (motor == Motors.NEO_V1) {

            //Defining the NEOv1 objects and Configuring the direction of the drive motor
            driveNEOv1 = new NEOv1(canIDs[0]);
            pivotNEOv1 = new NEOv1(canIDs[1]);
            driveNEOv1.setInverted(MK4i.getDriveMotorReversed(location, side_reversed));

            /*
            * Defining the relative encoders as the encoders of the NEOv1's encoders
            * having a seperate RelativeEncoder object allows for us to change the conversion factors to get our desired units
            */
            driveEncoder = driveNEOv1.getEncoder();
            pivotEncoder = pivotNEOv1.getEncoder();

            // Enabling the module's processes involving the motors
            moduleEnabled = true;

        } else {
            /* 
            * If the given motor device is not supported by this class it will report an error the the DriverStation 
            * and disable the module's processes involving the motor 
            */
            DriverStation.reportError(location.toString() + "Module motors provided are not REV Motors", true);
            moduleEnabled = false;
        }

        /*
         * Checks the AbsoluteEncoder device type:
         * if the absolute_encoder device is CAN_CODER it will define the CANcoder object
         * Otherwise it will disable the module and log an error to DriverStation
         * 
         * NOTE: Currently, Thrifty Encoders are not supported
         */
        if (abs_encoder == AbsoluteEncoder.CAN_CODER) {
            absoluteEncoder = new CANcoder(canIDs[3]);
        } else {
            DriverStation.reportError(location.toString() + "Error with provided Absolute Encoder", true);
            moduleEnabled = false;
        }

        //Defines the max mechanical speed given by the gear ratio and several physical aspects provided by SDS
        maxMechanicalSpeedMetersPerSecond = MK4i.getMaxSpeed(motor, gear_ratio);

        //Defines the Pivot Motion PID Controller with the given PIDConstants
        pivotController = new PIDController(
            pivot_controller_constants.kP, 
            pivot_controller_constants.kI, 
            pivot_controller_constants.kD
        );
    }

    //Returns the current driveEncoder's velocity in Meters/Second
    public double getDriveVelocity() {
        return moduleEnabled ? driveEncoder.getVelocity() : 0.0;
    }

    //Returns the current pivotEncoder's velocity in Radians/Second
    public double getPivotVelocity() {
        return moduleEnabled ? pivotEncoder.getVelocity() : 0.0;
    }

    //Returns the current driveEncoder's position in Meters
    public double getDrivePosition() {
        return moduleEnabled ? driveEncoder.getPosition() : 0.0;
    }

    //Returns the current pivotEncoder's position in Radians
    public double getPivotPosition() {
        return moduleEnabled ? pivotEncoder.getPosition() : 0.0;
    }

    //Returns the current absoluteEcnoder's absolute position in Radians
    public double getAbsolutePosition() {
        return moduleEnabled ? -absoluteEncoder.getAbsolutePosition().getValueAsDouble() : 0.0;
    }

    //Returns the current SwerveModuleState
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getPivotPosition()));
    }

    //Returns the current SwerveModulePosition
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getPivotPosition()));
    }

    //Use to set the desired output of the module 
    public void setDesiredState(SwerveModuleState desired_state) {

        //Discards miniscule speed values
        if (desired_state.speedMetersPerSecond < 0.001) {
            return;
        }

        //Uses an algorithm to minimize the distance traveled by the pivotMotor
        desired_state = SwerveModuleState.optimize(desired_state, new Rotation2d(getPivotPosition()));

        //Ensures the module is enabled before outputting to the motors
        if(moduleEnabled) {

            //Checks the Motor type to ensure proper output to the desired device
            switch (motor) {
                case FALCON_500:
                    return;
                case KRAKEN_X60:
                    return;
                case NEO_V1:
                    //Sets the output of the drive motor
                    driveNEOv1.set(desired_state.speedMetersPerSecond / maxMechanicalSpeedMetersPerSecond); 
                    //Sets the output of the pivot motor to a calculated value from a PID Controller
                    pivotNEOv1.set(pivotController.calculate(getPivotPosition(), desired_state.angle.getRadians()));
                case NEO_VORTEX:
                    //Sets the output of the drive motor
                    driveVortex.set(desired_state.speedMetersPerSecond / maxMechanicalSpeedMetersPerSecond);
                    //Sets the output of the pivot motor to a calculated value from a PID Controller
                    pivotVortex.set(pivotController.calculate(getPivotPosition(), desired_state.angle.getRadians()));
                default:
                    return;
            }

        } else { return; }
    }
} 