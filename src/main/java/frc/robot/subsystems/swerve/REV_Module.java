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
import frc.montylib.swerve.vendor.SDS.MK4i.AbsoluteEncoder;
import frc.montylib.swerve.vendor.SDS.MK4i.GearRatio;
import frc.montylib.swerve.vendor.SDS.MK4i.ModulePosition;
import frc.montylib.swerve.vendor.SDS.MK4i.Motors;

public class REV_Module {
    private NEOVortex driveVortex, pivotVortex = null;
    private NEOv1 driveNEOv1, pivotNEOv1 = null;

    

    private RelativeEncoder driveEncoder, pivotEncoder = null;
    private CANcoder absoluteEncoder = null;

    public boolean moduleEnabled = false;
    public PIDController pivotController = null;
    private Motors motor = null;
    private double maxMechanicalSpeedMetersPerSecond = 0.0;
    public REV_Module() {}

    public REV_Module(Motors motor, AbsoluteEncoder abs_encoder, ModulePosition position, Side side_reversed, GearRatio gear_ratio, PIDConstants pivot_controller_constants) {
        int[] canIDs = MK4i.getCanIDs(position);
        this.motor = motor;

        if (motor == Motors.NEO_VORTEX) {

            driveVortex = new NEOVortex(canIDs[0]);
            pivotVortex = new NEOVortex(canIDs[1]);
            driveVortex.setInverted(MK4i.getDriveMotorReversed(position, side_reversed));

            driveEncoder = driveVortex.getEncoder();
            pivotEncoder = pivotVortex.getEncoder();

            moduleEnabled = true;

        } else if (motor == Motors.NEO_V1) {

            driveNEOv1 = new NEOv1(canIDs[0]);
            pivotNEOv1 = new NEOv1(canIDs[1]);
            driveNEOv1.setInverted(MK4i.getDriveMotorReversed(position, side_reversed));

            driveEncoder = driveNEOv1.getEncoder();
            pivotEncoder = pivotNEOv1.getEncoder();

            moduleEnabled = true;

        } else {
            DriverStation.reportError(position.toString() + "Module motors provided are not REV Motors", true);
            moduleEnabled = false;
        }

        if (abs_encoder == AbsoluteEncoder.CAN_CODER) {
            absoluteEncoder = new CANcoder(canIDs[3]);
        } else {
            DriverStation.reportError(position.toString() + "Error with provided Absolute Encoder", true);
            moduleEnabled = false;
        }

        maxMechanicalSpeedMetersPerSecond = MK4i.getMaxSpeed(motor, gear_ratio);

        pivotController = new PIDController(
            pivot_controller_constants.kP, 
            pivot_controller_constants.kI, 
            pivot_controller_constants.kD
        );
    }

    public double getDriveVelocity() {
        return moduleEnabled ? driveEncoder.getVelocity() : 0.0;
    }

    public double getPivotVelocity() {
        return moduleEnabled ? pivotEncoder.getVelocity() : 0.0;
    }

    public double getDrivePosition() {
        return moduleEnabled ? driveEncoder.getPosition() : 0.0;
    }

    public double getPivotPosition() {
        return moduleEnabled ? pivotEncoder.getPosition() : 0.0;
    }

    public double getAbsolutePosition() {
        return moduleEnabled ? -absoluteEncoder.getAbsolutePosition().getValueAsDouble() : 0.0;
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getPivotPosition()));
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getPivotPosition()));
    }

    public void setDesiredState(SwerveModuleState desired_state) {

        if (desired_state.speedMetersPerSecond < 0.001) {
            return;
        }

        desired_state = SwerveModuleState.optimize(desired_state, new Rotation2d(getPivotPosition()));

        if(moduleEnabled) {

            switch (motor) {
                case FALCON_500:
                    return;
                case KRAKEN_X60:
                    return;
                case NEO_V1:
                    driveNEOv1.set(desired_state.speedMetersPerSecond / maxMechanicalSpeedMetersPerSecond);
                    pivotNEOv1.set(pivotController.calculate(getPivotPosition(), desired_state.angle.getRadians()));
                case NEO_VORTEX:
                    driveVortex.set(desired_state.speedMetersPerSecond / maxMechanicalSpeedMetersPerSecond);
                    pivotVortex.set(pivotController.calculate(getPivotPosition(), desired_state.angle.getRadians()));
                default:
                    return;
            }

        } else { return; }
    }
} 
