package frc.robot.subsystems.swerve;

import com.pathplanner.lib.util.PIDConstants;

import frc.montylib.Chassis.Side;
import frc.montylib.swerve.vendor.SDS.MK4i.*;

public class ModuleInterface {
    public Motors selectedMotor = null;
    public AbsoluteEncoder selectedEncoder = null;
    public ModuleLocation moduleLocation = null;
    public GearRatio moduleDriveGearRatio = null;
    public Side invertedSide = null;
    public PIDConstants pivotPIDConstants = null;

    public ModuleInterface(ModuleLocation location, ModuleInterfaceConstants constants) {
        this.moduleLocation = location;
        this.selectedMotor = constants._motor;
        this.selectedEncoder = constants._encoder;
        this.moduleDriveGearRatio = constants._gearRatio;
        this.invertedSide = constants._invertedSide;
        this.pivotPIDConstants = constants._pivotControllerConstants;
    }
}
