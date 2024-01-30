package frc.robot.subsystems.swerve;

import com.pathplanner.lib.util.PIDConstants;

import frc.montylib.Chassis.Side;
import frc.montylib.swerve.vendor.SDS.MK4i.*;

public class ModuleInterfaceConstants {
    public Motors _motor = null;
    public AbsoluteEncoder _encoder = null;
    public GearRatio _gearRatio = null;
    public Side _invertedSide = null;
    public PIDConstants _pivotControllerConstants = null;

    public ModuleInterfaceConstants(
       Motors motor,
       AbsoluteEncoder encoder,
       GearRatio module_gear_ratio,
       Side inverted_side,
       PIDConstants pivot_controller_constants
   ) {
     this._motor = motor;
     this._encoder = encoder;
     this._gearRatio = module_gear_ratio;
     this._invertedSide = inverted_side;
     this._pivotControllerConstants = pivot_controller_constants;
   }
}
