package frc.robot.subsystems.swerve.constants;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.montylib.Chassis.Side;
import frc.montylib.swerve.vendor.SDS.MK4i.AbsoluteEncoder;
import frc.montylib.swerve.vendor.SDS.MK4i.GearRatio;
import frc.montylib.swerve.vendor.SDS.MK4i.ModuleLocation;
import frc.montylib.swerve.vendor.SDS.MK4i.Motors;
import frc.robot.subsystems.swerve.ModuleInterface;
import frc.robot.subsystems.swerve.ModuleInterfaceConstants;

public class SwerveConstants {
    //PID Constants for the pivot motor controller
    public static final PIDConstants modulePivotPIDConstants = new PIDConstants(0.5, 0.0, 0.05);

    //Module Interface Constants for all modules
    public static final ModuleInterfaceConstants moduleConstants = new ModuleInterfaceConstants(
        Motors.NEO_V1, 
        AbsoluteEncoder.CAN_CODER, 
        GearRatio.L1, 
        Side.RIGHT, 
        modulePivotPIDConstants
    );

    //Module Interface for each module
    public static final ModuleInterface leftFrontInterface = new ModuleInterface(ModuleLocation.LEFT_FRONT, moduleConstants);
    public static final ModuleInterface rightFrontInterface = new ModuleInterface(ModuleLocation.RIGHT_FRONT, moduleConstants);
    public static final ModuleInterface leftBackInterface = new ModuleInterface(ModuleLocation.LEFT_BACK, moduleConstants);
    public static final ModuleInterface rightBackInterface = new ModuleInterface(ModuleLocation.RIGHT_BACK, moduleConstants);

    //Kinematics Setup
    public static final double kTrackHorizDistanceFromCenter = Units.inchesToMeters(29) / 2;
    public static final double kTrackVertDistanceFromCenter = Units.inchesToMeters(29) / 2;

    //Kinematics Object
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kTrackHorizDistanceFromCenter, kTrackVertDistanceFromCenter),
        new Translation2d(-kTrackHorizDistanceFromCenter, kTrackVertDistanceFromCenter),
        new Translation2d(kTrackHorizDistanceFromCenter, -kTrackVertDistanceFromCenter),
        new Translation2d(-kTrackHorizDistanceFromCenter, -kTrackVertDistanceFromCenter)
    );
}
