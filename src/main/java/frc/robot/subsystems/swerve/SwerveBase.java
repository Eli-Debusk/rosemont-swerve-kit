package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.montylib.hardware.NavX2;
import frc.robot.subsystems.swerve.constants.SwerveConstants;

public class SwerveBase extends SubsystemBase{
    
    private REV_Module leftFrontModule = new REV_Module(SwerveConstants.leftFrontInterface);
    private REV_Module rightFrontModule = new REV_Module(SwerveConstants.rightFrontInterface);
    private REV_Module leftBackModule = new REV_Module(SwerveConstants.leftBackInterface);
    private REV_Module rightBackModule = new REV_Module(SwerveConstants.rightBackInterface);

    private NavX2 gyroscope = new NavX2();

    private SwerveDriveOdometry odometer = new SwerveDriveOdometry(
        SwerveConstants.kDriveKinematics, 
        getRotation2d(), 
        getModulePositions()
    );
    private Field2d fieldData = new Field2d();

    //GYROSCOPE AND ACCELEROMETER
    
    /**Resets the gyroscope Yaw (z-axis) value */
    public void resetHeading() {
        gyroscope.reset();
    }

    /**
     * Gets the current heading of the gyroscope relative to 360deg
     * @return the current heading
     */
    public double getHeading() {
        return Math.IEEEremainder(gyroscope.getAngle(), 360);
    }

    /**
     * Gets the current Rotation2d of the robot heading
     * @return getHeading() converted into a Rotation2d
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    //DRIVE WIDE UTILITY FUNCTION

    /**Stops module output */
    public void stopModules() {
        leftFrontModule.stopModule();
        rightFrontModule.stopModule();
        leftBackModule.stopModule();
        rightBackModule.stopModule();
    }

    /**Resets the pivot position of each module to the absolute encoders position */
    public void autoZeroModulePivots() {
        leftFrontModule.autoZeroPivotEncoder();
        rightFrontModule.autoZeroPivotEncoder();
        leftBackModule.autoZeroPivotEncoder();
        rightBackModule.autoZeroPivotEncoder();
    }

    /**Resets the encoders of each module */
    public void resetModuleEncoders() {
        leftFrontModule.resetEncoders();
        rightFrontModule.resetEncoders();
        leftBackModule.resetEncoders();
        rightBackModule.resetEncoders();
    }

    /**
     * Gets the ModuleStates in the form of an array 
     * @return SwerveModuleStates in an array (LEFTFRONT, RIGHTFRONT, LEFTBACK, RIGHTBACK)
     */
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            leftFrontModule.getModuleState(),
            rightFrontModule.getModuleState(),
            leftBackModule.getModuleState(),
            rightBackModule.getModuleState()
        };
    }

    //OUTPUT FUNCTIONS

    /**
     * Sets the output to the modules from the conversion of ChassisSpeeds to SwerveModuleState
     * @param speeds the ChassisSpeeds that are converted to SwerveModuleStates
     */
    public void setDesiredSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] desiredStates = SwerveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, leftFrontModule.maxMechanicalSpeedMetersPerSecond);

        leftFrontModule.setDesiredState(desiredStates[0]);
        rightFrontModule.setDesiredState(desiredStates[1]);

        leftBackModule.setDesiredState(desiredStates[2]);
        rightBackModule.setDesiredState(desiredStates[3]);
    }

    //ODOMETRY AND POSE

    /**
     * Gets the ModulePositions in the form of an array 
     * @return SwerveModulePosition in an array (LEFTFRONT, RIGHTFRONT, LEFTBACK, RIGHTBACK)
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            leftFrontModule.getModulePosition(),
            rightFrontModule.getModulePosition(),
            leftBackModule.getModulePosition(),
            rightBackModule.getModulePosition()
        };
    }
}
