package frc.montylib.profiles;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.montylib.MontyMath;
import frc.montylib.profiles.ConfigurableMotionProfile.VariableSpeedMode;

/**A MontyLib class to handle converting an anolog input to a motor output */
public class ConfigurableMotionProfile {
    private double maxMechanicalSpeed = 0;
    private boolean accelerationLimitingEnabled = false;
    private boolean variableSpeedEnabled = false;

    private dualSpeedMotionController dualSpeedController = null;
    private triSpeedMotionController triSpeedController = null;

    private SlewRateLimiter accelLimiter = null;

    public enum VariableSpeedMode {
        INCREASE,
        DECREASE,
        BOTH
    }
    
    /**Constructs a ConfigurableMotionProfile with default values */
    public ConfigurableMotionProfile() {}

    /**
     * Constructs a ConfigurableMotionProfile with basic config
     * @param max_mechanical_speed the maximum mechanical speed of the mechanism when the actuator's power is 100%
     * @param enable_acceleration_limiting whether acceleration should be limited or not
     * @param enable_variable_speed whether variable speed should be enabled or not
     */
    public ConfigurableMotionProfile(double max_mechanical_speed, boolean enable_acceleration_limiting, boolean enable_variable_speed) {
        this.maxMechanicalSpeed = max_mechanical_speed;
        this.accelerationLimitingEnabled = enable_acceleration_limiting;
        this.variableSpeedEnabled = enable_variable_speed;
    }

    /**
     * Constructs a ConfigurableMotionProfile with basic config (only use if you do not desire acceleration limiting or variable speed)
     * @param max_mechanical_speed the maximum mechanical speed of the mechanism when the actuator's power is 100%
     */
    public ConfigurableMotionProfile(double max_mechanical_speed) {
        this.maxMechanicalSpeed = max_mechanical_speed;
    }

    /**
     * Configures motor acceleration limit
     * @param acceleration the maximum change in speed per second
     */
    public void configureAcceleration(double acceleration) {
        this.accelLimiter = new SlewRateLimiter(acceleration);
    }

    /**
     * Configures dual-speed control (ONLY if variable-speed-control = true)
     * @param primary_speed the base speed when no modification is applied
     * @param secondary_speed the secondary achievable speed 
     */
    public void configureDualSpeedControl(double primary_speed, double secondary_speed) {
        this.dualSpeedController = new dualSpeedMotionController(primary_speed, secondary_speed);
    }

    /**
     * Configures tri-speed control (ONLY if variable-speed-control = true)
     * @param primary_speed the base speed when no modification is applied
     * @param secondary_speed the secondary achievable speed 
     * @param tertiary_speed the tertiary achievable speed
     */
    public void configureTriSpeedControl(double primary_speed, double secondary_speed, double tertiary_speed) {
        this.triSpeedController = new triSpeedMotionController(primary_speed, secondary_speed, tertiary_speed);
    }

    /**
     * SINGLE-SPEED MODE - function to return a motor output
     * @param value the analog imput (usually controller axis)
     * @param primary_speed the maximum speed this function calculates
     * @return a calculated motor output to acieve a desired velocity
     */
    public double calculate(double value, double primary_speed) {
        return accelerationLimitingEnabled ? 
            accelLimiter.calculate(value * (primary_speed)) / maxMechanicalSpeed : value * (primary_speed / maxMechanicalSpeed);
    }

    /**
     * DUAL-SPEED MODE - function to return a motor output
     * @param mode the variable speed control mode (INCEASE | DECREASE)
     * @param value the analog input (usually controller axis)
     * @param variable_speed_factor a positive analog input bridging the primary and secondary speeds
     * @return a calculated motor output to achieve a desired velocity
     */
    public double calculate(VariableSpeedMode mode, double value, double variable_speed_factor) {
        return variableSpeedEnabled ? 
            accelerationLimitingEnabled ? 
                accelLimiter.calculate(value * (dualSpeedController.calculate(mode, variable_speed_factor))) / maxMechanicalSpeed
                :
                value * (dualSpeedController.calculate(mode, variable_speed_factor) / maxMechanicalSpeed) 
        : 
        0.0;
    }

    /**
     * TRI-SPEED MODE - function to return a motor output
     * @param mode the variable speed control mode (INCEASE | DECREASE | BOTH)
     * @param value the analog input (usually controller axis)
     * @param primary_variable_speed_factor a positive analog input bridging the primary and secondary speeds
     * @param secondary_variable_speed_factor a positive analog input bridging the primary or secondary speeds to the tertiery speed
     * @return a calculated motor output to achieve a desired velocity
     */
    public double calculate(VariableSpeedMode mode, double value, double primary_variable_speed_factor, double secondary_variable_speed_factor) {
        return variableSpeedEnabled ? 
            accelerationLimitingEnabled ? 
                accelLimiter.calculate(value * (triSpeedController.calculate(mode, primary_variable_speed_factor, secondary_variable_speed_factor))) / maxMechanicalSpeed
                :
                value * (triSpeedController.calculate(mode, primary_variable_speed_factor, secondary_variable_speed_factor) / maxMechanicalSpeed)
        : 
        0.0;
    }
}
/** Handles dual-speed motion requests */
class dualSpeedMotionController {
    private double primarySpeed, secondarySpeed = 0;

    //Constructors
    public dualSpeedMotionController() {}

    public dualSpeedMotionController(double primary_speed, double secondary_speed) {
        this.primarySpeed = primary_speed;
        this.secondarySpeed = secondary_speed;
    }

    public double calculate(VariableSpeedMode mode, double factor) {
        switch (mode) {
            case INCREASE: 
                return primarySpeed + (MontyMath.valueDifference(primarySpeed, secondarySpeed) * factor);
            case DECREASE:  
                return primarySpeed - (MontyMath.valueDifference(primarySpeed, secondarySpeed) * factor);
            default: return primarySpeed;
        }
    }
}

/** Handles tri-speed motion requests*/
class triSpeedMotionController {
    double primarySpeed, secondarySpeed, tertiarySpeed = 0;

    //Constuctors
    public triSpeedMotionController() {}

    public triSpeedMotionController(double primary_speed, double secondary_speed, double tertiery_speed) {
        this.primarySpeed = primary_speed;
        this.secondarySpeed = secondary_speed;
        this.tertiarySpeed = tertiery_speed;
    }

    public double calculate(VariableSpeedMode mode, double factor1, double factor2) {
        switch (mode) {
            case INCREASE: 
                return primarySpeed 
                    + (MontyMath.valueDifference(primarySpeed, secondarySpeed) * factor1)
                    + (MontyMath.valueDifference(secondarySpeed, tertiarySpeed) * factor2);
            case DECREASE:
                return primarySpeed 
                        - (MontyMath.valueDifference(primarySpeed, secondarySpeed) * factor1)
                        - (MontyMath.valueDifference(secondarySpeed, tertiarySpeed) * factor2);
            case BOTH:
                return primarySpeed 
                    + (MontyMath.valueDifference(primarySpeed, secondarySpeed) * factor1)
                    - (MontyMath.valueDifference(primarySpeed, tertiarySpeed) * factor2);
            default: return primarySpeed;
        }
    }
}