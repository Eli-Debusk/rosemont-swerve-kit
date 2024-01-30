package frc.montylib.hardware;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C.Port;

public class NavX2 extends AHRS {
    public NavX2() {
        super(Port.kMXP);
    }
}
