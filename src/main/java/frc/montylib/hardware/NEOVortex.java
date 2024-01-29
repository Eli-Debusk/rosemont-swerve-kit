package frc.montylib.hardware;

import com.revrobotics.CANSparkFlex;

public class NEOVortex extends CANSparkFlex{
    public NEOVortex(int deviceId) {
        super(deviceId, MotorType.kBrushless);
    }
}
