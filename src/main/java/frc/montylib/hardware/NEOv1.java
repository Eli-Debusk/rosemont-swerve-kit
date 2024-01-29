package frc.montylib.hardware;

import com.revrobotics.CANSparkMax;

/** For the sake of JAVA class names NEOv1 represents the NEO Brushless Motor v1.1 */
public class NEOv1 extends CANSparkMax {

    public NEOv1(int deviceId) {
        super(deviceId, MotorType.kBrushless);
    }
    
}
