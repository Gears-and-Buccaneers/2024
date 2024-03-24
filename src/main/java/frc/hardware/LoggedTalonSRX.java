package frc.hardware;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class LoggedTalonSRX extends TalonSRX implements LoggedHardware {

    public LoggedTalonSRX(int deviceNumber) {
        super(deviceNumber);
        // TODO Auto-generated constructor stub
    }

    @Override
    public void log() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'log'");
    }

}
