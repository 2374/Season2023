package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LightsSubsystem extends SubsystemBase {

    CANdle candle;

    public LightsSubsystem() {
        candle = new CANdle(Constants.CANDLE_ID, Constants.CANIVORE_CAN_BUS_NAME);
    }

    public void turnOnYellow() {
        candle.setLEDs(245, 255, 12);
    }

    public void turnOnPurple() {
        candle.setLEDs(196, 0, 255);
    }

    public void turnOff() {
        candle.setLEDs(0, 0, 0);
    }

}
