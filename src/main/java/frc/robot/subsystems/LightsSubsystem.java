package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LightsSubsystem extends SubsystemBase {

    CANdle candle;

    public LightsSubsystem() {
        candle = new CANdle(Constants.CANDLE_ID, Constants.CANIVORE_CAN_BUS_NAME);
        candle.configLEDType(CANdle.LEDStripType.RGB);
        candle.configBrightnessScalar(0.5);
    }

    public void turnOnYellow() {
        System.out.println("yellow");
        candle.setLEDs(255, 255, 0);
        System.out.println(candle.getTemperature());
    }

    public void turnOnPurple() {
        System.out.println("purple");
        candle.setLEDs(196, 0, 255);
    }

    public void turnOff() {
        System.out.println("off");
        candle.setLEDs(0, 0, 0);
    }

}
