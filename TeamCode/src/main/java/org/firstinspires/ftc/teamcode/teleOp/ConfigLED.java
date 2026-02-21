package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

public class ConfigLED {
    public LED intakeLed;
    public LED transferLed;

    public void init(HardwareMap hwMap) {
        intakeLed = hwMap.get(LED.class,"intakeLed");
        transferLed = hwMap.get(LED.class,"transferLed");
    }

    public void setIntakeLed(boolean isOn){
        if (isOn){
            intakeLed.on();
        }
        else {
            intakeLed.off();
        }
    }

    public void setTransferLed(boolean isOn){
        if (isOn){
            transferLed.on();
        }
        else {
            transferLed.off();
        }
    }
}