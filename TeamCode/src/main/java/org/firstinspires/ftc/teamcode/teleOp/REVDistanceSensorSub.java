package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class REVDistanceSensorSub {
    public DistanceSensor sensorDistance;
    public Rev2mDistanceSensor sensorTimeOfFlight;

    public void init(HardwareMap hwMap) {
        sensorDistance = hwMap.get(DistanceSensor.class, "sensor_distance");

        sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;
    }

    public double getDistanceMM() {
        return sensorDistance.getDistance(DistanceUnit.MM);}
    public double getDistanceInches() {
        return sensorDistance.getDistance(DistanceUnit.INCH);}
    public boolean hasTimedOut() {
        return sensorTimeOfFlight.didTimeoutOccur();}

}