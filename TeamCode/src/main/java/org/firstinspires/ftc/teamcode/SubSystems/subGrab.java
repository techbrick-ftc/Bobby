package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

public class subGrab {

    // Arm motors
    public CRServo Lroller;
    public CRServo Rroller;
    public CRServo rotator;

    public ColorRangeSensor colorSensor;

    public subGrab(HardwareMap hardwareMap) {

        //TODO: Setup configurations later
        Lroller = hardwareMap.get(CRServo.class, "LG");
        Rroller = hardwareMap.get(CRServo.class, "RG");
        rotator = hardwareMap.get(CRServo.class, "RTR");

        colorSensor = hardwareMap.get(ColorRangeSensor.class, "CS");

    }

    public void intake() {
        Lroller.setPower(-1);
        Rroller.setPower(1);
    }

    public void outtake() {
        Lroller.setPower(1);
        Rroller.setPower(-1);
    }

    public void stop() {
        Lroller.setPower(0);
        Rroller.setPower(0);
    }
}
