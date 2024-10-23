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

    public void intake(double pow) {
        Lroller.setPower(-pow);
        Rroller.setPower(pow);
    }

    public void outtake(double pow) {
        Lroller.setPower(pow);
        Rroller.setPower(-pow);
    }

    public void stop() {
        Lroller.setPower(0);
        Rroller.setPower(0);
    }
}
