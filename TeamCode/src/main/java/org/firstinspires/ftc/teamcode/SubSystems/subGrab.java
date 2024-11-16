package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.opencv.core.Mat;

public class subGrab {

    // Arm motors
    public CRServo Lroller;
    public CRServo Rroller;
    public Servo rotator;

    public ColorRangeSensor colorSensor;

    public subGrab(HardwareMap hardwareMap) {

        //TODO: Setup configurations later
        Lroller = hardwareMap.get(CRServo.class, "LG");
        Rroller = hardwareMap.get(CRServo.class, "RG");
        rotator = hardwareMap.get(Servo.class, "RTR");

        colorSensor = hardwareMap.get(ColorRangeSensor.class, "CS");

    }

    public void intake(double pow) {
        Lroller.setPower(pow);
        Rroller.setPower(-pow);
    }

    public void outtake(double pow) {
        Lroller.setPower(-pow);
        Rroller.setPower(pow);
    }

    public void stop() {
        Lroller.setPower(0);
        Rroller.setPower(0);
    }

    public void setRotation(double ang){
        rotator.setPosition(ang);
    }

    public double rotateWrist(double y, double x) {

        double angleRadians = 0;

        if (y >= 0 && x >= 0){
            angleRadians = Math.atan2(Math.abs(y), Math.abs(x));
            angleRadians += (3 * Math.PI) / 2;
        }
        else if (y >= 0 && x <= 0){
            angleRadians = Math.atan2(Math.abs(y), Math.abs(x));
            angleRadians += Math.PI;
        }
        else if (y <= 0 && x <= 0){
            angleRadians = Math.atan2(Math.abs(y), Math.abs(x));
            angleRadians += Math.PI / 2;
        }
        else{
            angleRadians = Math.atan2(Math.abs(y), Math.abs(x));
        }

        return angleRadians;
    }
}
