package org.firstinspires.ftc.teamcode.SubSystems;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Teleops.Main;

import java.util.Date;

public class subGrab {

    // Arm motors
    public CRServo intake;
    public Servo wrist1;
    public Servo wrist2;
    public Servo claw;

    double distanceTarget = 1.5;
    int direction = 0;

    // false = blue, true = red

    double hue;
    double val;
    float[] hsvValues = {0F, 0F, 0F};
    double hueTol = 10;
    double yellowValTol = .8;
    double blueValTol = .4;
    double redValTol = .65;
    double redAng1 = 20;
    double redAng2 = 20;
    double blueAng = 220;
    double yellowAng = 75;

    double grabAngle = .5;
    double lightGrabAngle = .52;
    double releaseAngle = .7;

    Date time = new Date();
    long initTime;
    int delayMS = 150;

    public ColorSensor colorSensor;
    public DistanceSensor distanceSensor;
    boolean detected = false;
    boolean lastDetected = false;

    public subGrab(HardwareMap hardwareMap) {

        intake = hardwareMap.get(CRServo.class, "IN");
        wrist1 = hardwareMap.get(Servo.class, "W1");
        wrist2 = hardwareMap.get(Servo.class, "W2");
        claw = hardwareMap.get(Servo.class, "CL");

        distanceSensor = hardwareMap.get(DistanceSensor.class, "CS");
        colorSensor = hardwareMap.get(ColorSensor.class, "CS");

    }

    public void intake(double pow) {
        intake.setPower(-pow);
        direction = 1;
    }

    public void outtake(double pow) {
        intake.setPower(pow);
        direction = -1;
    }


    public void grab(){
        claw.setPosition(grabAngle);
    }

    public void lightGrab(){
        claw.setPosition(lightGrabAngle);
    }

    public void release(){
        claw.setPosition(releaseAngle);
    }

    public void stop() {
        intake.setPower(0);
        direction = 0;
    }

    public boolean checkObjectIn(){
        time = new Date();

        if (distanceCheck()){
            updateHSV();
            hue = getHue();
            val = getVal();

            // If red
            if (Main.isRedTeam && redCheck() && val >= redValTol) {
                detected = true;
            }
            // If blue
            else if (!Main.isRedTeam && blueCheck() && val >= blueValTol) {
                detected = true;
            }
            // If yellow
            else if (yellowCheck() && val >= yellowValTol) {
                detected = true;
            }
            else {
                detected = false;
            }
        }
        else{
            detected = false;
        }

        if (!detected || !lastDetected){
            initTime = time.getTime();
        }
        lastDetected = detected;

        if (detected && time.getTime() - initTime >= delayMS){
            return true;
        }
        else {
            return false;
        }
    }

    public void setWristRotation(double ang){
        wrist1.setPosition(ang);
        wrist2.setPosition(1-ang);
    }

    public void updateHSV(){
        Color.RGBToHSV((int) (colorSensor.red()),
                (int) (colorSensor.green()),
                (int) (colorSensor.blue()),
                hsvValues);
    }

    public double getDistance(){
        return distanceSensor.getDistance(DistanceUnit.CM);
    }

    public boolean getDetected(){
        return detected;
    }

    public boolean distanceCheck(){
        return getDistance() <= distanceTarget;
    }

    public boolean redCheck(){
        return (hue <= redAng1 + hueTol || hue >= redAng2 - hueTol);
    }

    public boolean blueCheck(){
        return (hue >= blueAng - hueTol && hue <= blueAng + hueTol);
    }

    public boolean yellowCheck(){
        return (hue >= yellowAng - hueTol && hue <= yellowAng + hueTol);
    }

    public double getHue(){
        return hsvValues[0];
    }

    public double getVal(){
        return hsvValues[2];
    }
}
