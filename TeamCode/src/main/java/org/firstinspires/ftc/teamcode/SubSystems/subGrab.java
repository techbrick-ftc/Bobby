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
    public CRServo lRoller;
    public CRServo rRoller;
    public Servo rotator;

    double distanceTarget = 5.75;
    int direction = 0;

    public subDrive drive;

    // false = blue, true = red

    double hue;
    double val;
    float[] hsvValues = {0F, 0F, 0F};
    double hueTol = 10;
    double yellowValTol = .8;
    double blueValTol = .4;
    double redValTol = .65;
    double redAng1 = 0;
    double redAng2 = 360;
    double blueAng = 220;
    double yellowAng = 40;

    Date time = new Date();
    long initTime;
    int delayMS = 150;

    public ColorSensor colorSensor;
    public DistanceSensor distanceSensor;
    boolean detected = false;
    boolean lastDetected = false;

    public subGrab(HardwareMap hardwareMap) {

        //TODO: Setup configurations later
        lRoller = hardwareMap.get(CRServo.class, "LG");
        rRoller = hardwareMap.get(CRServo.class, "RG");
        rotator = hardwareMap.get(Servo.class, "RTR");

        distanceSensor = hardwareMap.get(DistanceSensor.class, "CS");
        colorSensor = hardwareMap.get(ColorSensor.class, "CS");

        drive = new subDrive(hardwareMap);

    }

    public void intake(double pow) {
        lRoller.setPower(pow);
        rRoller.setPower(-pow);
        direction = 1;
    }

    public void outtake(double lPow, double rPow) {
        lRoller.setPower(-lPow);
        rRoller.setPower(rPow);
        direction = -1;
    }

    public void stop() {
        lRoller.setPower(0);
        rRoller.setPower(0);
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

    public void setRotation(double ang){
        rotator.setPosition(ang);
    }

    public void rotateWrist(double y, double x) {

        double stickAngle = 0;

        if (x != 0){
            stickAngle = Math.atan(Math.abs(y) / Math.abs(x));

            if (y >= 0 && x <= 0){
                stickAngle = Math.PI - stickAngle;
            }
            else if (y <= 0 && x <= 0){
                stickAngle = Math.PI + stickAngle;
            }
            else if (y <= 0 && x >= 0){
                stickAngle = (Math.PI * 2) - stickAngle;
            }
        }
        else if (y >= 0){
            stickAngle = Math.PI / 2;
        }
        else{
            stickAngle = (3 * Math.PI) / 2;
        }

        double robotAngle = drive.getImu();

        double wristAngle = (stickAngle - robotAngle - (Math.PI / 2)) % (Math.PI * 2);

        if (wristAngle < 0){
            wristAngle += Math.PI * 2;
        }

        wristAngle = Range.scale(wristAngle, 0, Math.PI * 2, 0.95, 0.05);

        setRotation(wristAngle);

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
