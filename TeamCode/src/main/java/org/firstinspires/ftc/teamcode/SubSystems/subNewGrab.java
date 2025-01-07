package org.firstinspires.ftc.teamcode.SubSystems;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Date;

public class subNewGrab {

    // Arm motors
    public CRServo spinner;
    public Servo rot1;
    public Servo rot2;
    public Servo claw;

    double distanceTarget = 5.75;
    int direction = 0;

    float[] hsvValues = {0F, 0F, 0F};

    public static boolean distCheck;

    double distance;

    Date time = new Date();
    long initTime;
    int delayMS = 150;

    public ColorSensor colorSensor;
    public DistanceSensor distanceSensor;
    boolean detected = false;
    boolean lastDetected = false;

    public subNewGrab(HardwareMap hardwareMap) {

        //TODO: Setup configurations later
        spinner = hardwareMap.get(CRServo.class, "IN");
        rot1 = hardwareMap.get(Servo.class, "R1"); // Rotator for the wrist
        rot2 = hardwareMap.get(Servo.class, "R2");
        claw = hardwareMap.get(Servo.class, "CL");


        distanceSensor = hardwareMap.get(DistanceSensor.class, "CS");
        colorSensor = hardwareMap.get(ColorSensor.class, "CS");
    }

    public void spin(double pow) {
        spinner.setPower(pow);
    }

    public void stop() {
        spinner.setPower(0);
    }

    public boolean checkObjectIn(){
        distance = distanceSensor.getDistance(DistanceUnit.CM);
        time = new Date();

        distCheck = distance <= distanceTarget;

        if (distCheck){
            /*
            Color.RGBToHSV((int) (colorSensor.red()),
                    (int) (colorSensor.green()),
                    (int) (colorSensor.blue()),
                    hsvValues);
            hue = hsvValues[0];
            val = hsvValues[2];

            redCheck = (hue <= redAng1 + hueTol || hue >= redAng2 - hueTol);
            blueCheck = (hue >= blueAng - hueTol && hue <= blueAng + hueTol);

            if (val > .8)
            {

                // If red
                if (team && redCheck) {
                    detected = true;
                }
                // If blue
                else if (!team && blueCheck) {
                    detected = true;
                }
                // If yellow
                else if (hue >= yellowAng - hueTol & hue <= yellowAng + hueTol) {
                    detected = true;
                }
                else {
                    detected = false;
                }

                detected = true;
            }
            else{
                detected = false;
            }
             */
            detected = true;
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

    public boolean checkObjectOut(){
        double distance = distanceSensor.getDistance(DistanceUnit.CM);

        return distance >= distanceTarget;
    }

    public void setRotation(double ang){
        rot1.setPosition(ang);
        rot2.setPosition(ang);
    }

    public float[] getHSV(){
        Color.RGBToHSV((int) (colorSensor.red()),
                (int) (colorSensor.green()),
                (int) (colorSensor.blue()),
                hsvValues);
        return hsvValues;
    }

    public double getDistance(){
        return distance;
    }
}
