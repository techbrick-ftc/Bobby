package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Mat;

public class subGrab {

    // Arm motors
    public CRServo Lroller;
    public CRServo Rroller;
    public Servo rotator;

    double distanceTarget = 5.75;
    int direction = 0;

    public subDrive drive;

    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;

    public subGrab(HardwareMap hardwareMap) {

        //TODO: Setup configurations later
        Lroller = hardwareMap.get(CRServo.class, "LG");
        Rroller = hardwareMap.get(CRServo.class, "RG");
        rotator = hardwareMap.get(Servo.class, "RTR");

        distanceSensor = hardwareMap.get(DistanceSensor.class, "CS");
        colorSensor = hardwareMap.get(ColorSensor.class, "CS");

        drive = new subDrive(hardwareMap);

    }

    public void intake(double pow) {
        Lroller.setPower(pow);
        Rroller.setPower(-pow);
        direction = 1;
    }

    public void outtake(double pow) {
        Lroller.setPower(-pow);
        Rroller.setPower(pow);
        direction = -1;
    }

    public void stop() {
        Lroller.setPower(0);
        Rroller.setPower(0);
        direction = 0;
    }

    public void checkObject(){
        double distance = distanceSensor.getDistance(DistanceUnit.CM);

        if (direction == 1 && distance <= distanceTarget){
            stop();
        }
        if (direction == -1 && distance >= distanceTarget){
            stop();
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
}
