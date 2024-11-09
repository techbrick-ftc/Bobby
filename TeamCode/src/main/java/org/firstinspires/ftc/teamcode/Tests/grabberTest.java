package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SubSystems.subGrab;

@TeleOp (name="Grabber Test")
public class grabberTest extends LinearOpMode {

    private ColorRangeSensor colorSensor;
    private double redValue;
    private double greenValue;
    private double blueValue;
    private double alphaValue;
    private double dist;

    int color = 0;

    private double distThresh = 2;

    subGrab grab = null;

    public void readSensor()  {
        redValue = colorSensor.red();
        greenValue = colorSensor.green();
        blueValue = colorSensor.blue();
        alphaValue = colorSensor.alpha();
        dist = colorSensor.getDistance(DistanceUnit.MM);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        grab = new subGrab(hardwareMap);
        colorSensor = hardwareMap.get(ColorRangeSensor.class, "CS");

        waitForStart();

        while (opModeIsActive()) {

            readSensor();

            if (redValue > 60) {
                color = 1; // Red
            }

            // setting the grabber servos based on gamepad input
            if (gamepad2.right_bumper && dist < distThresh) {
                grab.intake(0.5);
            }

            else if (gamepad2.left_bumper || (dist < distThresh && color == 1)) {
                grab.outtake(0.5);
            }

            else {
                grab.stop();
            }

            redValue = colorSensor.red();
            telemetry.addData("Red", redValue);
            telemetry.addData("Blue", blueValue);
            telemetry.addData("Green", greenValue);
            telemetry.addData("Dist, CM", dist);
            telemetry.addData("Alpha", alphaValue);

            telemetry.update();
        }
    }
}