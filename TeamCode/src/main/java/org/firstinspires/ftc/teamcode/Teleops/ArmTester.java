package org.firstinspires.ftc.teamcode.Teleops;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.subGrab;
import org.firstinspires.ftc.teamcode.SubSystems.subLift;
import org.firstinspires.ftc.teamcode.SubSystems.subShoulder;

@TeleOp(name="Arm Test")
public class ArmTester extends LinearOpMode {

    int extendDist = 0;
    int rotateDist = 0;

    double defGrabPow = 0.9;
    double defShPow = 0.5;
    double defLiftPow = 5.0;
    double ang = .5;

    subShoulder should = null;
    subLift lift = null;
    subGrab grab = null;

    float hsvValues[] = {0F, 0F, 0F};

    @Override
    public void runOpMode() {

        should = new subShoulder(hardwareMap);
        lift = new subLift(hardwareMap);
        grab = new subGrab(hardwareMap);

/*
Slides min = 0
Slides max = 2850

Pitch min = 0
Pitch max = 5000
 */

        waitForStart();

        while(opModeIsActive()){

            Color.RGBToHSV((int) (grab.colorSensor.red()),
                    (int) (grab.colorSensor.green()),
                    (int) (grab.colorSensor.blue()),
                    hsvValues);



            if (Math.abs(gamepad1.right_stick_y) > .05) {
                should.setSlidesOverride((int)(should.lSlides.getCurrentPosition() + -50 * gamepad1.right_stick_y), defShPow);
            }

            if (Math.abs(gamepad1.left_stick_y) > .05) {
                should.setShldOverride((int)(should.shoulder.getCurrentPosition() + -50 * gamepad1.left_stick_y), defShPow);
            }

            if (gamepad1.dpad_down){
                lift.runLiftOverride(defLiftPow, 0);
            }

            else if (gamepad1.dpad_up){
                lift.runLiftOverride(defLiftPow, 1);
            }
            else{
                lift.runLiftOverride(0, 2);
            }

            ang = (gamepad2.left_stick_x + 1) / 2;
            grab.setRotation(ang);

            if (gamepad1.right_bumper){
                grab.intake(defGrabPow);
                telemetry.addData("Intake", true);
            }
            else if (gamepad1.left_bumper){
                grab.outtake(defGrabPow, defGrabPow);
                telemetry.addData("Outtake", true);
            }
            else if (gamepad2.dpad_up){
                grab.rRoller.setPower(defGrabPow);
            }
            else if (gamepad2.dpad_down){
                grab.rRoller.setPower(-defGrabPow);
            }
            else if (gamepad2.dpad_left){
                grab.lRoller.setPower(defGrabPow);
            }
            else if (gamepad2.dpad_right){
                grab.lRoller.setPower(-defGrabPow);
            }
            else{
                grab.stop();
            }



            telemetry.addData("Right Slide Value: ", should.rSlides.getCurrentPosition());
            telemetry.addData("Left Slide Value: ", should.lSlides.getCurrentPosition());
            telemetry.addData("Shoulder Value: ", should.shoulder.getCurrentPosition());
            telemetry.addData("Lift Value: ", lift.lift.getCurrentPosition());
            telemetry.addData("Hue Value: ", hsvValues[0]);
            telemetry.addData("Saturation Value: ", hsvValues[1]);
            telemetry.addData("Value Value: ", hsvValues[2]);
            telemetry.update();
        }
    }
}
