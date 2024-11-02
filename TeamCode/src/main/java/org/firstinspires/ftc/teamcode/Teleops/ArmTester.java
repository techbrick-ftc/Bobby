package org.firstinspires.ftc.teamcode.Teleops;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.subGrab;
import org.firstinspires.ftc.teamcode.SubSystems.subLift;
import org.firstinspires.ftc.teamcode.SubSystems.subShoulder;

@TeleOp(name="Arm Test")
public class ArmTester extends LinearOpMode {

    int extendDist = 0;
    int rotateDist = 0;

    double defGrabPow = .5;
    double defShPow = 0.5;
    double defLiftPow = 5.0;

    subShoulder should = null;
    subLift lift = null;
    subGrab grab = null;

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
            if (Math.abs(gamepad1.right_stick_y) > .05) {
                should.setSlides((int)(should.lSlides.getCurrentPosition() + -50 * gamepad1.right_stick_y), defShPow);
            }

            if (Math.abs(gamepad1.left_stick_y) > .05) {
                should.setShld((int)(should.shoulder.getCurrentPosition() + -50 * gamepad1.left_stick_y), defShPow);
            }

            if (gamepad1.dpad_down){
                lift.runLift(defLiftPow, 0);
            }
            else if (gamepad1.dpad_up){
                lift.runLift(defLiftPow, 1);
            }
            else{
                lift.runLift(0, 0);
            }

            if (gamepad1.right_bumper){
                grab.intake(defGrabPow);
                telemetry.addData("Intake", true);
            }
            else if (gamepad1.left_bumper){
                grab.outtake(defGrabPow);
                telemetry.addData("Outtake", true);
            }
            else{
                grab.stop();
            }

            telemetry.addData("Right Slide Value: ", should.rSlides.getCurrentPosition());
            telemetry.addData("Left Slide Value: ", should.lSlides.getCurrentPosition());
            telemetry.addData("Shoulder Value: ", should.shoulder.getCurrentPosition());
            telemetry.addData("Lift Value: ", lift.lift.getCurrentPosition());

            telemetry.update();
        }


    }
}
