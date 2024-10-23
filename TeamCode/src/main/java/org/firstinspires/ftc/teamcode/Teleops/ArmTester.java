package org.firstinspires.ftc.teamcode.Teleops;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.subShoulder;

@TeleOp(name="Arm Test")
public class ArmTester extends LinearOpMode {

    subShoulder should = new subShoulder(hardwareMap);
    int extendDist = 0;
    int rotateDist = 0;

    double defShPow = 0.5;

    @Override
    public void runOpMode() {
        if (Math.abs(gamepad1.right_stick_y) > .05) {
            should.setShld((int)(should.lSlides.getCurrentPosition() + 50 * gamepad1.right_stick_y), defShPow);
        }

        if (Math.abs(gamepad1.left_stick_y) > .05) {
            should.setShld((int)(should.shoulder.getCurrentPosition() + 50 * gamepad1.left_stick_y), defShPow);
        }

        telemetry.addData("Slide Value: ", should.lSlides.getCurrentPosition());
        telemetry.addData("Shoulder Value: ", should.shoulder.getCurrentPosition());
    }
}
