package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.SubSystems.subArm;
import org.firstinspires.ftc.teamcode.SubSystems.subDrive;
import org.firstinspires.ftc.teamcode.SubSystems.subGrab;

@TeleOp(name="Doesn't Matter")
public class Main extends LinearOpMode {

    // Timer
    ElapsedTime tm1 = new ElapsedTime();

    boolean driveAllowed = true;
    double defPow = 1;
    double slowPow = 0.5;
    double slowerPow = 0.25;
    double drivePow = defPow;

    double y_move;
    double x_move;
    double rotation_x;

    subDrive drive = null;


    @Override
    public void runOpMode() {

        drive = new subDrive(hardwareMap);
        subArm arm = new subArm(hardwareMap);
        subGrab grab = new subGrab(hardwareMap);

        waitForStart();
        tm1.startTime();

        while(opModeIsActive()) {

            if (gamepad1.b & gamepad1.right_trigger > 0.05) {
                arm.home();
            }

            else if (gamepad1.x) {
                arm.routine = 1;

                if (gamepad1.left_bumper) {
                    arm.routine = 2;
                }
            }

            else if (gamepad1.y) {
                arm.routine = 3;

                if (gamepad1.left_bumper) {
                    arm.routine = 4;
                }
            }

            if (arm.routine != 0) {

                drivePow = slowPow;

                if (arm.routine == 1) {
                    arm.bin(true);
                }

                else if (arm.routine == 2) {
                    arm.bin(false);
                }
            }

            else {
                arm.grabberUpdate(gamepad1.left_trigger, gamepad1.right_trigger);
            }

            driveUpdate();

        }
    }

    public void driveUpdate() {
        y_move = -gamepad1.left_stick_y;
        x_move = gamepad1.left_stick_x;

        rotation_x = gamepad1.right_stick_x * 1.0;

        if (driveAllowed) {
            drive.run(x_move, y_move, rotation_x, drivePow);
        }

        telemetry.addData("IMU: ", drive.getImu());
        telemetry.update();
    }



    // TODO: Controls
    // drive at full speed, automatically slow down at arm positions based on state
    // button to toggle between ground and up in input
    // return arm to raised height
    // button to return slides
    // X high baskets
    // Shift + X for low basket
    // Y for high chamber (bar)
    // Shift + Y for low chamber (bar)

    /* TODO: Control
        x - high basket
        lBump + x - low basket
        y - high bar
        lBump + y  - low bar
    */

}