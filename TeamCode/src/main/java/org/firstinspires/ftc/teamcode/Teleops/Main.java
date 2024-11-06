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
import org.firstinspires.ftc.teamcode.SubSystems.subLift;

@TeleOp(name="Doesn't Matter")
public class Main extends LinearOpMode {

    // Timer
    ElapsedTime tm1 = new ElapsedTime();

    boolean driveAllowed = true;
    double defPow = 1;
    double slowPow = 0.5;
    double slowerPow = 0.25;
    double drivePow = defPow;

    boolean slidesHome = true;
    boolean shouldHome = false;

    double y_move;
    double x_move;
    double rotation_x;

    subDrive drive = null;;
    subArm arm = null;
    subGrab grab = null;
    subLift lift = null;



    @Override
    public void runOpMode() {

        drive = new subDrive(hardwareMap);
        arm = new subArm(hardwareMap);
        grab = new subGrab(hardwareMap);
        lift = new subLift(hardwareMap);

        grab.setRotation(.5);

        waitForStart();
        tm1.startTime();

        while(opModeIsActive()) {

            if (gamepad1.b) {
                if (gamepad1.left_bumper){
                    arm.homeSlides();
                    slidesHome = true;
                }
                else if (gamepad1.right_bumper){
                    arm.homeShould();
                    shouldHome = true;
                }
                else{
                    arm.home();
                    shouldHome = true;
                    slidesHome = true;
                }
            }
            if (arm.routine == 0) {
                // Bins
                if (gamepad1.x) {
                    arm.routine = 1;

                    if (gamepad1.left_bumper) {
                        arm.routine = 2;
                    }
                }
                // Bars
                else if (gamepad1.y) {
                    arm.routine = 3;

                    if (gamepad1.left_bumper) {
                        arm.routine = 4;
                    }
                }
                // Intake
                else if (gamepad1.a) {
                    arm.routine = 5;
                    if (gamepad1.left_bumper) {
                        arm.routine = 6;
                    }
                }
            }

            if (arm.routine != 0) {

                drivePow = slowPow;

                shouldHome = false;
                slidesHome = false;

                if (arm.routine == 1) {
                    arm.bin(true);
                }

                else if (arm.routine == 2) {
                    arm.bin(false);
                }
                else if (arm.routine == 3) {
                    arm.highBar();
                }
                else if (arm.routine == 4) {
                    arm.lowBar(gamepad2.a);
                }
                else if (arm.routine == 5) {
                    arm.pitIntake(gamepad2.a);
                }
                else if (arm.routine == 6) {
                    arm.wallIntake();
                }

                arm.manualShould(gamepad2.left_stick_y);
                arm.manualSlides(gamepad2.right_stick_y);
            }

            arm.grabberUpdate(gamepad1.left_trigger, gamepad1.right_trigger);

            driveUpdate();

            if (slidesHome){
                if (arm.updateSlideHome()){
                    slidesHome = false;
                }
            }
            if (shouldHome){
                if (arm.updateShouldHome()){
                    shouldHome = false;
                }
            }


        }
    }

    public void driveUpdate() {
        y_move = -gamepad1.left_stick_y;
        x_move = gamepad1.left_stick_x;

        rotation_x = gamepad1.right_stick_x * -1.0;

        if (driveAllowed) {
            drive.run(x_move, y_move, rotation_x, drivePow);
        }

        telemetry.addData("State: ", arm.state);
        telemetry.addData("Intake Up: ", arm.intakeUp);
        telemetry.addData("IMU: ", drive.getImu());
        telemetry.addData("Routine", arm.routine);
        telemetry.update();
    }



    // TODO: Controls
    // drive at full speed, automatically slow down at arm positions based on state
    // button to toggle between ground and up in input
    // return arm to raised height
    // button to return slides
    // X high baskets
    // Left Bumper + X for low basket
    // Y for high chamber (bar)
    // Left Bumper + Y for low chamber (bar)
    // B to return slides and shoulder to home
    // Left Bumper + B to return slides to home
    // Right Bumper + B to return shoulder to home
    // A to activate intake
    // A while intake is active to switch between higher and lower points

    /* TODO: Control
        x - high basket
        lBump + x - low basket
        y - high bar
        lBump + y  - low bar
    */

}