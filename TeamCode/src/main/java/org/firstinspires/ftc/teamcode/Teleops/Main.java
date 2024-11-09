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
import org.firstinspires.ftc.teamcode.SubSystems.subHang;
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

    public static int routine = 0;

    subDrive drive = null;;
    subArm arm = null;
    subGrab grab = null;
    subHang hang = null;



    @Override
    public void runOpMode() {

        drive = new subDrive(hardwareMap);
        arm = new subArm(hardwareMap);
        grab = new subGrab(hardwareMap);
        hang = new subHang(hardwareMap);

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

            if (routine == 0) {
                // Bins
                if (gamepad1.x) {
                    routine = 1;

                    if (gamepad1.left_bumper) {
                        routine = 2;
                    }
                }
                // Bars
                else if (gamepad1.y) {
                    routine = 3;

                    if (gamepad1.left_bumper) {
                        routine = 4;
                    }
                }
                // Intake
                else if (gamepad1.a) {
                    routine = 5;
                    if (gamepad1.left_bumper) {
                        routine = 6;
                    }
                }
                // Hang
                else if (gamepad2.x && gamepad2.left_bumper){
                    routine = 7;
                }
            }
            else  {

                drivePow = slowPow;

                shouldHome = false;
                slidesHome = false;

                if (routine == 1) {
                    arm.bin(true);
                }

                else if (routine == 2) {
                    arm.bin(false);
                }
                else if (routine == 3) {
                    arm.highBar();
                }
                else if (routine == 4) {
                    arm.lowBar(gamepad1.a);
                }
                else if (routine == 5) {
                    arm.pitIntake(gamepad1.a);
                }
                else if (routine == 6) {
                    arm.wallIntake();
                }
                else if (routine == 7){
                    hang.hang(gamepad2.x, gamepad2.b);
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
        telemetry.addData("Routine", routine);
        telemetry.update();
    }


    // Blue Controller
    // X high baskets
    // Left Bumper + X for low basket
    // Y for high chamber (bar)
    // Left Bumper + Y for low chamber (bar)
    // B to return slides and shoulder to home
    // Left Bumper + B to return slides to home
    // Right Bumper + B to return shoulder to home
    // A to activate intake
    // A while intake is active to switch between higher and lower points or while at low bar to push downwards

    // Red Controller
    // Left bumper + X to initiate hang
    // A while hang is initialized to activate
    // B while hang is initialized to cancel


}