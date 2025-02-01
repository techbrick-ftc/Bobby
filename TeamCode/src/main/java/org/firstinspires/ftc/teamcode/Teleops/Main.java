package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.subArm;
import org.firstinspires.ftc.teamcode.SubSystems.subDrive;
import org.firstinspires.ftc.teamcode.SubSystems.subGrab;
import org.firstinspires.ftc.teamcode.SubSystems.subHang;
import org.firstinspires.ftc.teamcode.SubSystems.subLED;


@TeleOp(name="0A TeleOp")
public class Main extends LinearOpMode {

    // Timer
    ElapsedTime tm1 = new ElapsedTime();


    boolean driveAllowed = true;
    public static double defPow = 1;
    public static double midPow = .6;
    double slowPow = 0.5;
    double slowerPow = 0.25;
    public static double drivePow = defPow;
    static boolean lockPow = false;
    static boolean slowMode = false;

    public static boolean isRedTeam = false;
    boolean ranInitCode = false;

    boolean slidesHome = true;
    boolean shouldHome = false;

    double y_move;
    double x_move;
    double rotation_x;

    public static boolean heldHeading = false;
    boolean g1start = false;
    boolean g2start = false;
    boolean lastG1Start = false;
    boolean lastG2Start = false;
    boolean endgame = false;

    boolean g1A = false;
    boolean lastG1A = false;

    public static double binsAngle = (5 * Math.PI) / 4;
    public static double wallAngle = Math.PI;
    public static double targetAngle = binsAngle;
    double angle;

    public static double defWristAngle = .5;

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

        waitForStart();
        tm1.startTime();

        while(opModeIsActive()) {
            if (!ranInitCode){
                grab.setWristRotation(defWristAngle);
                ranInitCode = true;
            }

            lastG1Start = g1start;
            lastG2Start = g2start;
            lastG1A = g1A;

            g1start = gamepad1.start;
            g2start = gamepad2.start;
            g1A = gamepad1.a;

            if (g2start && !lastG2Start){
                //slowMode = !slowMode;
                toggleColor();
            }
            if (g1start && !lastG1Start){
                if (!heldHeading){
                    toggleHeadingLock();
                }
                else{
                    toggleAngle();
                }
            }
            if (gamepad2.y){
                arm.zero();
            }
            if (gamepad2.dpad_up && gamepad2.right_bumper){
                drive.recalibrate();
            }
            if (gamepad2.x && gamepad2.left_bumper){
                routine = 7;
                endgame = true;
            }
            if (gamepad2.b){
                arm.stopAll();
                hang.stopLift();
            }
            if (endgame){
                hang.releaseSlides(gamepad2.a);
            }

            if (slowMode){
                speedMid();
            }
            else{
                speedHigh();
            }

            if (gamepad1.b) {
                arm.home();
                shouldHome = true;
                slidesHome = true;
                drivePow = defPow;
                grab.setWristRotation(defWristAngle);
            }

            if (routine == 0) {


                lockPow = false;

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
                        //routine = 4;
                    }
                }
                // Intake
                else if (g1A) {
                    routine = 5;
                    if (gamepad1.left_bumper) {
                        routine = 6;
                    }
                }
                else if (gamepad1.right_bumper){
                    routine = 8;
                }
            }
            else  {

                shouldHome = false;
                slidesHome = false;

                if (routine == 1) {
                    arm.highBin(gamepad1.x);
                }

                else if (routine == 2) {
                    arm.lowBin();
                }
                else if (routine == 3) {
                    arm.highBar(g1A);
                }
                else if (routine == 4) {
                    //arm.lowBar(g1A);
                }
                else if (routine == 5) {
                    arm.pitIntake(gamepad1.right_trigger);
                }
                else if (routine == 6) {
                    arm.wallIntake(g1A);
                }
                else if (routine == 7){
                    hang.altHang(gamepad2.x);
                }
                else if (routine == 8){
                    //arm.playerOuttake();
                }
            }

            if (Math.abs(gamepad2.left_stick_y) > .05 || Math.abs(gamepad2.right_stick_y) > .05){
                arm.manualShould(gamepad2.left_stick_y);
                arm.manualSlides(gamepad2.right_stick_y);
            }

            if (gamepad2.right_bumper) {
                if (gamepad2.dpad_down) {
                    arm.resetShoulder();
                    arm.resetSlides();
                }
                else if (gamepad2.dpad_right){
                    arm.resetSlides();
                }
                else if (gamepad2.dpad_left){
                    arm.resetShoulder();
                }
            }


            arm.grabberUpdate(gamepad1.right_trigger, gamepad1.left_trigger);

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

    void speedHigh(){
        drivePow = defPow;
    }

    void speedMid(){
        drivePow = midPow;
    }



    public static void activateSlowMode(){
        Main.slowMode = true;
    }

    public static void deactivateSlowMode(){
        Main.slowMode = false;
    }


    public void driveUpdate() {
        y_move = -gamepad1.left_stick_y;
        x_move = gamepad1.left_stick_x;

        rotation_x = gamepad1.right_stick_x * -1.0;

        if (Math.abs(rotation_x) > .05){
            deactivateHeadingLock();
        }

        if (driveAllowed) {

            if (heldHeading) {

                /*
                angle = (-drive.getImu() - targetAngle) % (2 * Math.PI);
                angle *= 2;

                drive.run(x_move, y_move, angle, drivePow);
                 */

                angle = -drive.getImu() + Math.PI - targetAngle;

                if (angle >= Math.PI){
                    angle -= 2*Math.PI;
                }
                if (angle <= -Math.PI){
                    angle += 2*Math.PI;
                }


                drive.run(x_move, y_move, angle, drivePow);

            }

            else {
                drive.run(x_move, y_move, rotation_x, drivePow);
            }
        }



        telemetry.addData("State: ", arm.state);
        telemetry.addData("Routine: ", routine);
        telemetry.addData("IMU: ", drive.getImu());
        telemetry.addData("Arm: ", arm.should.lSlides.getCurrentPosition());
        telemetry.addData("Shoulder: ", arm.should.shoulder.getCurrentPosition());
        telemetry.addData("Team (false = blue): ", isRedTeam);
        telemetry.addData("Distance: ", grab.getDistance());
        grab.updateHSV();
        telemetry.addData("Val: ", grab.getVal());
        telemetry.addData("Hue: ", grab.getHue());
        telemetry.update();
    }

    public static void toggleColor(){
        isRedTeam = !isRedTeam;
    }

    public static void toggleHeadingLock(){
        heldHeading = !heldHeading;
    }

    public static void activateHeadingLock(){
        heldHeading = true;
    }

    public static void deactivateHeadingLock(){
        heldHeading = false;
    }

    public static void toggleAngle(){
        if (targetAngle == wallAngle){
            targetAngle = binsAngle;
        }
        else{
            targetAngle = wallAngle;
        }
    }

    public static void setWallAngle(){
        targetAngle = wallAngle;
    }

    public static void setBinsAngle(){
        targetAngle = binsAngle;
    }


    // Blue Controller (controller 1)
    // X high baskets
    // Left Bumper + X for low basket
    // Y for high bar
    // Left Bumper + Y for low bar
    // A while at low bar to push downwards
    // B to return slides and shoulder to home
    // Left Bumper + B to return slides to home
    // Right Bumper + B to return shoulder to home
    // A to activate intake mode
    // A while intake is active to switch between higher and lower points
    // Triggers to activate intake wheels
    // D pad to control intake direction

    // Red Controller (controller 2)
    // Left bumper + X to initiate hang
    // X while hang is active to continue hang
    // A at endgame to start timer
    // B to stop all motors at their current position
    // Left stick to manually adjust slide length
    // Right stick to manually adjust shoulder pitch
    // Start to toggle color (default blue)
    // D pad up to reset IMU
    // D pad down to reset slides and shoulders



}