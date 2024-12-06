package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.subArm;
import org.firstinspires.ftc.teamcode.SubSystems.subDrive;
import org.firstinspires.ftc.teamcode.SubSystems.subGrab;
import org.firstinspires.ftc.teamcode.SubSystems.subHang;

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

    boolean slidesHome = true;
    boolean shouldHome = false;

    double y_move;
    double x_move;
    double rotation_x;

    boolean heldHeading = false;
    boolean g1start = false;
    boolean g2start = false;
    boolean lastG1Start = false;
    boolean lastG2Start = false;
    boolean endgame = false;

    double angleCorrection = (5 * Math.PI) / 4;
    double angle;

    public static double defWristRotate = .5;

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

        grab.setRotation(defWristRotate);

        waitForStart();
        tm1.startTime();

        while(opModeIsActive()) {

            lastG1Start = g1start;
            lastG2Start = g2start;

            g1start = gamepad1.start;
            g2start = gamepad2.start;

            if (g1start && !lastG1Start){
                slowMode = !slowMode;
            }
            if (g2start && !lastG2Start){
                grab.toggleColor();
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
                    drivePow = defPow;
                }
                grab.setRotation(defWristRotate);
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
                    arm.highBar(gamepad1.a);
                }
                else if (routine == 4) {
                    arm.lowBar(gamepad1.a);
                }
                else if (routine == 5) {
                    // arm.pitIntakeFine(gamepad1.a, gamepad1.left_stick_x, gamepad1.left_stick_y);
                    arm.pitIntakeCoarse(gamepad1.a, gamepad1.dpad_left, gamepad1.dpad_right, gamepad1.dpad_up, gamepad1.dpad_down);
                }
                else if (routine == 6) {
                    arm.wallIntake();
                }
                else if (routine == 7){
                    hang.altHang(gamepad2.x);
                }
            }

            if (Math.abs(gamepad2.left_stick_y) > .05 || Math.abs(gamepad2.right_stick_y) > .05){
                routine = 0;
                arm.state = 0;

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

        if (driveAllowed) {
            if (heldHeading) {
                angle = (drive.getImu() - angleCorrection) % (2 * Math.PI);
                if (angle < 0){
                    angle += 2 * Math.PI;
                }
                angle = angle / (2 * Math.PI);

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
        telemetry.addData("Team (false = blue): ", grab.getColor());
        telemetry.addData("Timer started: ", hang.getTimerStarted());
        telemetry.addData("Time checking: ", hang.getEndTime());
        telemetry.update();
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