package org.firstinspires.ftc.teamcode.Autos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.SubSystems.subAutoGrab;
import org.firstinspires.ftc.teamcode.SubSystems.subGrab;

@Config
@Autonomous(name = "MecInAuto")
public class MecInAutoMoves extends LinearOpMode {

    //TODO: Shoulder motor initializations
    DcMotorEx shoulder;
    DcMotorEx lSlides;
    DcMotorEx rSlides;

    int firstY = -42;
    int secondY = -52;
    int thirdY = -60;

    int pushIn = -54;
    int farOut = -12;

    //TODO: Tune robot positions here
    // x, y, heading
    double[] firstMove = {-50, 2, Math.toRadians(180)};
    double[] toScore = {-29, 2, Math.toRadians(180)};
    double[] clearPos = {-36, 2, Math.toRadians(180)};
    double[] firsty = {-36, -30, Math.toRadians(180)};
    double[] second = {farOut, -30, Math.toRadians(180)};
    double[] third = {farOut, firstY, Math.toRadians(180)};
    double[] fourth = {pushIn, firstY, Math.toRadians(180)};
    double[] fifth = {farOut, firstY, Math.toRadians(180)};
    double[] sixth = {farOut, secondY, Math.toRadians(180)};
    double[] seventh = {pushIn, secondY, Math.toRadians(180)};
    double[] eigth = {farOut, secondY, Math.toRadians(180)};
    double[] ninth = {farOut, thirdY, Math.toRadians(180)};
    double[] tenth = {pushIn, thirdY, Math.toRadians(180)};

    double[] wallInit = {-50, -28, Math.toRadians(180)};
    double[] wallNext = {-57, -28, Math.toRadians(180)};

    //TODO: Tune arm positions here
    // pitch, slides
    int[] ready = {1800, 10};
    int[] barInit = {3170, 160};
    int[] barRaise = {3170, 770};
    int[] gGrab1 = {20, 300};
    int[] gGrab2 = {20, 200};
    int[] gGrab3 = {20, 600};
    int[] stash = {20, 1000};
    int[] wall = {1000, 10};

    //TODO: Grab pows
    double inAng = 0.34;
    double upAng = 0.7;
    double depoAng = 0.44;
    double wallInAng = 0.09;

    DcMotorEx front_Left;
    DcMotorEx front_Right;
    DcMotorEx back_Left;
    DcMotorEx back_Right;

    subGrab grab;

    public class AutoArm {

        public AutoArm(HardwareMap hardwareMap) {

            //TODO: Shoulder motor declarations
            shoulder = hardwareMap.get(DcMotorEx.class, "AR");
            lSlides = hardwareMap.get(DcMotorEx.class, "SL");
            rSlides = hardwareMap.get(DcMotorEx.class, "SR");

            shoulder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            lSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            shoulder.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            shoulder.setDirection(DcMotorEx.Direction.REVERSE);
            lSlides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rSlides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            lSlides.setDirection(DcMotorEx.Direction.REVERSE);

            grab = new subGrab(hardwareMap);
        }

        //Drive arm to position
        private void armToPos(int pitch, int slide) {

            // Setting position
            shoulder.setTargetPosition(pitch);
            lSlides.setTargetPosition(slide);
            rSlides.setTargetPosition(slide);

            // Setting power
            shoulder.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            lSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            rSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            shoulder.setPower(1.0);
            lSlides.setPower(1.0);
            rSlides.setPower(1.0);

        }

        private boolean reached(int tol) {
            return Math.abs(lSlides.getCurrentPosition() - lSlides.getTargetPosition()) < tol && Math.abs(shoulder.getCurrentPosition() - shoulder.getTargetPosition()) < tol;
        }

        //TODO: No grabber code included here

        public class ReadyPos implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armToPos(ready[0], ready[1]);
                return !reached(20);
            }
        }
        public Action readyPos() {
            return new ReadyPos();
        }

        public class BarInit implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.lightGrab();
                armToPos(barInit[0], barInit[1]);
                grab.setWristRotation(depoAng);
                return !reached(20);
            }
        }
        public Action barInit() {
            return new BarInit();
        }

        public class BarScore implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.grab();
                armToPos(barRaise[0], barRaise[1]);
                grab.setWristRotation(depoAng);
                return !reached(20);
            }
        }
        public Action barScore() {
            return new BarScore();
        }

        public class PrepWall implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.setWristRotation(wallInAng);
                grab.release();
                armToPos(wall[0], wall[1]);
                return !reached(20);
            }
        }
        public Action prepWall() {
            return new PrepWall();
        }

        public class MovePos implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armToPos(wall[0] + 500, wall[1]);
                return !reached(20);
            }
        }
        public Action movePos() {
            return new MovePos();
        }

        public class Release implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.release();
                return false;
            }
        }
        public Action release() {
            return new Release();
        }

        public class Grab implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.grab();
                return false;
            }
        }
        public Action grab() {
            return new Grab();
        }

    }

    public class MecDrive {

        public MecDrive (HardwareMap hardwareMap) {
            back_Right = hardwareMap.get(DcMotorEx.class, "BRM");
            back_Left = hardwareMap.get(DcMotorEx.class, "BLM");
            front_Right = hardwareMap.get(DcMotorEx.class, "FRM");
            front_Left = hardwareMap.get(DcMotorEx.class, "FLM");
        }

        public void run(double x_move, double y_move, double rotation_x, double heading) {

            if (Math.abs(x_move) > 0.05 || Math.abs(y_move) > 0.05 || Math.abs(rotation_x) > 0.05) {
                // Orientation
                double angle = heading;

                // Calculating power variables
                double x = x_move * Math.cos(-(angle)) - y_move * Math.sin(-(angle));
                double y = y_move * Math.cos(-(angle)) + x_move * Math.sin(-(angle));

                // Dividing power by a denominator to set maximum output to 1
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rotation_x), 1);

                double frontLeftPower = (y + x + rotation_x) / denominator;
                double backLeftPower = (y - x + rotation_x) / denominator;
                double frontRightPower = (y - x - rotation_x) / denominator;
                double backRightPower = (y + x - rotation_x) / denominator;

                // Writing the power to the motors
                front_Left.setPower(frontLeftPower);
                back_Left.setPower(backLeftPower);
                front_Right.setPower(frontRightPower);
                back_Right.setPower(backRightPower);
            }

            else {
                front_Left.setPower(0);
                back_Left.setPower(0);
                front_Right.setPower(0);
                back_Right.setPower(0);
            }
        }
    }


    //RR
    Pose2d initialPose;
    PinpointDrive drive;
    ElapsedTime tm1;

    AutoArm arm;
    MecDrive mec;

    @Override
    public void runOpMode() {

        initialPose = new Pose2d(-62, -10, Math.toRadians(90));
        drive = new PinpointDrive(hardwareMap, initialPose);
        mec = new MecDrive(hardwareMap);

        arm = new AutoArm(hardwareMap);
        tm1 = new ElapsedTime();

        arm.armToPos(ready[0], ready[1]);
        grab.setWristRotation(0.1);
        grab.lightGrab();

        TrajectoryActionBuilder first = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(firstMove[0], firstMove[1]), firstMove[2]);


        telemetry.addData("Program", "States Right");
        telemetry.update();

        //arm.readyPos();

        waitForStart();

        TrajectoryActionBuilder scorePos = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(toScore[0] + 3, toScore[1]), toScore[2]);

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                scorePos.build(),
                                arm.barInit()
                        ),
                        arm.barScore(),
                        arm.release()
                )
        );

        tm1.reset();
        time = tm1.milliseconds();

        while (time < 400 && opModeIsActive()) {
            time = tm1.milliseconds();
        }

        TrajectoryActionBuilder clearing = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(firsty[0], firsty[1]), firsty[2]);


        Actions.runBlocking(
                new SequentialAction (
                        new ParallelAction(
                                arm.movePos(),
                                clearing.build()
                        )
                )
        );

        mec.run(1.0, 0, 0, Math.PI);

        tm1.reset();
        time = tm1.milliseconds();

        while (time < 400 && opModeIsActive()) {
            time = tm1.milliseconds();
        }

        mec.run(0, 0, 0, Math.PI);

        while (time < 1000 && opModeIsActive()) {
            time = tm1.milliseconds();
        }
    }
}
