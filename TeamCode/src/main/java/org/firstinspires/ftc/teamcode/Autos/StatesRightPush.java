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
@Autonomous(name = "States Right")
public class StatesRightPush extends LinearOpMode {

    //TODO: Shoulder motor initializations
    DcMotorEx shoulder;
    DcMotorEx lSlides;
    DcMotorEx rSlides;

    int firstY = -36;
    int secondY = -48;
    int thirdY = -60;

    int pushIn = -56;
    int farOut = -12;

    //TODO: Tune robot positions here
    // x, y, heading
    double[] firstMove = {-50, 2, Math.toRadians(180)};
    double[] toScore = {-29, 2, Math.toRadians(180)};
    double[] clearPos = {-33, 2, Math.toRadians(180)};
    double[] first = {-33, -24, Math.toRadians(180)};
    double[] second = {farOut, -24, Math.toRadians(180)};
    double[] third = {farOut, firstY, Math.toRadians(180)};
    double[] fourth = {pushIn, firstY, Math.toRadians(180)};
    double[] fifth = {farOut, firstY, Math.toRadians(180)};
    double[] sixth = {farOut, secondY, Math.toRadians(180)};
    double[] seventh = {pushIn, secondY, Math.toRadians(180)};
    double[] eigth = {farOut, secondY, Math.toRadians(180)};
    double[] ninth = {farOut, thirdY, Math.toRadians(180)};
    double[] tenth = {pushIn, thirdY, Math.toRadians(180)};

    double[] wallInit = {-54, -28, Math.toRadians(180)};
    double[] wallNext = {-57, -28, Math.toRadians(180)};

    //TODO: Tune arm positions here
    // pitch, slides
    int[] ready = {2420, 10};
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

    //RR
    Pose2d initialPose;
    PinpointDrive drive;
    ElapsedTime tm1;

    AutoArm arm;

    @Override
    public void runOpMode() {

        initialPose = new Pose2d(-62, -10, Math.toRadians(90));
        drive = new PinpointDrive(hardwareMap, initialPose);

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

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                first.build(),
                                arm.barInit()
                        )
                )
        );

        TrajectoryActionBuilder scorePos = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(toScore[0], toScore[1]), toScore[2]);

        Actions.runBlocking(
                new SequentialAction(
                        scorePos.build(),
                        arm.barScore(),
                        arm.release()
                )
        );

        tm1.reset();
        time = tm1.milliseconds();

        while (time < 400) {
            time = tm1.milliseconds();
        }

        TrajectoryActionBuilder clearing = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(second[0], second[1]), second[2])
                .strafeToLinearHeading(new Vector2d(third[0], third[1]), third[2])
                .strafeToLinearHeading(new Vector2d(fourth[0], fourth[1]), fourth[2])
                .strafeToLinearHeading(new Vector2d(fifth[0], fifth[1]), fifth[2])
                .strafeToLinearHeading(new Vector2d(sixth[0], sixth[1]), sixth[2])
                .strafeToLinearHeading(new Vector2d(seventh[0], seventh[1]), seventh[2])
                .strafeToLinearHeading(new Vector2d(eigth[0], eigth[1]), eigth[2])
                .strafeToLinearHeading(new Vector2d(ninth[0], ninth[1]), ninth[2])
                .strafeToLinearHeading(new Vector2d(tenth[0], tenth[1]), tenth[2])
                .strafeToLinearHeading(new Vector2d(wallInit[0], wallInit[1]), wallInit[2])
                .strafeToLinearHeading(new Vector2d(wallNext[0], wallNext[1]), wallNext[2]);


        Actions.runBlocking(
                new SequentialAction (
                        new ParallelAction(
                                arm.prepWall(),
                                clearing.build()
                        ),

                        arm.grab()
                )
        );

        tm1.reset();
        time = tm1.milliseconds();
        while (time < 400) {
            time = tm1.milliseconds();
        }

        TrajectoryActionBuilder goScore = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(clearPos[0], clearPos[1]), clearPos[2])
                .strafeToLinearHeading(new Vector2d(toScore[0], toScore[1]), toScore[2]);

        Actions.runBlocking(
                new SequentialAction (
                        new ParallelAction(
                                arm.barInit(),
                                goScore.build()
                        ),

                        arm.barScore(),
                        arm.release()
                )
        );

        tm1.reset();
        time = tm1.milliseconds();
        while (time < 400) {
            time = tm1.milliseconds();
        }

        Actions.runBlocking(
                new SequentialAction (
                        arm.prepWall()
                )
        );
    }
}
