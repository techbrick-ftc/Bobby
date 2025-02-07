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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.SubSystems.subAutoGrab;
import org.firstinspires.ftc.teamcode.SubSystems.subShoulder;

@Config
@Disabled
@Autonomous(name = "3 on Right")
public class Right3SpecTemplate extends LinearOpMode {

    // Shoulder motor initializations
    DcMotorEx shoulder;
    DcMotorEx lSlides;
    DcMotorEx rSlides;

    // Grabber initializations
    public CRServo Lroller;
    public CRServo Rroller;
    public Servo rotator;

    public ColorSensor colorSensor;
    public DistanceSensor distanceSensor;

    public class AutoArm {

        subAutoGrab grab;

        public AutoArm(HardwareMap hardwareMap) {

            // Shoulder motor declarations
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

            // Grabber declarations
            Lroller = hardwareMap.get(CRServo.class, "LG");
            Rroller = hardwareMap.get(CRServo.class, "RG");
            rotator = hardwareMap.get(Servo.class, "RTR");

            grab = new subAutoGrab(hardwareMap);

        }

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

        private void armToPos(int pitch, int slide, double powP, double powS) {

            // Setting position
            shoulder.setTargetPosition(pitch);
            lSlides.setTargetPosition(slide);
            rSlides.setTargetPosition(slide);

            // Setting power
            shoulder.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            lSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            rSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            shoulder.setPower(powP);
            lSlides.setPower(powS);
            rSlides.setPower(powS);

        }

        private boolean reached(int tol) {
            return Math.abs(lSlides.getCurrentPosition() - lSlides.getTargetPosition()) < tol && Math.abs(shoulder.getCurrentPosition() - shoulder.getTargetPosition()) < tol;
        }

        public class ArmToBar implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotator.setPosition(0.5);
                grab.stop();
                armToPos(3390, 700);
                return !reached(10);
            }
        }
        public Action armToBar() {
            return new ArmToBar();
        }

        public class Score implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotator.setPosition(0.5);
                armToPos(3190, 850);
                return !reached(20);
            }
        }
        public Action score() {
            return new Score();
        }

        public class ArmIn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotator.setPosition(0.5);
                armToPos(2800, 10);
                return !reached(20);
            }
        }
        public Action armIn() {
            return new ArmIn();
        }

        public class FirstSlideReady implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotator.setPosition(0.5);
                grab.stop();
                armToPos(10, 600, 1, 0.3);
                return !reached(20);
            }
        }
        public Action firstSlideReady() {
            return new FirstSlideReady();
        }

        public class SlideReady implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotator.setPosition(0.5);
                grab.stop();
                armToPos(10, 600);
                return !reached(20);
            }
        }
        public Action slideReady() {
            return new SlideReady();
        }

        public class SlideOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotator.setPosition(0.5);
                grab.intake(1);
                armToPos(10, 1800);
                return !(reached(20) || grab.checkObjectIn(false));
            }
        }
        public Action slideOut() {
            return new SlideOut();
        }

        public class Depo implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotator.setPosition(0.5);
                armToPos(10, 1500);
                grab.outtake(1);
                return !reached(20);
            }
        }
        public Action depo() {
            return new Depo();
        }

        public class PitchToWall implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotator.setPosition(0.5);
                armToPos(1430, 10);
                grab.intake(1);
                return !reached(20);
            }
        }
        public Action pitchToWall() {
            return new PitchToWall();
        }


        public class ToWall implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotator.setPosition(0.5);
                armToPos(1430, 705);
                grab.intake(1);
                return !reached(20);
            }
        }
        public Action toWall() {
            return new ToWall();
        }

        public class PitchUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotator.setPosition(0.5);
                armToPos(1700, 705);
                grab.stop();
                return !reached(30);
            }
        }
        public Action pitchUp() {
            return new PitchUp();
        }
    }

    //RR
    Pose2d initialPose;
    PinpointDrive drive;

    AutoArm arm;

    @Override
    public void runOpMode() {

        initialPose = new Pose2d(0, 0, Math.toRadians(0));
        drive = new PinpointDrive(hardwareMap, initialPose);

        arm = new AutoArm(hardwareMap);

        arm.armToPos(2420, 10);

        TrajectoryActionBuilder part1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(32, 0));

        TrajectoryActionBuilder waiter1 = drive.actionBuilder(initialPose)
                .waitSeconds(0.25);
        TrajectoryActionBuilder waiter2 = drive.actionBuilder(initialPose)
                .waitSeconds(0.25);

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        arm.armToBar(), // Raises arm to correct
                        part1.build() // Drive into the bar
                )
        );

        TrajectoryActionBuilder part2 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(24, 0), Math.toRadians(0));

        Actions.runBlocking(
                new SequentialAction(
                        arm.score(), // drop arm down to score
                        arm.armIn(), // pull arm back
                        part2.build() // start moving to grabbing position
                )
        );

        TrajectoryActionBuilder part3 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(20, -29), Math.toRadians(-45));

        Actions.runBlocking(
                new ParallelAction(
                        arm.firstSlideReady(), // move out the slides a bit
                        part3.build() // Get to the position to grab the first additional specimen
                )
        );

        TrajectoryActionBuilder part4 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(22, -24), Math.toRadians(-150));

        Actions.runBlocking(
                new SequentialAction(
                        arm.slideOut(), // move the arm out till you grab something
                        arm.slideReady(), // bring the arm back
                        part4.build(), // rotate over
                        arm.depo(), // put it out for the human player
                        arm.slideReady() // pull back
                )
        );

        TrajectoryActionBuilder part5 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(22, -36), Math.toRadians(-45));

        Actions.runBlocking(
                new SequentialAction(
                        part5.build(), // Go back for another
                        arm.slideOut(), // move out slide till you grab something
                        arm.slideReady() // move slides back
                )
        );

        TrajectoryActionBuilder part6 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(22, -36), Math.toRadians(-150));

        Actions.runBlocking(
                new SequentialAction(
                        part6.build(), // Angle again
                        arm.depo(), // deposit for human player
                        arm.slideReady() // draw back in
                )
        );

        TrajectoryActionBuilder part7 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(10, -24), Math.toRadians(-180));


        Actions.runBlocking(
                new SequentialAction(
                        arm.pitchToWall(), // Extend arm out
                        part7.build() // Go to the wall to pickup
                )
        );

        TrajectoryActionBuilder part8 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(26, 4), Math.toRadians(0));

        Actions.runBlocking(
                new SequentialAction(
                        arm.toWall(),
                        waiter1.build(), // wait for a bit to grab properly
                        arm.pitchUp() // pitch up
                )
        );

        Actions.runBlocking(
                new ParallelAction(
                        part8.build(), // move to slightly back from the scoring position
                        arm.armToBar() // move arm up
                )
        );

        TrajectoryActionBuilder part9 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(32, 4), Math.toRadians(0));

        Actions.runBlocking(
                new SequentialAction(
                        part9.build(), // Get to scoring
                        arm.score() // put arm down
                )
        );

        TrajectoryActionBuilder part10 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(26, 2), Math.toRadians(0));

        Actions.runBlocking(
                new ParallelAction(
                        part10.build(), // Pull back and take arm in
                        arm.armIn() // take arm in
                )
        );

        TrajectoryActionBuilder part11 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(10.5, -28), Math.toRadians(180));

        Actions.runBlocking(
                new ParallelAction(
                        arm.pitchToWall(),
                        part11.build() // Go back for another
                )
        );

        TrajectoryActionBuilder part12 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(26, 8), Math.toRadians(0));


        Actions.runBlocking(
                new SequentialAction(
                        arm.toWall(),
                        waiter1.build(), // wait for a bit to grab properly
                        arm.pitchUp()
                )
        );

        Actions.runBlocking(
                new ParallelAction(
                        part12.build(), // move to slightly back from the scoring position
                        arm.armToBar() // move arm up
                )
        );

        TrajectoryActionBuilder part13 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(32, 8), Math.toRadians(0));

        Actions.runBlocking(
                new ParallelAction(
                        part13.build(), // move to slightly back from the scoring position
                        arm.score() // move arm up
                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        arm.armIn()
                )
        );
    }
}
