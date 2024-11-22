package org.firstinspires.ftc.teamcode.Autos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.PinpointDrive;

@Config
@Autonomous(name = "Blue Right Test 2")
@Disabled
public class BlueRightTest2 extends LinearOpMode {

    public class AutoArm {

        // Shoulder motor initializations
        DcMotorEx shoulder;
        DcMotorEx lSlides;
        DcMotorEx rSlides;

        // Grabber initializations
        public CRServo Lroller;
        public CRServo Rroller;
        public Servo rotator;

        double distanceTarget;
        int direction;

        ColorSensor colorSensor;
        DistanceSensor distanceSensor;

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

            distanceSensor = hardwareMap.get(DistanceSensor.class, "CS");
            colorSensor = hardwareMap.get(ColorSensor.class, "CS");

            distanceTarget = 5.75;
            direction = 0;

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

        private void intake(double pow) {
            Lroller.setPower(pow);
            Rroller.setPower(-pow);
            direction = 1;
        }

        private void outtake(double pow) {
            Lroller.setPower(-pow);
            Rroller.setPower(pow);
            direction = -1;
        }

        private void stop() {
            Lroller.setPower(0);
            Rroller.setPower(0);
            direction = 0;
        }


        public class ArmToBar implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotator.setPosition(0.5);
                armToPos(3290, 810);
                return false;
            }
        }
        public Action armToBar() {
            return new ArmToBar();
        }

        public class ArmToBarDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armToPos(2800, 0);
                return false;
            }
        }
        public Action armToBarDown() {
            return new ArmToBarDown();
        }

        public class ArmToWall implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armToPos(1220, 850);
                intake(0.5);
                return false;
            }
        }
        public Action armToWall() {
            return new ArmToWall();
        }

        public class ArmBack implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                stop();
                armToPos(3290, 0);
                return false;
            }
        }
        public Action armBack() {
            return new ArmBack();
        }
    }

    //RR
    Pose2d initialPose;
    PinpointDrive drive;

    AutoArm arm;

    @Override
    public void runOpMode() {

        initialPose = new Pose2d(-66, -18, 0);
        drive = new PinpointDrive(hardwareMap, initialPose);

        arm = new AutoArm(hardwareMap);

        TrajectoryActionBuilder toBar = drive.actionBuilder(initialPose)
                .waitSeconds(1)
                .splineTo(new Vector2d(-30, -12), 0);

        TrajectoryActionBuilder backToPush;

        telemetry.addData("Program", "Blue Right");
        telemetry.update();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        arm.armToBar(),
                        toBar.build()
                )
        );

        backToPush = drive.actionBuilder(drive.pose)
                .lineToX(-46)
                .strafeTo(new Vector2d(-42, -38))
                .setTangent(0)
                .lineToX(-12)
                .strafeTo(new Vector2d(-12, -48))
                .strafeTo(new Vector2d(-56, -48))
                .strafeToSplineHeading(new Vector2d(-44, -48), Math.toRadians(170), new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                        return 4;
                    }
                })
                .waitSeconds(1)
                .splineTo(new Vector2d(-48, -12), Math.toRadians(180))
                .waitSeconds(1);

        //.lineToXConstantHeading(50);

        Actions.runBlocking(
                new ParallelAction(
                        arm.armToBarDown(),
                        backToPush.build()
                )
        );

        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("h", drive.pose.heading.toDouble());
    }
}
