package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class subShoulder {

    // Arm motors
    public DcMotorEx shoulder;
    public DcMotorEx slides;

    // Arms Vars
    ElapsedTime tm;

    public subShoulder(HardwareMap hardwareMap) {

        //TODO: Setup configurations later
        shoulder = hardwareMap.get(DcMotorEx.class, "AR");

        shoulder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shoulder.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shoulder.setDirection(DcMotorEx.Direction.REVERSE);

        tm = new ElapsedTime();
        tm.startTime();
    }

    public void setShld(int pos, double pow) {
        shoulder.setTargetPosition(pos);
        shoulder.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        shoulder.setPower(pow);
    }

    public void setSlides(int pos, double pow) {
        slides.setTargetPosition(pos);
        slides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slides.setPower(pow);
    }

    public boolean reached(DcMotorEx motor, int tol) {
        return Math.abs(motor.getTargetPosition() - motor.getCurrentPosition()) < tol;
    }
}
