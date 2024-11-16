package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class subShoulder {

    // Arm motors
    public DcMotorEx shoulder;
    public DcMotorEx lSlides;
    public DcMotorEx rSlides;

    public int slideMin = 0;
    public int shouldMin = 0;

    public int slideMax = 3000;
    public int shouldMax = 5200;

    // Arms Vars
    ElapsedTime tm;

    public subShoulder(HardwareMap hardwareMap) {

        //TODO: Setup configurations later
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

        tm = new ElapsedTime();
        tm.startTime();
    }

    public void setShld(int pos, double pow) {
        if (pos >= shouldMin && pos <= shouldMax) {
            shoulder.setTargetPosition(pos);
        }
        else if (pos < shouldMin){
            shoulder.setTargetPosition(slideMin);
        }
        else{
            shoulder.setTargetPosition(slideMax);
        }
        shoulder.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        shoulder.setPower(pow);
    }

    public void setShldOverride(int pos, double pow) {
        shoulder.setTargetPosition(pos);
        shoulder.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        shoulder.setPower(pow);
    }

    public void setSlides(int pos, double pow) {
        if (pos >= slideMin && pos <= slideMax) {
            lSlides.setTargetPosition(pos);
            rSlides.setTargetPosition(pos);
        }
        else if (pos < slideMin){
            lSlides.setTargetPosition(slideMin);
            rSlides.setTargetPosition(slideMin);
        }
        else {
            lSlides.setTargetPosition(slideMax);
            rSlides.setTargetPosition(slideMax);
        }

        lSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lSlides.setPower(pow);
        rSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rSlides.setPower(pow);
    }

    public void setSlidesOverride(int pos, double pow) {
        lSlides.setTargetPosition(pos);
        rSlides.setTargetPosition(pos);

        lSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lSlides.setPower(pow);
        rSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rSlides.setPower(pow);
    }

    public boolean reached(DcMotorEx motor, int tol) {
        return Math.abs(motor.getTargetPosition() - motor.getCurrentPosition()) < tol;
    }

}
