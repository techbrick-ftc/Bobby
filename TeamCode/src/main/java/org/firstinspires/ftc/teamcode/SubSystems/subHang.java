package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Teleops.Main;

import java.util.Date;


public class subHang {

    subShoulder should;
    subLift lift;
    subLED LED;

    int state = 0;

    double defShPow = 1;
    double defSlPow = 1;
    double defLiftPow = 1;

    int defTol = 20;
    int rotateTol = 50;
    int shouldTol = 100;

    Date time = new Date();
    long endTime = 0;
    int delayMS = 6000;
    boolean timerStarted = false;

    // Shoulder, slides, lift
    int[] init = {2100, 1850, 2150};
    int[] lowHang1 = {2260, 2850, 1060};
    int[] lowHang2 = {2580, 2850, 1060};
    int[] hookSlides = {2500, 2450, 1060};
    int[] pullUp = {2190, 1100, 550};
    int[] rotate = {1940, 1100, 550};
    int[] pullHigher = {1400, 290, 270};
    int[] finalize = {1030, 70, 10};


    public subHang(HardwareMap hardwareMap){
        should = new subShoulder(hardwareMap);
        lift = new subLift(hardwareMap);
        LED = new subLED(hardwareMap);
    }

    public void hang(boolean x){
        if (state == 0){
            should.setShld(init[0], defShPow);
            should.setSlides(init[1], defSlPow);
            lift.setLift(init[2], defLiftPow);
            state++;
        }
        else if (state == 1 && should.reached(should.lSlides, defTol) && should.reached(should.shoulder, defTol) && lift.reached(lift.lift, defTol) && x) {
            should.setShld(lowHang1[0], defShPow);
            should.setSlides(lowHang1[1], defSlPow);
            lift.setLift(lowHang1[2], defLiftPow);
            state++;
        }
        else if (state == 2 && should.reached(should.lSlides, defTol) && should.reached(should.shoulder, defTol) && lift.reached(lift.lift, defTol) && x) {
            should.setShld(lowHang2[0], defShPow);
            should.setSlides(lowHang2[1], defSlPow);
            lift.setLift(lowHang2[2], defLiftPow);
            state++;
        }
        else if (state == 3 && should.reached(should.lSlides, defTol) && should.reached(should.shoulder, defTol) && lift.reached(lift.lift, defTol) && x) {
            should.setShld(hookSlides[0], defShPow);
            should.setSlides(hookSlides[1], defSlPow);
            lift.setLift(hookSlides[2], defLiftPow);
            state++;
        }
        else if (state == 4 && should.reached(should.lSlides, defTol) && should.reached(should.shoulder, defTol) && lift.reached(lift.lift, defTol) && x) {
            should.setShld(pullUp[0], defShPow);
            should.setSlides(pullUp[1], defSlPow);
            lift.setLift(pullUp[2], defLiftPow);
            state++;
        }
        else if (state == 5 && should.reached(should.lSlides, rotateTol) && should.reached(should.shoulder, rotateTol) && lift.reached(lift.lift, rotateTol) && x) {
            should.setShld(rotate[0], defShPow);
            should.setSlides(rotate[1], defSlPow);
            lift.setLift(rotate[2], defLiftPow);
            state++;
        }
        else if (state == 6 && should.reached(should.lSlides, defTol) && should.reached(should.shoulder, defTol) && lift.reached(lift.lift, defTol) && x) {
            should.setShld(pullHigher[0], defShPow);
            should.setSlides(pullHigher[1], defSlPow);
            lift.setLift(pullHigher[2], 0);
            state++;
        }
        else if (state == 7 && should.reached(should.lSlides, defTol) && should.reached(should.shoulder, defTol) && lift.reached(lift.lift, defTol) && x) {
            should.setShld(finalize[0], defShPow);
            should.setSlides(finalize[1], defSlPow);
            lift.setLift(finalize[2], 0);
            state = 0;
            Main.routine = 0;
        }
    }

    public void altHang(boolean x){
        if (state == 0){
            should.setShld(init[0], defShPow);
            should.setSlides(init[1], defSlPow);
            lift.setLift(init[2], defLiftPow);
            state++;
        }
        else if (state == 1 && should.reached(should.lSlides, defTol) && should.reached(should.shoulder, defTol) && lift.reached(lift.lift, defTol) && x) {
            should.setShld(lowHang1[0], defShPow);
            should.setSlides(lowHang1[1], defSlPow);
            lift.setLift(lowHang1[2], defLiftPow);
            state++;
        }
        else if (state == 2 && should.reached(should.lSlides, defTol) && should.reached(should.shoulder, defTol) && lift.reached(lift.lift, defTol) && x) {
            should.setShld(lowHang2[0], defShPow);
            should.setSlides(lowHang2[1], defSlPow);
            lift.setLift(lowHang2[2], defLiftPow);
            state++;
        }
        else if (state == 3 && should.reached(should.lSlides, defTol) && should.reached(should.shoulder, rotateTol) && lift.reached(lift.lift, defTol) && x) {
            should.setShld(hookSlides[0], defShPow);
            should.setSlides(hookSlides[1], defSlPow);
            lift.setLift(hookSlides[2], defLiftPow);
            state++;
        }
        else if (state == 4 && should.reached(should.lSlides, defTol) && should.reached(should.shoulder, defTol) && lift.reached(lift.lift, rotateTol) && x) {
            should.setShld(finalize[0], defShPow);
            should.setSlides(finalize[1], defSlPow);
            lift.setLift(finalize[2], defLiftPow);
            LED.setFire();
            state = 0;
            Main.routine = 0;
        }
    }

    public void releaseSlides(boolean a){
        time = new Date();
        if (a){
            endTime = time.getTime();
            timerStarted = true;
        }
        if (time.getTime() - endTime > delayMS && timerStarted){
            should.lSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            should.rSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            should.lSlides.setPower(0);
            should.rSlides.setPower(0);
            state = 0;
            Main.routine = 0;
        }
    }

    public void stopLift(){
        lift.setLift(lift.lift.getCurrentPosition(), defLiftPow);
        state = 0;
        Main.routine = 0;
    }

    public boolean getTimerStarted(){
        return timerStarted;
    }

    public long getEndTime(){
        return endTime;
    }
}
