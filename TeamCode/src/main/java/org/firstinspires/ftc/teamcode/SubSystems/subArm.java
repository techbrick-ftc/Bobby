package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Teleops.Main;

import java.util.Date;

public class subArm {

    public subShoulder should;
    subGrab grab;
    subLED LED;

    public int state = 0;

    double defShPow = 1;
    double defSlPow = 1;
    int defTol = 20;
    int highTol = 60;
    int highBinTol = 10;
    public boolean intakeUp = true;
    int slidesSlowHeight = 2500;
    int slideReturnHeight = 1300;

    Date time = new Date();
    long initTime;
    int delayMS = 150;
    int realignTime = 500;
    int realignPosInit = 100;
    int realignPos = -100;

    double wristDownAngle = .34;

    // Old Positions
    // Shoulder, Slides
    /*
    int[] ground = {10, 5};
    int[] home = {1400, 50};
    int[] init = {2420, 20};
    int[] intake = {20, 1905};
    int[] intakeIn = {650, 100};
    int[] wall = {1040, 705};
    int[] barHighInit = {2400, 835};
    int[] barHigh = {1940, 835};
    int[] barLowInit = {1220, 505};
    int[] barLow = {680, 505};
    int[] lowBin = {2330, 1345};
    int[] highBin = {2650, 3100};
    */

    // Positions
    // Should, Slides, Wrist
    int[] ground = {10, 5, 50};
    int[] home = {1400, 50, 50};
    int[] extendIntake = {10, 1150, 56};
    int[] intake = {10, 1150, 34};
    int[] retractIntake = {10, 10, 65};
    int[] wallIntake = {1000, 10, 9};
    int[] barInit = {3170, 160, 44};
    int[] barRaise = {3170, 770, 44};
    int[] highBin = {2900, 2950, 30};
    int[] lowBin = {2100, 1550, 33};

    public subArm(HardwareMap hardwareMap) {
        should = new subShoulder(hardwareMap);
        grab = new subGrab(hardwareMap);
    }

    public void home() {
        should.setShld(home[0], defShPow);
        should.setSlides(home[1], defSlPow);
        grab.setWristRotation(convertAngle(home[2]));
        Main.deactivateSlowMode();
        Main.routine = 0;
        state = 0;
    }

    public void homeSlides() {
        should.setSlides(home[1], defSlPow);

    }

    public void homeShould() {
        should.setShld(home[0], defSlPow);
    }

    public void highBin(boolean x) {

        if (should.lSlides.getCurrentPosition() > slidesSlowHeight){
            Main.activateSlowMode();
        }

        if (state == 0) {
            Main.setBinsAngle();
            Main.activateHeadingLock();
            should.setShld(highBin[0], defShPow);
            should.setSlides(highBin[1], defSlPow);
            grab.setWristRotation(convertAngle(highBin[2]));
            state++;
        }
        if (state == 1) {
            if (should.reached(should.lSlides, defTol) && should.reached(should.shoulder, defTol) && x){
                should.setSlides(home[1], defSlPow);
                grab.setWristRotation(convertAngle(home[2]));
                state++;
            }
        }
        if (state == 2){
            if (should.lSlides.getCurrentPosition() < slideReturnHeight){
                home();
            }
        }
    }

    public void lowBin() {

        if (state == 0) {
            Main.setBinsAngle();
            Main.activateHeadingLock();
            should.setShld(lowBin[0], defSlPow);
            should.setSlides(lowBin[1], defSlPow);
            grab.setWristRotation(convertAngle(lowBin[2]));
            state++;
        }
        if (state == 1) {
            if (should.reached(should.lSlides, defTol) && should.reached(should.shoulder, defTol)){
                state = 0;
                Main.routine = 0;
            }
        }
    }

    public void highBar(boolean a){
        if (state == 0) {
            Main.setWallAngle();
            Main.activateHeadingLock();
            should.setShld(barInit[0], defShPow);
            should.setSlides(barInit[1], defSlPow);
            grab.setWristRotation(convertAngle(barInit[2]));
            state++;
        }

        if (state == 1) {
            if (should.reached(should.lSlides, defTol) && should.reached(should.shoulder, defTol) && a) {
                grab.grab();
                should.setSlides(barRaise[1], defSlPow);
                state++;
            }
        }
        else if (state == 2){
            if (should.reached(should.lSlides, defTol) && should.reached(should.shoulder, defTol)){
                grab.release();
                should.setSlidesOverride(realignPos, defSlPow);
                should.setShld(home[0], defShPow);
                time = new Date();
                initTime = time.getTime();
                state++;
            }
        }
        else if (state == 3){
            time = new Date();
            if (time.getTime() - initTime >= realignTime){
                should.resetSlides();
                home();
            }
        }
    }

    /*
    public void lowBar(boolean a){
        if (state == 0) {
            should.setShld(barLowInit[0], defShPow);
            should.setSlides(barLowInit[1], defSlPow);
            state++;
        }
        else if (state == 1){
            if (should.reached(should.lSlides, defTol) && should.reached(should.shoulder, defTol) && a) {
                should.setShld(barLow[0], defShPow);
                should.setSlides(barLow[1], defSlPow);
                state++;
            }
        }
        else if (state == 2){
            if (should.reached(should.lSlides, defTol) && should.reached(should.shoulder, defTol)){
                state = 0;
                Main.routine = 0;
            }
        }
    }
    */

    public void pitIntake(boolean a, boolean lastA){
        if (state == 0){
            should.setShld(extendIntake[0], defShPow);
            should.setSlides(extendIntake[1], defSlPow);
            grab.setWristRotation(convertAngle(extendIntake[2]));
            intakeUp = true;
            state++;
        }
        else if (state == 1){
            if (should.reached(should.lSlides, highTol) && should.reached(should.shoulder, highTol)) {
                Main.activateSlowMode();

                if (a && !lastA && intakeUp){
                    grab.setWristRotation(convertAngle(intake[2]));
                    intakeUp = !intakeUp;
                }
                else if (a && !lastA){
                    grab.setWristRotation(Main.defWristAngle);
                    intakeUp = !intakeUp;
                }

                if (grab.checkObjectIn()){
                    should.setShld(retractIntake[0], defShPow);
                    should.setSlides(retractIntake[1], defSlPow);
                    grab.setWristRotation(convertAngle(retractIntake[2]));
                    Main.deactivateSlowMode();
                    state++;
                }
            }
        }
        else if (state == 2){
            if (should.reached(should.lSlides, highTol) && should.reached(should.shoulder, highTol)) {
                should.setShld(home[0], defShPow);
                should.setSlides(home[1], defSlPow);
                grab.setWristRotation(convertAngle(home[2]));
                state = 0;
                Main.routine = 0;
            }
        }
    }

    public void wallIntake(boolean a){
        if (state == 0) {
            Main.activateSlowMode();
            Main.setWallAngle();
            Main.activateHeadingLock();
            should.setShld(wallIntake[0], defShPow);
            should.setSlides(wallIntake[1], defSlPow);
            grab.setWristRotation(convertAngle(wallIntake[2]));
            grab.release();
            Main.activateSlowMode();
            state++;
        }
        if (state == 1) {
            if (should.reached(should.shoulder, defTol)) {
                if (a) {
                    grab.grab();
                    Main.deactivateSlowMode();
                    time = new Date();
                    initTime = time.getTime();
                    state++;
                }
            }
        }
        if (state == 2){
            time = new Date();
            if (time.getTime() - initTime >= delayMS){
                should.setShld(barInit[0], defShPow);
                state++;
            }
        }
        if (state == 3){
            if (should.reached(should.shoulder, defTol)) {
                grab.lightGrab();
                state = 0;
                Main.routine = 3;
            }

        }
    }

    public void manualSlides(double stick_y){
        if (Math.abs(stick_y) > .05) {
            should.setSlidesOverride((int) (should.lSlides.getCurrentPosition() - 50 * stick_y), defSlPow);
        }
    }

    public void manualShould(double stick_y){
        if (Math.abs(stick_y) > .05) {
            should.setShldOverride((int) (should.shoulder.getCurrentPosition() - 50 * stick_y), defSlPow);
        }
    }

    public void grabberUpdate(double lt, double rt) {
        if (rt > .05) {
            grab.outtake(rt);
        }
        else {
            if (lt > .05 && !grab.checkObjectIn()) {
                grab.intake(lt);
            }
            else {
                grab.stop();
            }
        }
    }

    public boolean updateShouldHome(){
        if (should.reached(should.shoulder, defTol)) {
            state = 0;
            Main.routine = 0;
            return true;
        }
        return false;
    }

    public boolean updateSlideHome(){
        if (should.reached(should.lSlides, defTol)) {
            state = 0;
            Main.routine = 0;
            return true;
        }
        return false;
    }

    public double convertAngle(int ang){
        return ((double)ang) / 100.0;
    }

    public void resetShoulder() {
        should.resetShoulder();
    }

    public void resetSlides(){
        should.resetSlides();
    }

    public void zero(){
        should.setShld(ground[0], defShPow);
        should.setSlides(ground[1], defSlPow);
        Main.routine = 0;
        state = 0;
    }

    public void stopAll(){
        should.setShld(should.shoulder.getCurrentPosition(), defShPow);
        should.setSlides(should.lSlides.getCurrentPosition(), defSlPow);
        state = 0;
        Main.routine = 0;
    }
}
