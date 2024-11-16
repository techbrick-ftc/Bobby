package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Teleops.Main;

public class subArm {

    subShoulder should;
    subGrab grab;

    public int state = 0;

    double defShPow = 0.5;
    double defSlPow = 0.5;
    double defOutPow = .5;
    double outtakeLimit = .3;
    int defTol = 20;
    public boolean intakeUp = true;

    // Positional vars
    //TODO: Setup these values
    //Shoulder, Slides
    int[] ground = {0, 0};
    int[] home = {1800, 0};
    int[] intakeLow = {100, 2050};
    int[] intakeHigh = {540, 2050};
    int[] wall = {1080, 1200};
    int[] barHigh = {3290, 810};
    int[] barLowInit = {1700, 650};
    int[] barLow = {950, 650};
    int[] lowBin = {3250, 1340};
    int[] highBin = {3825, 3000};

    public subArm(HardwareMap hardwareMap) {
        should = new subShoulder(hardwareMap);
        grab = new subGrab(hardwareMap);
    }

    public void home() {
        should.setShld(home[0], defShPow);
        should.setSlides(home[1], defSlPow);
        Main.routine = 0;
        state = 0;
    }

    public void homeSlides() {
        should.setSlides(home[1], defSlPow);

    }

    public void homeShould() {
        should.setShld(home[0], defSlPow);
    }

    public void bin(boolean high) {

        if (state == 0) {
            if (high) {
                should.setShld(highBin[0], defShPow);
                should.setSlides(highBin[1], defSlPow);
            }

            else {
                should.setShld(lowBin[0], defSlPow);
                should.setSlides(lowBin[1], defSlPow);
            }
            state++;
        }

        if (state == 1) {
            if (should.reached(should.lSlides, defTol) && should.reached(should.shoulder, defTol)) {
                state = 0;
                Main.routine = 0;
            }
        }
    }

    public void highBar(){
        if (state == 0) {
            should.setShld(barHigh[0], defShPow);
            should.setSlides(barHigh[1], defSlPow);
            state++;
        }

        if (state == 1) {
            if (should.reached(should.lSlides, defTol) && should.reached(should.shoulder, defTol)) {
                state = 0;
                Main.routine = 0;
            }
        }
    }

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

    public void pitIntake(boolean a, double x, double y){
        if (state == 0){
            should.setShld(intakeHigh[0], defShPow);
            should.setSlides(intakeHigh[1], defSlPow);
            state++;
        }
        else if (state == 1){
            if (should.reached(should.lSlides, defTol) && should.reached(should.shoulder, defTol)) {

                if (a && intakeUp){
                    should.setShld(intakeLow[0], defShPow);
                    should.setSlides(intakeLow[1], defSlPow);
                    intakeUp = !intakeUp;
                }
                else if (a){
                    should.setShld(intakeHigh[0], defShPow);
                    should.setSlides(intakeHigh[1], defSlPow);
                    intakeUp = !intakeUp;
                }

                if (Math.abs(x) >= .1 || Math.abs(y) >= .1){
                    grab.rotateWrist(-y, x);
                }

                if (grab.checkObjectIn()){
                    should.setShld(home[0], defShPow);
                    should.setSlides(home[1], defSlPow);
                    grab.setRotation(Main.defWristRotate);
                    state++;
                }
            }
        }
        else if (state == 2){
            if (should.reached(should.lSlides, defTol) && should.reached(should.shoulder, defTol)) {
                state = 0;
                Main.routine = 0;
            }
        }
    }

    public void wallIntake(){
        if (state == 0) {
            should.setShld(wall[0], defShPow);
            should.setSlides(wall[1], defSlPow);
            state++;
        }

        if (state == 1) {
            if (should.reached(should.lSlides, defTol) && should.reached(should.shoulder, defTol)) {
                state = 0;
                Main.routine = 0;
            }
        }
    }

    public void manualSlides(double stick_y){
        if (Math.abs(stick_y) > .05) {
            should.setSlides((int) (should.lSlides.getTargetPosition() - 50 * stick_y), defSlPow);
        }
    }

    public void manualShould(double stick_y){
        if (Math.abs(stick_y) > .05) {
            should.setShld((int) (should.shoulder.getTargetPosition() - 50 * stick_y), defSlPow);
        }
    }

    public void grabberUpdate(double lt, double rt) {
        if (rt > .05) {
            grab.outtake(rt * outtakeLimit);
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
}
