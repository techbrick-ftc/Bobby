package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Teleops.Main;

public class subArm {

    public subShoulder should;
    subGrab grab;

    public int state = 0;

    double defShPow = 1;
    double defSlPow = 1;
    double defOutPow = .5;
    double outtakeLimitLeft = .5;
    double outtakeLimitRight = .2;
    int defTol = 20;
    int highTol = 60;
    int highBinTol = 10;
    public boolean intakeUp = true;
    int slidesSlowHeight = 2500;

    // Positions
    // Shoulder, Slides
    int[] ground = {5, 5};
    int[] home = {1800, 50};
    int[] init = {2420, 20};
    int[] intakeLow = {100, 1905};
    int[] intakeHigh = {650, 1905};
    int[] intakeIn = {900, 100};
    int[] wall = {1450, 705};
    int[] barHighInit = {3350, 835};
    int[] barHigh = {2700, 835};
    int[] barLowInit = {1700, 505};
    int[] barLow = {950, 505};
    int[] lowBin = {3250, 1345};
    int[] highBin = {3690, 3100};

    public subArm(HardwareMap hardwareMap) {
        should = new subShoulder(hardwareMap);
        grab = new subGrab(hardwareMap);
    }

    public void home() {
        should.setShld(home[0], defShPow);
        should.setSlides(home[1], defSlPow);
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
            should.setShld(highBin[0], defShPow);
            should.setSlides(highBin[1], defSlPow);
            state++;
        }
        if (state == 1) {
            if (should.reached(should.lSlides, highBinTol) && should.reached(should.shoulder, highBinTol) && x){
                should.setSlides(home[1], defSlPow);
                state++;
            }
        }
        if (state == 2){
            if (should.lSlides.getCurrentPosition() < highBin[1] / 2){
                Main.deactivateSlowMode();
            }
            if (should.reached(should.lSlides, highBinTol) && should.reached(should.shoulder, highBinTol)){
                should.setShld(home[0], defSlPow);
                state = 0;
                Main.routine = 0;
            }
        }
    }

    public void lowBin() {

        if (state == 0) {
            should.setShld(lowBin[0], defSlPow);
            should.setSlides(lowBin[1], defSlPow);
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
            should.setShld(barHighInit[0], defShPow);
            should.setSlides(barHighInit[1], defSlPow);
            state++;
        }

        if (state == 1) {
            if (should.reached(should.lSlides, defTol) && should.reached(should.shoulder, defTol) && a) {
                should.setShld(barHigh[0], defShPow);
                should.setSlides(barHigh [1], defSlPow);
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

    public void pitIntakeFine(boolean a, double x, double y){
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
                    should.setShld(intakeIn[0], defShPow);
                    should.setSlides(intakeIn[1], defSlPow);
                    grab.setRotation(Main.defWristRotate);
                    state++;
                }
            }
        }
        else if (state == 2){
            if (should.reached(should.lSlides, defTol) && should.reached(should.shoulder, defTol)) {
                Main.drivePow = Main.defPow;
                should.setShld(home[0], defShPow);
                should.setSlides(home[1], defSlPow);
                state = 0;
                Main.routine = 0;
            }
        }
    }

    public void pitIntakeCoarse(boolean a, boolean l, boolean r, boolean u, boolean d){
        if (state == 0){
            should.setShld(intakeHigh[0], defShPow);
            should.setSlides(intakeHigh[1], defSlPow);
            state++;
        }
        else if (state == 1){
            if (should.reached(should.lSlides, highTol) && should.reached(should.shoulder, highTol)) {
                Main.activateSlowMode();

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

                else if (l){
                    if (u){
                        grab.setRotation(.3875);
                    }
                    else if (d){
                        grab.setRotation(.1625);
                    }
                    else {
                        grab.setRotation(.275);
                    }
                }
                else if (r){
                    if (u){
                        grab.setRotation(.8875);
                    }
                    else if (d){
                        grab.setRotation(.6625);
                    }
                    else {
                        grab.setRotation(.775);
                    }
                }
                else if (u){
                    grab.setRotation(.5);
                }
                else if (d){
                    grab.setRotation(.05);
                }

                if (grab.checkObjectIn()){
                    should.setShld(intakeIn[0], defShPow);
                    should.setSlides(intakeIn[1], defSlPow);
                    grab.setRotation(Main.defWristRotate);
                    Main.deactivateSlowMode();
                    state++;
                }
            }
        }
        else if (state == 2){
            if (should.reached(should.lSlides, highTol) && should.reached(should.shoulder, highTol)) {
                should.setShld(home[0], defShPow);
                should.setSlides(home[1], defSlPow);
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
                should.setSlides(wall[1], 0);
                should.lSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                state++;
            }
        }
        if (state == 2) {
            if (should.reached(should.shoulder, defTol) && grab.checkObjectIn()) {
                Main.deactivateSlowMode();
                should.lSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                should.rSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                state = 0;
                Main.routine = 3;
            }
        }
    }

    public void manualSlides(double stick_y){
        if (Math.abs(stick_y) > .05) {
            should.setSlides((int) (should.lSlides.getCurrentPosition() - 50 * stick_y), defSlPow);
        }
    }

    public void manualShould(double stick_y){
        if (Math.abs(stick_y) > .05) {
            should.setShld((int) (should.shoulder.getCurrentPosition() - 50 * stick_y), defSlPow);
        }
    }

    public void grabberUpdate(double lt, double rt) {
        if (rt > .05) {
            grab.outtake(rt * outtakeLimitLeft, rt * outtakeLimitRight);
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
