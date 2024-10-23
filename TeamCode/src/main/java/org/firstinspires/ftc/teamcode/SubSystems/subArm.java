package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class subArm {

    subShoulder should;
    subGrab grab;

    public int state = 0;
    public int routine = 0;

    double defShPow = 0.5;
    double defSlPow = 0.5;
    double defOutPow = .5;
    int defTol = 20;

    // Positional vars
    //TODO: Setup these values
    int[] ground = {0, 0};
    int[] intakeLow = {0, 0};
    int[] intakeHigh = {200, 0};
    int[] wallHigh = {0, 0};
    int[] wallLow = {0, 0};
    int[] railHigh = {0, 0};
    int[] railLow = {350, 0};
    int[] railHighInit = {0, 0};
    int[] railLowInit = {300, 0};
    int[] railHighEnd = {0, 0};
    int[] railLowEnd = {400, 0};
    int[] lowBin = {350, 0};
    int[] highBin = {450, 0};

    public subArm(HardwareMap hardwareMap) {
        should = new subShoulder(hardwareMap);
        grab = new subGrab(hardwareMap);
    }

    public void home() {
        should.setShld(ground[0], defShPow);
        should.setSlides(ground[0], defSlPow);

        if (should.reached(should.slides, defTol) && should.reached(should.shoulder, defTol)) {
            state = 0;
            routine = 0;
        }
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
            if (should.reached(should.slides, defTol) && should.reached(should.shoulder, defTol)) {
                state = 0;
                routine = 0;
            }
        }
    }

    public void grabberUpdate(double lt, double rt) {
        if (rt > .05) {
            grab.outtake(rt);
        }

        else if (lt > .05) {
            grab.intake(lt);
        }

        else {
            grab.stop();
        }
    }

    public void bar(boolean high, boolean a){
        if (state == 0) {
            if (high) {
                should.setShld(railHighInit[0], defShPow);
                should.setSlides(railHighInit[1], defSlPow);
            }

            else {
                should.setShld(railLowInit[0], defSlPow);
                should.setSlides(railLowInit[1], defSlPow);
            }
            state++;
        }

        if (state == 1) {
            if (should.reached(should.slides, defTol) && should.reached(should.shoulder, defTol) && a) {
                if (high) {
                    should.setShld(railHigh[0], defShPow);
                    should.setSlides(railHigh[1], defSlPow);
                }

                else {
                    should.setShld(railLow[0], defSlPow);
                    should.setSlides(railLow[1], defSlPow);
                }
                state++;
            }
        }

        if (state == 3) {
            if (a) {
                grab.outtake(defOutPow);
                if (high) {
                    should.setShld(railHighEnd[0], defShPow);
                    should.setSlides(railHighEnd[1], defSlPow);
                }

                else {
                    should.setShld(railLowEnd[0], defSlPow);
                    should.setSlides(railLowEnd[1], defSlPow);
                }
                state++;
            }
        }

        if (state == 4) {
            if (should.reached(should.slides, defTol) && should.reached(should.shoulder, defTol)) {
                grab.stop();
                state = 0;
                routine = 0;
            }
        }
    }
}
