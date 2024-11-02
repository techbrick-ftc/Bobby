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
    //Shoulder, Slides
    int[] ground = {0, 0};
    int[] home = {2500, 0};
    int[] intakeLow = {0, 2600};
    int[] intakeHigh = {950, 2600};
    int[] wall = {1500, 1200};
    int[] barHigh = {3815, 810};
    int[] barLowInit = {2050, 650};
    int[] barLow = {750, 650};
    int[] lowBin = {3715, 1340};
    int[] highBin = {4400, 3000};


    public subArm(HardwareMap hardwareMap) {
        should = new subShoulder(hardwareMap);
        grab = new subGrab(hardwareMap);
    }

    public void home() {
        should.setShld(home[0], defShPow);
        should.setSlides(home[1], defSlPow);

        if (should.reached(should.lSlides, defTol) && should.reached(should.shoulder, defTol)) {
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
            if (should.reached(should.lSlides, defTol) && should.reached(should.shoulder, defTol)) {
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
}
