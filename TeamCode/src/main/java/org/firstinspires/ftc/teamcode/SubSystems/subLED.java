package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class subLED {

    RevBlinkinLedDriver blinkin1;
    int state;

    public subLED(HardwareMap hardwareMap){
        blinkin1 = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin1");
        blinkin1.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(76));
        state = 0;
    }

    public void setBlue(){
        blinkin1.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        state = 1;
    }

    public void setRed(){
        blinkin1.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        state = 1;
    }

    public void setYellow(){
        blinkin1.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        state = 1;
    }

    public void setFire(){
        blinkin1.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(22));
        state = 2;
    }

    public void setIdle(){
        blinkin1.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(76));
        state = 0;
    }

    public int getState(){
        return state;
    }
}
