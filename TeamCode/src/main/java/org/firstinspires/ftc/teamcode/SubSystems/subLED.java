package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class subLED {

    RevBlinkinLedDriver blinkin;
    int state;

    public subLED(HardwareMap hardwareMap){
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin1");
        setIdle();
    }

    public void setBlue(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        state = 1;
    }

    public void setRed(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        state = 1;
    }

    public void setYellow(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        state = 1;
    }

    public void setFire(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(22));
        state = 2;
    }

    public void setIdle(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(76));
        state = 0;
    }

    public int getState(){
        return state;
    }
}
