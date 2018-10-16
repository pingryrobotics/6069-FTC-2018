package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Hang {
    private DcMotor winch;

    public Hang(HardwareMap hardwareMap) {
        winch = hardwareMap.get(DcMotor.class, "winch");
    }

    public void hangUp(){
        winch.setPower(1);
    }
    public void hangDown(){
        winch.setPower(-1);
    }
    public void stopHang(){
        winch.setPower(0);
    }


}
