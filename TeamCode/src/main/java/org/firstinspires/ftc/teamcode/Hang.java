package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Hang {
    private DcMotor winch;
    private Servo latchServo;

    public Hang(HardwareMap hardwareMap) {
        winch = hardwareMap.get(DcMotor.class, "winch");
        winch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        latchServo = hardwareMap.get(Servo.class, "latch servo");
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
    public void unlatch(){
        latchServo.setPosition(0.25);
    }
    public void latch(){
        latchServo.setPosition(0.75);
    }


}
