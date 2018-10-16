package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ElementArm {
    private DcMotor armExtension;
    private DcMotor wrist;
    private Servo intake;
    private Servo collectionBox;
    double pushSpeed = 0.75;
    double pullSpeed = 0.75;
    public static void main(String args[]){

    }
    public ElementArm(HardwareMap hardwareMap){
        armExtension = hardwareMap.get(DcMotor.class, "armExtension");
        wrist = hardwareMap.get(DcMotor.class, "wrist");
        intake = hardwareMap.get(Servo.class, "ziptieIntake");
        collectionBox = hardwareMap.get(Servo.class, "collectionBox");
    }

    //A note for these current values for the set positions as of 10.15.18 they are just filled in and will be adjusted later.
    public void extend(double speed){
        armExtension.setPower(speed);
    }
    public void retract(double speed){
        armExtension.setPower(-speed);
    }
    public void rotateUp(){
        wrist.setTargetPosition(1);
    }
    public void rotateDown(){
        wrist.setTargetPosition(-1);
    }
    public void outtake(){
        collectionBox.setPosition(-0.5);
    }
    public void intake(){
        intake.setDirection(Servo.Direction.REVERSE);
    }
    public void stopExtend(){
        armExtension.setPower(0);
    }
    public void stopSpin(){
        intake.setPosition(0);
    }
}
