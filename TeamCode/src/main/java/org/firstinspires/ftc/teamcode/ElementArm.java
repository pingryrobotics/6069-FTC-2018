package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ElementArm {
    private DcMotor armRotation;
    private DcMotor wrist;
    private Servo intake;
    private Servo collectionBox;

    private final int outerLimit = 114;
    double pushSpeed = 0.75;
    double pullSpeed = 0.75;
    public static void main(String args[]){

    }
    public ElementArm(HardwareMap hardwareMap){
        armRotation = hardwareMap.get(DcMotor.class, "armExtension");
        wrist = hardwareMap.get(DcMotor.class, "wrist");
        intake = hardwareMap.get(Servo.class, "ziptieIntake");
        collectionBox = hardwareMap.get(Servo.class, "collectionBox");

        armRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //collectionBox.scaleRange(0,2.0/3);
    }

    public void resetEncoders(){
        armRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //A note for these current values for the set positions as of 10.15.18 they are just filled in and will be adjusted later.
    public void rotateOut(double speed){
        if(armRotation.getCurrentPosition() >= outerLimit){
            armRotation.setPower(0);
        }else {
            armRotation.setPower(speed);
        }
    }
    public void rotateBack(double speed){
        armRotation.setPower(-speed);
    }

    public void outtake(){
        collectionBox.setPosition(.5);
    }
    /*
    public void intake(){
        intake.setDirection(Servo.Direction.REVERSE);
    }
    public void stopExtend(){
        armRotation.setPower(0);
    }

    public void stopSpin(){
        intake.setPosition(0);
    }
*/
}
