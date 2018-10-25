package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Big Blue Robotics on 10/4/2017.
 */

public class MecanumDrive {
    public DcMotor leftFront = null;
    public DcMotor leftRear = null;
    public DcMotor rightFront = null;
    public DcMotor rightRear = null;
    private final double ticksPerInch = (28*35)/(6* Math.PI);//28 / Math.PI;//1120 * 20 / (4 * Math.PI);

    public BNO055IMU imu;

    private double lastAngle = 0;
    private double averageVelocity = 0;
    private double averageGoal = 0;
    private ElapsedTime t;

    public MecanumDrive(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightBack");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;


        resetEncoders();

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        t = new ElapsedTime();
    }

    public void resetEncoders(){
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double getGyroAngle(){
        return imu.getAngularOrientation().firstAngle;
    }

    /**
     * Raw Move at power (Use drive instead)
     * @param leftFrontPower
     * @param rightFrontPower
     * @param leftRearPower
     * @param rightRearPower
     */
    @Deprecated
    public void rawMove(double leftFrontPower, double rightFrontPower, double leftRearPower, double rightRearPower){
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftRear.setPower(leftRearPower);
        rightRear.setPower(rightRearPower);
    }

    /**
     * Strafe using encoders
     * @param inches Inches to strafe (positive is right, negative is left)
     * @param power positive power to strafe at
     */
    public void moveStrafe(double inches, double power){
        power = Math.abs(power);

        resetEncoders();

        leftFront.setTargetPosition((int) (inches * ticksPerInch));
        rightFront.setTargetPosition((int) (-inches * ticksPerInch));
        leftRear.setTargetPosition((int) (-inches * ticksPerInch));
        rightRear.setTargetPosition((int) (inches * ticksPerInch));

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(power);
        rightFront.setPower(power);
        leftRear.setPower(power);
        rightRear.setPower(power);
    }

    public void brake(){
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    /**
     * Move robot forward/backwards using encoders
     * @param inches # of inches to move (negative is backwards)
     * @param power Positive power to move the motors
     */
    public void moveEncoderStraight(double inches, double power){
        power = Math.abs(power);

        resetEncoders();

        leftFront.setTargetPosition((int) (inches * ticksPerInch));
        rightFront.setTargetPosition((int) (inches * ticksPerInch));
        leftRear.setTargetPosition((int) (inches * ticksPerInch));
        rightRear.setTargetPosition((int) (inches * ticksPerInch));

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(inches < 0){
            power *= -1;
        }
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftRear.setPower(power);
        rightRear.setPower(power);
    }

    /**
     * Turn an amount of degrees using encoders
     * @param degrees
     * @param power
     */
    @Deprecated
    public void encoderTurn(double degrees, double power){
        boolean turnRight = degrees > 0;
        power = Math.abs(power);

        resetEncoders();

        double inches = degrees/180 * Math.PI * 13.5;
        int leftFrontTarget = (int) (leftFront.getCurrentPosition() - (inches * 140 / Math.PI));
        int leftRearTarget = (int) (leftRear.getCurrentPosition() - (inches * 140 / Math.PI));
        int rightFrontTarget = (int) (rightFront.getCurrentPosition() + (inches * 140 / Math.PI));
        int rightRearTarget = (int) (rightRear.getCurrentPosition() + (inches * 140 / Math.PI));

        leftFront.setTargetPosition(leftFrontTarget);
        leftRear.setTargetPosition(leftRearTarget);
        rightFront.setTargetPosition(rightFrontTarget);
        rightRear.setTargetPosition(rightRearTarget);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(turnRight){
            leftFront.setPower(power);
            leftRear.setPower(power);
            rightFront.setPower(-power);
            rightRear.setPower(-power);
        }else{
            leftFront.setPower(-power);
            leftRear.setPower(-power);
            rightFront.setPower(power);
            rightRear.setPower(power);
        }
    }

    public void gyroTurn(double angle, double power){
        power = Math.abs(power);

        double gyroAngle = getGyroAngle();
        double driveAngle = power*Math.max(-1, Math.min(1, (gyroAngle - angle)/20.0));

        System.out.println("Angle: "+gyroAngle);
        System.out.println("Turn power: "+driveAngle);
        drive(0, 0, driveAngle);
    }

    /**
     * Encoders are done
     * @return whether or not any of the 4 drive encoders is busy
     */
    public boolean encoderDone(){
        return ((leftFront.isBusy()?1:0) + (leftRear.isBusy()?1:0) + (rightFront.isBusy()?1:0) + (rightRear.isBusy()?1:0))<= 1;
    }

    /**
     * Drive the robot using gyro strafe correction
     * @param direction Angle to strafe at (0 is forward, Pi/2 is left...?)
     * @param rotation Value between -1 and 1 representing power to turn at
     * @param magnitude Speed to strafe at
     */
    public void drive(double direction, double magnitude, double rotation){
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Get the current gyro angle
        double gyroAngle = imu.getAngularOrientation().firstAngle;
        //Find the change since our last angle
        double dAngle = (lastAngle - gyroAngle);
        //Find the change in time since out last measurement
        double dTime = t.seconds();
        t.reset();
        //Find the change in angle over time (angular velocity)
        double velocity = dAngle/dTime;

        //Find the AVERAGE velocity (just a smoothed out velocity that has been averaged to minimize static noise and improve accuracy)
        averageVelocity = (averageVelocity * 3 + velocity)/4;

        //Find the AVERAGE rotational goal (smoothed out)
        averageGoal = (averageGoal * 3 + rotation)/4.0;

        //Update the last angle
        lastAngle = gyroAngle;

        //Update the rotational goal to compensate for how off we are from the goal.
        //Dividing by 300 to convert the degrees per second into power for a motor. We found that about 300 degrees per second is a 1 in turning power.
        //The 1.5x is a multiplier to make sure the offset is applied 2enough to have an actual effect.

        //Commented for now to make drivable (find a new value instead of 300.0 and then uncomment to enable)
        rotation += (averageGoal - averageVelocity/120.0)*1.0;


        direction += Math.PI/4.0;  //Strafe direction needs to be offset so that forwards has everything go at the same power

        final double v1 = magnitude * Math.cos(direction) + rotation;
        final double v2 = magnitude * Math.sin(direction) - rotation;
        final double v3 = magnitude * Math.sin(direction) + rotation;
        final double v4 = magnitude * Math.cos(direction) - rotation;

        leftFront.setPower(v1);
        rightFront.setPower(v2);
        leftRear.setPower(v3);
        rightRear.setPower(v4);
    }
}
