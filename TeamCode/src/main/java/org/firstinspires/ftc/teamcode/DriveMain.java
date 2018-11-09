
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;


@TeleOp(name="Monica Mecanum", group="Iterative Opmode")
public class DriveMain extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftRear = hardwareMap.get(DcMotor.class, "leftBack");
        rightRear = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double theta = Math.atan2(-gamepad1.left_stick_y, -gamepad1.left_stick_x);
        double magnitude = Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2));
        double turn = -Range.clip(gamepad1.right_stick_x, -1, 1);
        double rf = Math.sin(theta + (Math.PI/4)) * magnitude;
        double lf = Math.sin(theta - (Math.PI/4)) * magnitude;
        double rb = Math.sin(theta - (Math.PI/4)) * magnitude;
        double lb = Math.sin(theta + (Math.PI/4)) * magnitude;

        //ElementArm elementArm = new ElementArm(hardwareMap);
       Hang hang = new Hang(hardwareMap);
       ElementArm elementArm = new ElementArm(hardwareMap);

       if(gamepad1.dpad_up){
           leftRear.setPower(0.3);
           rightRear.setPower(0.3);
           leftFront.setPower(0.3);
           rightFront.setPower(0.3);
       }else if(gamepad1.dpad_down){
           leftRear.setPower(-0.3);
           rightRear.setPower(-0.3);
           leftFront.setPower(-0.3);
           rightFront.setPower(-0.3);
       } else{
           leftRear.setPower(lb + turn);
           rightRear.setPower(rb - turn);
           leftFront.setPower(lf + turn);
           rightFront.setPower(rf - turn);
       }

        if(gamepad1.right_bumper){
            //winch down for hang
            hang.hangDown();
        }
        else if(gamepad1.left_bumper){
            //hang up
            hang.hangUp();
        }
        else{
            hang.stopHang();
        }
        if(gamepad1.a)
            hang.latch();
        else if(gamepad1.b)
            hang.unlatch();
        double motorPower = gamepad2.left_stick_y;
        if(motorPower != 0){
            if(motorPower >  0)
                elementArm.rotateOut(motorPower);
            else
                elementArm.rotateBack(motorPower);
        }
        if(gamepad2.right_bumper){
            elementArm.outtake();
        }
       /* if(gamepad1.x)
            elementArm.temporaryUp();
        if(gamepad1.y)
            elementArm.temporaryDown();
*/



    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        leftRear.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0); 
        rightFront.setPower(0);
    }

}
 
