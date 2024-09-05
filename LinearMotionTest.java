package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "LinearMotionTest")
public class LinearMotionTest extends LinearOpMode {

    static final double sqrt2over2 = Math.sqrt(2) / 2;
    private DcMotor drive0;
    private DcMotor drive1;
    private DcMotor drive2;
    private DcMotor drive3;
    private DcMotor linearMotionMotor0;
    private DcMotor linearMotionMotor1;
    private DcMotor pezLinear;


    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        double leftY;
        double leftX;
        double rightY;
        double rightX;
        double EleftY;
        double EleftX;
        double ErightY;
        double ErightX;
        double limit0 = 0.5;
        double limit1 = 0.5;
        double limit = 1;
        
        boolean circlePressed = false;
        boolean squarePressed = false;
        boolean trianglePressed = false;
        boolean xPressed = false;
        

        drive0 = hardwareMap.get(DcMotor.class, "drive0");
        drive1 = hardwareMap.get(DcMotor.class, "drive1");
        drive2 = hardwareMap.get(DcMotor.class, "drive2");
        drive3 = hardwareMap.get(DcMotor.class, "drive3");
        linearMotionMotor0 = hardwareMap.get(DcMotor.class, "linear0");;
        linearMotionMotor1 = hardwareMap.get(DcMotor.class, "linear1");
        pezLinear = hardwareMap.get(DcMotor.class, "pez");
        // Put initialization blocks here.

        
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // ====== DRIVE CONTROL =====

                leftX = gamepad1.left_stick_x;
                leftY = gamepad1.left_stick_y;
                rightX = gamepad1.right_stick_x;
                rightY = gamepad1.right_stick_y;

                // gas pedal
                if (gamepad1.right_trigger > 0.1) {
                    leftY = -gamepad1.right_trigger;
                    leftX = 0;
                }

                if (gamepad1.left_trigger > 0.1) {
                    leftY = gamepad1.left_trigger;
                    leftX = 0;
                }

                // square
                EleftY = Math.pow(leftY, 2) * Math.signum(leftY) * limit;
                EleftX = Math.pow(leftX, 2) * Math.signum(leftX) * limit;
                ErightY = Math.pow(rightY, 2) * Math.signum(rightY) * 0.4;
                ErightX = Math.pow(rightX, 2) * Math.signum(rightX) * 0.4;

                if (ErightX != 0) {
                    drive0.setPower(ErightX);
                    drive1.setPower(ErightX);
                    drive2.setPower(ErightX);
                    drive3.setPower(ErightX);
                } else if (EleftXd == 0 && EleftYd == 0) {
                    drive0.setPower(0);
                    drive1.setPower(0);
                    drive2.setPower(0);
                    drive3.setPower(0);
                } else {
                    drive2.setPower(sqrt2over2 * (EleftXd + EleftYd));
                    drive1.setPower(sqrt2over2 * (EleftXd + EleftYd) * -1);
                    drive0.setPower(sqrt2over2 * (EleftXd - EleftYd));
                    drive3.setPower(sqrt2over2 * (EleftXd - EleftYd) * -1);
                }
                // ====== LINEAR MOTION CONTROL =====

                // control of Balu's linear motion motor
                if(gamepad1.dpad_up){
                    linearMotionMotor0.setPower(-1*limit0);
                }
                else if(gamepad1.dpad_down){
                    linearMotionMotor0.setPower(1*limit0);
                }
                else{
                    linearMotionMotor0.setPower(0);
                }

                // control of Eteles's linear motion motor
                if(gamepad1.dpad_right){
                    linearMotionMotor1.setPower(1*limit1);
                }
                else if(gamepad1.dpad_left){
                    linearMotionMotor1.setPower(-1*limit1);
                }
                else{
                    linearMotionMotor1.setPower(0);
                }
                
                // control of PEZ linear motion motor
                if(gamepad1.left_bumper){
                    pezLinear.setPower(1);
                }
                else if(gamepad1.right_bumper){
                    pezLinear.setPower(-1);
                }
                else{
                    pezLinear.setPower(0);
                }
                telemetry.update();
            }
        }
    }

    public void turnHDHexNTimes(int N, DcMotor motor) {
        int countsPerRevolution = 1440;  // Example value, replace with actual CPR for your motor
        int targetPosition = N * countsPerRevolution;

        motor.setTargetPosition(targetPosition);

        // Set the motor to run to the target position.
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the motor power to start the movement.
        motor.setPower(0);  // Adjust power as needed

        // Wait until the motor reaches the target position
        while (opModeIsActive() && motor.isBusy()) {
            telemetry.addData("Current Position", motor.getCurrentPosition());
            telemetry.update();
        }
    }
}