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

                // stick inputs
                leftX = gamepad1.left_stick_x;
                leftY = gamepad1.left_stick_y;
                rightX = gamepad1.right_stick_x;
                rightY = gamepad1.right_stick_y;

                
                if(gamepad1.circle & !circlePressed & limit1 < 1){
                    circlePressed = true;
                    limit1 += 0.1;
                }
                else if(!gamepad1.circle & circlePressed){
                    circlePressed = false;
                }

                
                if(gamepad1.square & !squarePressed & limit1 > 0){
                    squarePressed = true;
                    limit1 -= 0.1;
                }
                else if(!gamepad1.square & squarePressed){
                    squarePressed = false;
                }
                telemetry.addData("Balu limit", limit1);


                
                if(gamepad1.triangle & !trianglePressed & limit0 > 1){
                    trianglePressed = true;
                    limit0 += 0.1;
                }
                else if(!gamepad1.triangle & trianglePressed){
                    trianglePressed = false;
                }

                
                if(gamepad1.x & !xPressed & limit0 < 0){
                    xPressed = true;
                    limit0 -= 0.1;
                }
                else if(!gamepad1.x & xPressed){
                    xPressed = false;
                }
                telemetry.addData("Etele limit", limit0);

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
                EleftY = Math.pow(leftY, 3) * Math.signum(leftY)*limit;
                EleftX = Math.pow(leftX, 3) * Math.signum(leftX)*limit;
//                ErightY = Math.pow(rightY, 2) * Math.signum(rightY) * 0.4;
                ErightX = Math.pow(rightX, 3) * Math.signum(rightX)*limit;

                if (ErightX > 0.3) {
                    drive0.setPower(ErightX);
                    drive1.setPower(ErightX);
                    drive2.setPower(ErightX);
                    drive3.setPower(ErightX);
                } else if (EleftX == 0 && EleftY == 0) {
                    drive0.setPower(0);
                    drive1.setPower(0);
                    drive2.setPower(0);
                    drive3.setPower(0);
                } else {
                    drive1.setPower((EleftX + EleftY) * -1);
                    drive2.setPower((EleftX + EleftY));
                    drive0.setPower((EleftX - EleftY));
                    drive3.setPower((EleftX - EleftY) * -1);
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