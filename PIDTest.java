package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp

public class PIDTest extends LinearOpMode {

    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;


    @Override
    public void runOpMode() {
        leftMotor = hardwareMap.get(DcMotorEx.class, "left"); 
        rightMotor = hardwareMap.get(DcMotorEx.class, "right"); 
    
        leftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        
        leftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        double targetPower = 0.2;
        double kP = 0.3;
        
        waitForStart();
        while (opModeIsActive()) {
            // float leftX = gamepad1.left_stick_x;
            float leftY = gamepad1.left_stick_y;
            
            
            int leftPosition = leftMotor.getCurrentPosition();
            int rightPosition = rightMotor.getCurrentPosition();

            // Calculate the speed difference
            int error = leftPosition - rightPosition;

            // Apply a correction to the motor powers
            double correction = kP * error;

            double leftPower = targetPower - correction;
            double rightPower = targetPower + correction;

            // Set motor powers
            if(gamepad1.circle){
                leftMotor.setPower(leftPower);
                rightMotor.setPower(rightPower);
            }
            else if(gamepad1.square){
                leftMotor.setPower(-leftPower);
                rightMotor.setPower(-rightPower);
            }
            else{
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            }
            
        }
    }
}