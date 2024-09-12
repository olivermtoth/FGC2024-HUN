package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp

public class EncoderTestOliver extends LinearOpMode{

    private DcMotor motor;
    private int[] height = {50, 1600, 3800, 6300};
    private int heightIndex = 0; 
    private DcMotor pez;


    @Override
    
    public void runOpMode() {
        motor = hardwareMap.get(DcMotor.class, "motor"); 
        pez = hardwareMap.get(DcMotor.class, "pez"); 
        
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        boolean circlePressed = false;
        
        waitForStart();
        while (opModeIsActive()) {
            
            if(gamepad1.circle & !circlePressed){
                heightIndex = (heightIndex + 1)%4;
                circlePressed = true;
            }
            else if(!gamepad1.circle & circlePressed){
                circlePressed = false;
            }
            
            if(gamepad1.triangle){
                motor.setTargetPosition(-height[heightIndex]);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(0.5);
            }
            
            if(gamepad1.square){
                heightIndex = 0;
                motor.setTargetPosition(-height[heightIndex]);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(0.5);
            }
            
            pez.setPower(gamepad1.left_stick_y);
            
            
            
            telemetry.addData("Position", heightIndex);
            telemetry.addData("encoder", motor.getCurrentPosition());
            telemetry.update();
        }
}
}
