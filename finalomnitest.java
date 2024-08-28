public package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class OmniTest extends LinearOpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor frontMotor;
    private DcMotor rearMotor;

    @Override
    public void runOpMode() {
        leftMotor = hardwareMap.get(DcMotor.class, "left"); // port 0
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor = hardwareMap.get(DcMotor.class, "right"); // port 1
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontMotor = hardwareMap.get(DcMotor.class, "front"); // port 2
        frontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearMotor = hardwareMap.get(DcMotor.class, "rear"); // port 2
        rearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        double counter = 0.00;
        boolean circlePressed = false;
        boolean squarePressed = false;
        while (opModeIsActive()) {
            // float leftX = gamepad1.left_stick_x;
            float leftY = gamepad1.left_stick_y;
            
            if(gamepad1.circle & !circlePressed & counter < 1){
                    circlePressed = true;
                    counter += (double)0.01;
                }
                else if(!gamepad1.circle & circlePressed){
                    circlePressed = false;
                }
             
             if(gamepad1.square & !squarePressed & counter > 0){
                    squarePressed = true;
                    counter -= (double)0.01;}
            else if(!gamepad1.square & squarePressed){
                    squarePressed = false;}
            
            
                
            //frontMotor.setPower(counter);
            frontMotor.setPower(leftY);
            telemetry.addData("leftY",leftY);
            telemetry.update();
        }
    }
} {
    
}
