package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp

public class OneMotor extends LinearOpMode {
    private DcMotor motor0, motor1, motor2;

    public void runOpMode() {
        double leftY;
        double rightY;
        
        motor0 = hardwareMap.get(DcMotor.class, "motor0");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        waitForStart();
        while(opModeIsActive()) {
            leftY = gamepad1.left_stick_y;
            rightY = gamepad1.right_stick_y;
            motor0.setPower(leftY * -1);
            motor1.setPower(rightY * -1);
            if(gamepad1.left_trigger>0){
                motor2.setPower(gamepad1.left_trigger);
            }
            else if(gamepad1.right_trigger){
                motor2.setPower(-gamepad1.right_trigger);
            }
            else{
                motor2.setPower(0);
            }
        }
    }
}