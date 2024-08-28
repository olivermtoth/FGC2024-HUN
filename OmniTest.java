package org.firstinspires.ftc.teamcode;

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
        leftMotor.setMode(DcMotor.RunMode.RUN_WITH_ENCODER);
        rightMotor = hardwareMap.get(DcMotor.class, "right"); // port 1
        rightMotor.setMode(DcMotor.RunMode.RUN_WITH_ENCODER);
        frontMotor = hardwareMap.get(DcMotor.class, "front"); // port 2
        frontMotor.setMode(DcMotor.RunMode.RUN_WITH_ENCODER);
        rearMotor = hardwareMap.get(DcMotor.class, "rear"); // port 2
        rearMotor.setMode(DcMotor.RunMode.RUN_WITH_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
            float leftX = gamepad1.left_stick_x;
            float leftY = gamepad1.left_stick_y;

            leftMotor.setPower(leftX+leftY);
            rightMotor.setPower(-(leftX+leftY));
            frontMotor.setPower(leftX);
            rearMotor.setPower(-leftX);   
        }
    }
}