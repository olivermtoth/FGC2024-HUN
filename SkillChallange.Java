package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp

public class SkillChallange extends LinearOpMode {

    static final double sqrt2over2 = Math.sqrt(2) / 2;
    private DcMotor drive0;
    private DcMotor drive1;
    private DcMotor drive2;
    private DcMotor drive3;

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
        double EleftXd = 0.0;
        double EleftYd = 0.0;
        double limit = 1.0;

        drive0 = hardwareMap.get(DcMotor.class, "drive0");
        drive1 = hardwareMap.get(DcMotor.class, "drive1");
        drive2 = hardwareMap.get(DcMotor.class, "drive2");
        drive3 = hardwareMap.get(DcMotor.class, "drive3");
        
        waitForStart();
        while (opModeIsActive()) {

            // stick inputs
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
        }
    }
}