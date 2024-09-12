package org.firstinspires.ftc.teamcode;
    
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp

public class Alpha extends LinearOpMode {

    static final double sqrt2over2 = Math.sqrt(2) / 2;
    private Blinker control_Hub;
    private DcMotor drive0;
    private DcMotor drive1;
    private DcMotor drive2;
    private DcMotor drive3;
    private DcMotor linearMotionMotor0;
    private DcMotor linearMotionMotor1;
    private DcMotor linearMotionMotor2;
    private final int[] height = {50, 1600, 3800, 6300};


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
        int direction = 0;
        int heightIndex = 0;
        boolean circlePressed = false;


        drive0 = hardwareMap.get(DcMotor.class, "drive0");
        drive1 = hardwareMap.get(DcMotor.class, "drive1");
        drive2 = hardwareMap.get(DcMotor.class, "drive2");
        drive3 = hardwareMap.get(DcMotor.class, "drive3");

        linearMotionMotor0 = hardwareMap.get(DcMotor.class, "balu");
        linearMotionMotor1 = hardwareMap.get(DcMotor.class, "etele");
        linearMotionMotor2 = hardwareMap.get(DcMotor.class, "pez");

        linearMotionMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            leftX = gamepad1.left_stick_x;
            leftY = gamepad1.left_stick_y;
            rightX = gamepad1.right_stick_x;
            rightY = gamepad1.right_stick_y;

            if (gamepad1.right_trigger > 0.1) {
                leftY = -gamepad1.right_trigger;
                leftX = 0;
            }

            if (gamepad1.left_trigger > 0.1) {
                leftY = gamepad1.left_trigger;
                leftX = 0;
            }

            EleftY = Math.pow(leftY, 2) * Math.signum(leftY) * limit;
            EleftX = Math.pow(leftX, 2) * Math.signum(leftX) * limit;
            ErightY = Math.pow(rightY, 2) * Math.signum(rightY) * 0.4;
            ErightX = Math.pow(rightX, 2) * Math.signum(rightX) * 0.4;

            switch (direction) {
                case 2:
                    EleftXd = -EleftX;
                    EleftYd = -EleftY;
                    break;
                case 3:
                    EleftXd = EleftY;
                    EleftYd = -EleftX;
                    break;
                case 0:
                    EleftXd = EleftX;
                    EleftYd = EleftY;
                    break;
                case 1:
                    EleftXd = -EleftY;
                    EleftYd = EleftX;
                    break;
            }


            drive2.setPower(sqrt2over2 * (EleftXd + EleftYd));
            drive1.setPower(sqrt2over2 * (EleftXd + EleftYd) * -1);
            drive0.setPower(sqrt2over2 * (EleftXd - EleftYd));
            drive3.setPower(sqrt2over2 * (EleftXd - EleftYd) * -1);

            // control of Balu's linear motion motor
            if(gamepad1.dpad_up){
                linearMotionMotor0.setPower(-1);
            }
            else if(gamepad1.dpad_down){
                linearMotionMotor0.setPower(1);
            }
            else{
                linearMotionMotor0.setPower(0);
            }

            // control of Eteles's linear motion motor
            if(gamepad1.circle & !circlePressed){
                heightIndex = (heightIndex + 1)%4;
                circlePressed = true;
            }
            else if(!gamepad1.circle & circlePressed){
                circlePressed = false;
            }

            if(gamepad1.triangle){
                linearMotionMotor1.setTargetPosition(-height[heightIndex]);
                linearMotionMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearMotionMotor1.setPower(0.5);
            }

            if(gamepad1.square){
                heightIndex = 0;
                linearMotionMotor1.setTargetPosition(-height[heightIndex]);
                linearMotionMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearMotionMotor1.setPower(0.5);
            }



            telemetry.addData("Position", heightIndex);
            telemetry.addData("encoder", linearMotionMotor1.getCurrentPosition());

            // control of PEZ linear motion motor
            if(gamepad1.left_bumper){
                linearMotionMotor2.setPower(1);
            }
            else if(gamepad1.right_bumper){
                linearMotionMotor2.setPower(-1);
            }
            else{
                linearMotionMotor2.setPower(0);
            }

            telemetry.update();
        }
    }
}