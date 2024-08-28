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

public class New2024 extends LinearOpMode {

    static final double SQRT2OVER2 = Math.sqrt(2) / 2;
    private DcMotor drive0;
    private DcMotor drive1;
    private DcMotor drive2;
    private DcMotor drive3;

    private DcMotor linear0;
    private DcMotor linear1;
    
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
        double spin = 0.0;
        double upPos = 0.35;
        double downPos = 0.25;
        double ExpectedVelocity = 0.0;

        boolean cornerFront = false;
        int direction = 0;
        boolean useLinear = true;
        boolean twoDriver = true;
        boolean undervoltPortection = true;
        double latchState = 0.0;
        double o2ReleaseState = 1.0;

        //Temporary variables
        boolean g1g2temp = false;
        boolean CornerFrontTemp = false;
        boolean SpinnerTemp = false;
        boolean SpinnerReverseTemp = false;
        boolean UnloaderLinearSwapTemp = false;
        boolean undervoltPortectionTemp = false;
        boolean o2ReleaseTemp = false;
        boolean latchTemp = false;
        boolean limitTemp = false;

        //Motors
        drive0 = hardwareMap.get(DcMotor.class, "drive0");
        drive1 = hardwareMap.get(DcMotor.class, "drive1");
        drive2 = hardwareMap.get(DcMotor.class, "drive2");
        drive3 = hardwareMap.get(DcMotor.class, "drive3");
        linear = hardwareMap.get(DcMotor.class, "linear");

        waitForStart();

        while(opModeIsActive()) {
            // Switching between 1 and 2 controller mode
            if (gamepad1.ps && gamepad1.share && !g1g2temp) {
                twoDriver = !twoDriver;
                g1g2temp = true;
                gamepad1.rumble(1000);
                gamepad2.rumble(1000);
                // resetting temp variables
                CornerFrontTemp = false;
                SpinnerTemp = false;
                SpinnerReverseTemp = false;
                UnloaderLinearSwapTemp = false;
            } else if (!gamepad1.ps && !gamepad1.share && g1g2temp) {
                g1g2temp = false;
            }

            telemetry.addData("Two driver mode:", twoDriver); // telemetry

            // 1 driver mode
            if(!twoDriver){ 

                // Linear Motion
                if (useLinear) {
                    unload.setPower(0);
                    if (!gamepad1.triangle && !gamepad1.square) {
                        linear.setPower(0);
                    } else if (gamepad1.triangle && !gamepad1.square && !TopMagnet.isPressed()) {
                        linear.setPower(1);
                    } else if (!gamepad1.triangle && gamepad1.square && !BottomMagnet.isPressed()) {
                        linear.setPower(-1);
                    }
                } else


                telemetry.addData("Linear:", useLinear); // telemetry

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

                // direction
                if (gamepad1.dpad_up) {
                    direction = 0;
                    gamepad1.rumble(150);
                } else if (gamepad1.dpad_right) {
                    direction = 1;
                    gamepad1.rumble(150);
                } else if (gamepad1.dpad_down) {
                    direction = 2;
                    gamepad1.rumble(150);
                } else if (gamepad1.dpad_left) {
                    direction = 3;
                    gamepad1.rumble(150);
                }

                telemetry.addData("Direction", direction); // telemetry
            } 

            // 2 controller controls
            else{ 
                // ================= GAMEPAD 1 =================
                // linear
                if (!gamepad1.triangle && !gamepad1.square && Math.abs(gamepad2.left_stick_y) < 0.1) {
                    linear.setPower(0);
                } else if (gamepad1.triangle && !gamepad1.square && Math.abs(gamepad2.left_stick_y) < 0.1) {
                    linear.setPower(1);
                } else if (!gamepad1.triangle && gamepad1.square && Math.abs(gamepad2.left_stick_y) < 0.1) {
                    linear.setPower(-1);
                }

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


                // direction
                if (gamepad1.dpad_up) {
                    direction = 0;
                    gamepad1.rumble(150);
                } else if (gamepad1.dpad_right) {
                    direction = 1;
                    gamepad1.rumble(150);
                } else if (gamepad1.dpad_down) {
                    direction = 2;
                    gamepad1.rumble(150);
                } else if (gamepad1.dpad_left) {
                    direction = 3;
                    gamepad1.rumble(150);
                }

                // ================= GAMEPAD 2 =================

                // linear
                if (!gamepad1.triangle && !gamepad1.square) {
                    if (Math.abs(gamepad2.left_stick_y) > 0.1) {
                        linear.setPower(-gamepad2.left_stick_y);
                    } else {
                        linear.setPower(0);
                    }
                }


                // undervolt protection
                if (gamepad2.dpad_up && !undervoltPortectionTemp) {
                    undervoltPortection = !undervoltPortection;
                    undervoltPortectionTemp = true;
                    gamepad2.rumble(300);
                } else if (!gamepad2.dpad_up && undervoltPortectionTemp) {
                    undervoltPortectionTemp = false;
                }

                



            // spinner unstuck-er
            /*
             * if (Math.abs(spin0.getVelocity()) < ExpectedVelocity * 0.4) {
             * spin0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
             * spin0.setTargetPosition((int) 140);
             * spin0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             * spin0.setPower(spin);
             * }
             * if (Math.abs(spin1.getVelocity()) < ExpectedVelocity * 0.4) {
             * spin1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
             * spin1.setTargetPosition((int) -112);
             * spin1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             * spin1.setPower(spin);
             * }
             * 
             * if (spin0.getMode() == DcMotor.RunMode.RUN_TO_POSITION &&
             * spin0.getCurrentPosition() >= 140) {
             * spin0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
             * spin0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
             * }
             * if (spin1.getMode() == DcMotor.RunMode.RUN_TO_POSITION &&
             * spin1.getCurrentPosition() <= -112) {
             * spin1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
             * spin1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
             * }
             */

            // square
            EleftY = Math.pow(leftY, 2) * Math.signum(leftY) * limit;
            EleftX = Math.pow(leftX, 2) * Math.signum(leftX) * limit;
            ErightY = Math.pow(rightY, 2) * Math.signum(rightY) * 0.4;
            ErightX = Math.pow(rightX, 2) * Math.signum(rightX) * 0.4;

            // direction switch
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

            // telemetry
            // telemetry.addData("Spinner power", "0(%.2f), 1(%.2f)", spin0.getPower(), spin1.getPower());
            // telemetry.addData("Spinner velocity", "0(%d), 1(%d)", spin0.getVelocity(), spin1.getVelocity());
            telemetry.addData("G1 Effective LeftY", EleftY);
            telemetry.addData("G1 Effective LeftX", EleftX);
            telemetry.addData("G1 Effective RightY", ErightY);
            telemetry.addData("G1 Effective RightX", ErightX);

            // Undervolt Protection 
            if (hardwareMap.voltageSensor.iterator().next().getVoltage() < 10.0f && undervoltPortection) {
                EleftYd = EleftYd * 0.8;
                EleftXd = EleftXd * 0.8;
                telemetry.addData("CUTTING POWER TO DRIVETRAIN", "");
            }
            telemetry.update();

            // Movements 
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
            } else if (cornerFront) {
                drive2.setPower(EleftYd);
                drive1.setPower(-EleftYd);
                drive0.setPower(EleftXd);
                drive3.setPower(-EleftXd);
            } else if (!cornerFront) {
                drive2.setPower(SQRT2OVER2 * (EleftXd + EleftYd));
                drive1.setPower(SQRT2OVER2 * (EleftXd + EleftYd) * -1);
                drive0.setPower(SQRT2OVER2 * (EleftXd - EleftYd));
                drive3.setPower(SQRT2OVER2 * (EleftXd - EleftYd) * -1);
            }
        }
    }
}}