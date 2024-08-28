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

public class Final extends LinearOpMode {

    static final double sqrt2over2 = Math.sqrt(2) / 2;
    private Blinker control_Hub;
    private DcMotor drive0;
    private DcMotor drive1;
    private DcMotor drive2;
    private DcMotor drive3;
    private DcMotorEx spin0;
    private DcMotorEx spin1;
    private DcMotor linear;
    private Servo latch;
    private DcMotor unload;
    private TouchSensor BottomMagnet;
    private TouchSensor TopMagnet;
    private Servo o2Release;
    private Servo unhook;
    private Servo slapper;

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
        boolean swapLR = false;
        double servoState = -1;
        int direction = 0;
        boolean useLinear = true;
        boolean g1g2control = true;
        boolean undervoltPortection = true;
        double latchState = 0.0;
        double o2ReleaseState = 1.0;

        boolean g1g2temp = false;
        boolean CornerFrontTemp = false;
        boolean SpinnerTemp = false;
        boolean SpinnerReverseTemp = false;
        boolean UnloaderLinearSwapTemp = false;
        boolean undervoltPortectionTemp = false;
        boolean o2ReleaseTemp = false;
        boolean latchTemp = false;
        boolean limitTemp = false;

        drive0 = hardwareMap.get(DcMotor.class, "drive0");
        drive1 = hardwareMap.get(DcMotor.class, "drive1");
        drive2 = hardwareMap.get(DcMotor.class, "drive2");
        drive3 = hardwareMap.get(DcMotor.class, "drive3");
        spin0 = hardwareMap.get(DcMotorEx.class, "spin0");
        spin1 = hardwareMap.get(DcMotorEx.class, "spin1");
        linear = hardwareMap.get(DcMotor.class, "linear");
        latch = hardwareMap.get(Servo.class, "latch");
        unload = hardwareMap.get(DcMotor.class, "unload");
        BottomMagnet = hardwareMap.get(TouchSensor.class, "BottomMagnet");
        TopMagnet = hardwareMap.get(TouchSensor.class, "TopMagnet");
        o2Release = hardwareMap.get(Servo.class, "o2Release");
        unhook = hardwareMap.get(Servo.class, "unhook");
        slapper = hardwareMap.get(Servo.class, "slapper");

        latch.setDirection(Servo.Direction.FORWARD);
        latch.setPosition(0.0);
        spin0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spin1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spin0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spin1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        latch.setPosition(upPos);
        while (opModeIsActive()) {
            // 1 or 2 controller controls
            if (gamepad1.ps && gamepad1.share && !g1g2temp) {
                g1g2control = !g1g2control;
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

            telemetry.addData("Using 2 controllers", g1g2control); // telemetry

            // 1 controller controls
            if(!g1g2control){ 

                // unloader/linear swap
                if (gamepad1.ps && !gamepad1.share && !UnloaderLinearSwapTemp) {
                    useLinear = !useLinear;
                    UnloaderLinearSwapTemp = true;
                    gamepad1.rumble(150);
                } else if (!gamepad1.ps && !gamepad1.share && UnloaderLinearSwapTemp) {
                    UnloaderLinearSwapTemp = false;
                }

                // linear
                if (useLinear) {
                    unload.setPower(0);
                    if (!gamepad1.triangle && !gamepad1.square) {
                        linear.setPower(0);
                    } else if (gamepad1.triangle && !gamepad1.square) {
                        linear.setPower(1);
                    } else if (!gamepad1.triangle && gamepad1.square) {
                        linear.setPower(-1);
                    }
                    telemetry.addData("TopMagnet: ", TopMagnet.isPressed());
                    telemetry.addData("BottomMagnet: ", BottomMagnet.isPressed());
                } else

                // unload
                if (!useLinear) {
                    linear.setPower(0);
                    if (!gamepad1.triangle && !gamepad1.square) {
                        unload.setPower(0);
                    } else if (gamepad1.triangle && !gamepad1.square) {
                        unload.setPower(1);
                    } else if (!gamepad1.triangle && gamepad1.square) {
                        unload.setPower(-1);
                    }
                }

                telemetry.addData("Using linear over unloading", useLinear); // telemetry

                // stick inputs
                leftX = gamepad1.left_stick_x;
                leftY = gamepad1.left_stick_y;
                rightX = gamepad1.right_stick_x;
                rightY = gamepad1.right_stick_y;

                // corner front
                if (gamepad1.right_bumper && !CornerFrontTemp) {
                    cornerFront = !cornerFront;
                    CornerFrontTemp = true;
                    gamepad1.rumble(250);
                } else if (!gamepad1.right_bumper && CornerFrontTemp) {
                    CornerFrontTemp = false;
                }

                telemetry.addData("Corner front", cornerFront); // telemetry

                // spinner
                if (gamepad1.circle && !gamepad1.share && !SpinnerTemp) {
                    spin = spin == 0 ? 1 : 0;
                    SpinnerTemp = true;
                    gamepad1.rumble(500);
                } else if (!gamepad1.circle && !gamepad1.share && SpinnerTemp) {
                    SpinnerTemp = false;
                }

                if (gamepad1.circle && gamepad1.share && !SpinnerReverseTemp) {
                    spin = spin == 0 ? -1 : 0;
                    SpinnerReverseTemp = true;
                    gamepad1.rumble(500);
                } else if (!gamepad1.circle && gamepad1.share && SpinnerReverseTemp) {
                    SpinnerReverseTemp = false;
                }
                ExpectedVelocity = spin * 26000;

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

                // corner front
                if (gamepad1.right_bumper && !CornerFrontTemp) {
                    cornerFront = !cornerFront;
                    CornerFrontTemp = true;
                    gamepad1.rumble(250);
                } else if (!gamepad1.right_bumper && CornerFrontTemp) {
                    CornerFrontTemp = false;
                }

                // gas pedal
                if (gamepad1.right_trigger > 0.1) {
                    leftY = -gamepad1.right_trigger;
                    leftX = 0;
                }

                if (gamepad1.left_trigger > 0.1) {
                    leftY = gamepad1.left_trigger;
                    leftX = 0;
                }

                // latch
                if (gamepad1.circle && !latchTemp) {
                    latchState = latchState == downPos ? upPos : downPos;
                    latchTemp = true;
                    gamepad1.rumble(150);
                } else if (!gamepad1.circle && latchTemp) {
                    latchTemp = false;
                }
                latch.setPosition(latchState);

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

                //latch
                if (gamepad1.left_bumper == true && limitTemp == false) {
                    if (limit == 0.4) {
                        limit = 1.0;
                    } else {
                        limit = 0.4;
                    }
                    limitTemp = true;
                } else if (gamepad1.left_bumper == false && limitTemp == true) {
                    limitTemp = false;
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

                // unload
                if (Math.abs(gamepad2.right_stick_y) < 0.1) {
                    unload.setPower(0);
                } else {
                    unload.setPower(gamepad2.right_stick_y);
                }

                // spinner
                if (gamepad2.left_bumper && !gamepad2.cross && !SpinnerTemp) {
                    spin = spin == 0 ? 1 : 0;
                    SpinnerTemp = true;
                    gamepad2.rumble(500);
                } else if (!gamepad2.left_bumper && !gamepad2.cross && SpinnerTemp) {
                    SpinnerTemp = false;
                }

                if (!gamepad2.left_bumper && gamepad2.cross && !SpinnerReverseTemp) {
                    spin = spin == 0 ? -1 : 0;
                    SpinnerReverseTemp = true;
                    gamepad2.rumble(500);
                } else if (!gamepad2.left_bumper && !gamepad2.cross && SpinnerReverseTemp) {
                    SpinnerReverseTemp = false;
                }
                ExpectedVelocity = spin * 26000;

                // undervolt protection
                if (gamepad2.dpad_up && !undervoltPortectionTemp) {
                    undervoltPortection = !undervoltPortection;
                    undervoltPortectionTemp = true;
                    gamepad2.rumble(300);
                } else if (!gamepad2.dpad_up && undervoltPortectionTemp) {
                    undervoltPortectionTemp = false;
                }

                // o2 release
                if (gamepad2.triangle && !o2ReleaseTemp) {
                    o2ReleaseState = o2ReleaseState == 1.0 ? 0.0 : 1.0;
                    o2ReleaseTemp = true;
                    gamepad2.rumble(150);
                } else if (!gamepad2.triangle && o2ReleaseTemp) {
                    o2ReleaseTemp = false;
                }
                o2Release.setPosition(o2ReleaseState);

                // unhook
                if (gamepad2.square) {
                    unhook.setPosition(1.0);
                }

                if (gamepad2.ps) gamepad2.rumble(100);

                // slapper
                if (gamepad2.dpad_down) {
                    slapper.setPosition(1.0);
                } else {
                    slapper.setPosition(0.0);
                }

                if (gamepad2.share) {
                    linear.setPower(-1);
                    try {
                      Thread.sleep(5000);
                    }
                    catch (Exception e){
                    }
                    linear.setPower(0);
                  }
            }

            // spinner
            spin0.setPower(-spin); // Reversed 2024.02.08
            spin1.setPower(spin);

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

            //    
            if (hardwareMap.voltageSensor.iterator().next().getVoltage() < 10.0f && undervoltPortection) {
                EleftYd = EleftYd * 0.8;
                EleftXd = EleftXd * 0.8;
                telemetry.addData("CUTTING POWER TO DRIVETRAIN", "");
            }
            telemetry.update();

            // movement
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
                drive2.setPower(sqrt2over2 * (EleftXd + EleftYd));
                drive1.setPower(sqrt2over2 * (EleftXd + EleftYd) * -1);
                drive0.setPower(sqrt2over2 * (EleftXd - EleftYd));
                drive3.setPower(sqrt2over2 * (EleftXd - EleftYd) * -1);
            }
        }
    }
}