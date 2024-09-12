package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Perestaltic")
public class PerMotion extends LinearOpMode {
    private DcMotor motor;
    private final int MAX= 6000;
    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        motor = hardwareMap.get(DcMotor.class, "motor0");
        // magnet = hardwareMap.get(TouchSensor.class, "magnet");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if(gamepad1.x && motor.getCurrentPosition() < MAX){
                    motor.setTargetPosition(motor.getCurrentPosition()+28*60);
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motor.setPower(1);
                    motor.setTargetPosition(motor.getCurrentPosition()-28*30);
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motor.setPower(1);
                }
                else{
                    motor.setPower(gamepad1.left_stick_y);
                }
                telemetry.update();
            }
        }
    }


}
