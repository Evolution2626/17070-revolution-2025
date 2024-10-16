package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {

    double flEncoderPos;
    double frEncoderPos;
    double blEncoderPos;
    double brEncoderPos;

    double armOutPos;
    double armInPos;
    double elevatorInPos;
    double elevatorOutPos;


    Servo servo1;
    Servo servo2;
    @Override
    public void runOpMode() throws InterruptedException {

        servo1 = hardwareMap.get(Servo.class, "pince1");
        servo2 = hardwareMap.get(Servo.class, "pince2");
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");
        DcMotor elevatorMotor = hardwareMap.dcMotor.get("elevator");
        DcMotor armMotor = hardwareMap.dcMotor.get("arm");
        DigitalChannel elevatorIn = hardwareMap.digitalChannel.get("elevatorIn");
        DigitalChannel elevatorOut = hardwareMap.digitalChannel.get("elevatorOut");
        AnalogInput armSensor = hardwareMap.analogInput.get("armSensor");

        double flEncoder = frontLeftMotor.getCurrentPosition();
        double frEncoder = frontRightMotor.getCurrentPosition();
        double blEncoder = backLeftMotor.getCurrentPosition();
        double brEncoder = backRightMotor.getCurrentPosition();

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatorIn.setMode(DigitalChannel.Mode.INPUT);
        elevatorOut.setMode(DigitalChannel.Mode.INPUT);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flEncoderPos = flEncoder;
        frEncoderPos = frEncoder;
        blEncoderPos = blEncoder;
        brEncoderPos = brEncoder;

        elevatorInPos = elevatorMotor.getCurrentPosition();
        armInPos = armMotor.getCurrentPosition();

        elevatorOutPos = elevatorInPos + 0;// TODO add the magic maximum value
        armOutPos = armInPos + 0;// TODO same thing here

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            if(gamepad1.x){
                servo1.setPosition(0.25);
                servo2.setPosition(0.75);
            }
            if(gamepad1.y){
                servo1.setPosition(0.75);
                servo2.setPosition(0.25);
            }
            if(gamepad1.a){
                moveElevator(1.0, elevatorMotor, elevatorIn, elevatorOut);
            }
            if(gamepad1.b){
                moveElevator(-1.0, elevatorMotor, elevatorIn, elevatorOut);
            }


            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
    }
    public void moveArm(double power, DcMotor armMotor, AnalogInput armSensor){
        if(power >= 0 && armSensor.getVoltage() < 0.75){
            armMotor.setPower(power);
        }
        if(power >= 0 && armSensor.getVoltage() > 2.55){
            armMotor.setPower(power);
        }
        else{
            armMotor.setPower(0.0);
        }

    }
    public void moveElevator(double power, DcMotor elevatorMotor, DigitalChannel elevatorIn, DigitalChannel elevatorOut){
        if(elevatorIn.getState() && power >= 0){
            elevatorMotor.setPower(power);
        }
        if(elevatorOut.getState() && power >= 0){
            elevatorMotor.setPower(power);
        }
        else{
            elevatorMotor.setPower(0.0);
        }

    }
}
