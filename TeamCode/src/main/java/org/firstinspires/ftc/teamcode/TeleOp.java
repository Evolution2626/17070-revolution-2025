package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    boolean armOut = false;
    boolean elevatorOut = false;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");
        DcMotor elevatorMotor = hardwareMap.dcMotor.get("elevator");
        DcMotor armMotor = hardwareMap.dcMotor.get("arm");

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
            boolean aButton = gamepad2.a;
            boolean xButton = gamepad2.x;
            boolean yButton = gamepad2.y;
            boolean bButton = gamepad2.b;


            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            if(aButton){
                if(armOut){
                   armMotor.setTargetPosition((int)armInPos);
                }
                else{
                    armMotor.setTargetPosition((int)armOutPos);
                }
                armMotor.setPower(0.5);// TODO check for the right amount of power
                armOut = !armOut;
            }
            if(Math.abs(armMotor.getCurrentPosition() - armMotor.getTargetPosition()) < 50){// TODO check for the right encoder position tolerance
                armMotor.setPower(0);
            }

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
    }
}
