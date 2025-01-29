package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    CRServo servoPinceR;
    CRServo servoPinceL;
    CRServo servoBucket;
    private ElapsedTime runtime = new ElapsedTime();
    double time = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {


        servoPinceR = hardwareMap.get(CRServo.class, "servoPinceR");
        servoPinceL = hardwareMap.get(CRServo.class, "servoPinceL");
        servoBucket = hardwareMap.get(CRServo.class, "servoBucket");

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");
        DcMotor armMotor = hardwareMap.dcMotor.get("arm");
        DcMotor elevatorMotor = hardwareMap.dcMotor.get("elevator");

        DigitalChannel elevatorIn = hardwareMap.digitalChannel.get("elevatorIn");
        DigitalChannel elevatorOut = hardwareMap.digitalChannel.get("elevatorOut");
        DigitalChannel armLimit = hardwareMap.digitalChannel.get("armLimit");

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
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatorIn.setMode(DigitalChannel.Mode.INPUT);
        elevatorOut.setMode(DigitalChannel.Mode.INPUT);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = Math.pow(gamepad1.right_stick_x, 3);

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            if (gamepad2.y) {
                servoPinceR.setPower(-1);
                servoPinceL.setPower(1);
            } else if (gamepad2.a) {
                if (!armLimit.getState()) {
                    servoPinceR.setPower(1);
                    servoPinceL.setPower(-1);
                    time = runtime.time();
                }
                else {
                    servoPinceR.setPower(1);
                    servoPinceL.setPower(-1);
                }
            } else if (time == 0.0) {
                servoPinceR.setPower(0);
                servoPinceL.setPower(0);
            }

            armMotor.setPower(-gamepad2.left_stick_y * 0.30);
            elevatorMotor.setPower(ElevatorFunction.moveElevator(-gamepad2.right_stick_y * 0.75, elevatorIn, elevatorOut));

            if (gamepad2.right_trigger > 0.5) servoBucket.setPower(-1);
            else if (gamepad2.left_trigger > 0.5) servoBucket.setPower(1);
            else servoBucket.setPower(0);

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            if (time != 0.0) {
                if (time + 1.5 <= runtime.time()) {
                    servoPinceR.setPower(0);
                    servoPinceL.setPower(0);
                    time = 0.0;
                }

            }

        }
    }


}
