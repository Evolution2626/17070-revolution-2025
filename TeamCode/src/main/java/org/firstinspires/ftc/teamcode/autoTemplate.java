package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.*;




@Disabled
@Autonomous
public class autoTemplate extends LinearOpMode {

    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");


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
        DrivetrainFunction.calculateTicks(120.0);
        waitForStart();

        while (opModeIsActive()) {
            double targetFL = DrivetrainFunction.calculateTicks(120.0);
            double targetFR = DrivetrainFunction.calculateTicks(120.0);
            double targetBL = DrivetrainFunction.calculateTicks(120.0);
            double targetBR = DrivetrainFunction.calculateTicks(120.0);
            encoderDrive(targetFL, targetFR, targetBL, targetBR);

            targetFL = frontLeftMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(-120.0);
            targetFR = frontRightMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(-120.0);
            targetBL = backLeftMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(-120.0);
            targetBR =  backRightMotor.getCurrentPosition() +DrivetrainFunction.calculateTicks(-120.0);
            encoderDrive(targetFL, targetFR, targetBL, targetBR);
            sleep(100000);

        }
    }

    public void encoderDrive(double targetFL, double targetFR, double targetBL, double targetBR) {
        while (DrivetrainFunction.calculatePower(targetFL, frontLeftMotor.getCurrentPosition()) != 0
                && DrivetrainFunction.calculatePower(targetFR, frontRightMotor.getCurrentPosition()) != 0
                && DrivetrainFunction.calculatePower(targetBL, backLeftMotor.getCurrentPosition()) != 0
                && DrivetrainFunction.calculatePower(targetBR, backRightMotor.getCurrentPosition()) != 0 && opModeIsActive()) {
            frontLeftMotor.setPower(DrivetrainFunction.calculatePower(targetFL, frontLeftMotor.getCurrentPosition()));
            frontRightMotor.setPower(DrivetrainFunction.calculatePower(targetFR, frontRightMotor.getCurrentPosition()));
            backLeftMotor.setPower(DrivetrainFunction.calculatePower(targetBL, backLeftMotor.getCurrentPosition()));
            backRightMotor.setPower(DrivetrainFunction.calculatePower(targetBR, backRightMotor.getCurrentPosition()));
        }
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);


    }


}

