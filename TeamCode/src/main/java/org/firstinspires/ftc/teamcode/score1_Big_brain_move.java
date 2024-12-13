package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.VoltageSensor;


@Autonomous
public class score1_Big_brain_move extends LinearOpMode {

    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    DcMotor elevatorMotor;

    CRServo servoBucket;
    DigitalChannel elevatorIn;
    DigitalChannel elevatorOut;


    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        elevatorMotor = hardwareMap.dcMotor.get("elevator");
        servoBucket = hardwareMap.get(CRServo.class, "servoBucket");


        elevatorIn = hardwareMap.digitalChannel.get("elevatorIn");
        elevatorOut = hardwareMap.digitalChannel.get("elevatorOut");


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
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();

        while (opModeIsActive()) {
            double targetFL = DrivetrainFunction.calculateTicks(-45.0);
            double targetFR = DrivetrainFunction.calculateTicks(-45.0);
            double targetBL = DrivetrainFunction.calculateTicks(-45.0);
            double targetBR = DrivetrainFunction.calculateTicks(-45.0);
            encoderDrive(targetFL, targetFR, targetBL, targetBR);
            sleep(300);
            targetFL = targetFL + DrivetrainFunction.calculateTicks(-35.0);
            targetFR = targetFR + DrivetrainFunction.calculateTicks(35.0);
            targetBL = targetBL + DrivetrainFunction.calculateTicks(-35.0);
            targetBR =  targetBR + DrivetrainFunction.calculateTicks(35.0);
            encoderDrive(targetFL, targetFR, targetBL, targetBR);
            sleep(300);
            elevatorMotor.setPower(ElevatorFunction.moveElevator(0.75, elevatorIn, elevatorOut));

            targetFL = targetFL + DrivetrainFunction.calculateTicks(60.0);
            targetFR = targetFR + DrivetrainFunction.calculateTicks(60.0);
            targetBL = targetBL + DrivetrainFunction.calculateTicks(60.0);
            targetBR =  targetBR + DrivetrainFunction.calculateTicks(60.0);
            encoderDrive(targetFL, targetFR, targetBL, targetBR);
            elevatorMotor.setPower(0);
            sleep(500);
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            servoBucket.setPower(-1);
            sleep(1800);

            servoBucket.setPower(1);
            sleep(500);
            elevatorMotor.setPower(ElevatorFunction.moveElevator(-0.75, elevatorIn, elevatorOut));
            sleep(700);
            servoBucket.setPower(0);
            sleep(400);
            elevatorMotor.setPower(0);
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

