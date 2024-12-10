package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;



@Autonomous
public class score2 extends LinearOpMode {

    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    DcMotor elevatorMotor;

    CRServo servoBucket;
    CRServo servoPinceR;
    CRServo servoPinceL;


    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        elevatorMotor = hardwareMap.dcMotor.get("elevator");
        DcMotor armMotor = hardwareMap.dcMotor.get("arm");
        servoBucket = hardwareMap.get(CRServo.class, "servoBucket");
        servoPinceR = hardwareMap.get(CRServo.class, "servoPinceR");
        servoPinceL = hardwareMap.get(CRServo.class, "servoPinceL");


        DigitalChannel elevatorIn = hardwareMap.digitalChannel.get("elevatorIn");
        DigitalChannel elevatorOut = hardwareMap.digitalChannel.get("elevatorOut");


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
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            double targetFL = DrivetrainFunction.calculateTicks(-35.0);
            double targetFR = DrivetrainFunction.calculateTicks(-35.0);
            double targetBL = DrivetrainFunction.calculateTicks(-35.0);
            double targetBR = DrivetrainFunction.calculateTicks(-35.0);
            encoderDrive(targetFL, targetFR, targetBL, targetBR);
            sleep(500);
            targetFL = frontLeftMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(-22.0);
            targetFR = frontRightMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(22.0);
            targetBL = backLeftMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(-22.0);
            targetBR =  backRightMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(22.0);
            encoderDrive(targetFL, targetFR, targetBL, targetBR);
            sleep(500);
            elevatorMotor.setPower(ElevatorFunction.moveElevator(0.75, elevatorIn, elevatorOut));

            targetFL = frontLeftMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(40.0);
            targetFR = frontRightMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(40.0);
            targetBL = backLeftMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(40.0);
            targetBR =  backRightMotor.getCurrentPosition() +DrivetrainFunction.calculateTicks(40.0);
            encoderDrive(targetFL, targetFR, targetBL, targetBR);
            sleep(500);
            elevatorMotor.setPower(0);
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            servoBucket.setPower(-1);
            sleep(1800);

            servoBucket.setPower(1);
            sleep(500);

            while(elevatorOut.getState()) {
                elevatorMotor.setPower(-0.85);

            }
            elevatorMotor.setPower(0);
            servoBucket.setPower(0);

            targetFL = frontLeftMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(-20.0);
            targetFR = frontRightMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(-20.0);
            targetBL = backLeftMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(-20.0);
            targetBR =  backRightMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(-20.0);
            encoderDrive(targetFL, targetFR, targetBL, targetBR);
            sleep(300);
            targetFL = frontLeftMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(20.0);
            targetFR = frontRightMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(-30.0);
            targetBL = backLeftMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(20.0);
            targetBR =  backRightMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(-30.0);
            encoderDrive(targetFL, targetFR, targetBL, targetBR);
            sleep(300);
            armMotor.setPower(-0.5);
            sleep(1000);
            armMotor.setPower(0);
            servoPinceR.setPower(0);
            servoPinceL.setPower(0);
            targetFL = frontLeftMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(-10.0);
            targetFR = frontRightMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(-10.0);
            targetBL = backLeftMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(-10.0);
            targetBR =  backRightMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(-10.0);
            encoderDrive(targetFL, targetFR, targetBL, targetBR);

            sleep(600);
            servoPinceR.setPower(-1);
            servoPinceL.setPower(1);
            sleep(1000);
            armMotor.setPower(0.5);
            sleep(1000);
            armMotor.setPower(0);
            servoPinceR.setPower(1);
            servoPinceL.setPower(-1);
            sleep(1000);
            servoPinceR.setPower(0);
            servoPinceL.setPower(0);
            targetFL = frontLeftMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(10.0);
            targetFR = frontRightMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(10.0);
            targetBL = backLeftMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(10.0);
            targetBR =  backRightMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(10.0);
            encoderDrive(targetFL, targetFR, targetBL, targetBR);
            sleep(300);
            targetFL = frontLeftMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(-25.0);
            targetFR = frontRightMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(30.0);
            targetBL = backLeftMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(-25.0);
            targetBR =  backRightMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(30.0);
            encoderDrive(targetFL, targetFR, targetBL, targetBR);
            sleep(300);
            targetFL = frontLeftMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(20.0);
            targetFR = frontRightMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(20.0);
            targetBL = backLeftMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(20.0);
            targetBR =  backRightMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(20.0);
            encoderDrive(targetFL, targetFR, targetBL, targetBR);
            sleep(300);
            elevatorMotor.setPower(ElevatorFunction.moveElevator(0.75, elevatorIn, elevatorOut));
            sleep(1000);
            elevatorMotor.setPower(0);
            servoBucket.setPower(-1);
            sleep(1000);
            servoBucket.setPower(1);
            sleep(1000);
            servoBucket.setPower(0);
            while(elevatorOut.getState()) {
                elevatorMotor.setPower(-0.85);

            }

            elevatorMotor.setPower(0);
            sleep(300);
            targetFL = frontLeftMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(25.0);
            targetFR = frontRightMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(-30.0);
            targetBL = backLeftMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(25.0);
            targetBR =  backRightMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(-30.0);
            encoderDrive(targetFL, targetFR, targetBL, targetBR);
            sleep(300);
            targetFL = frontLeftMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(-10.0);
            targetFR = frontRightMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(-10.0);
            targetBL = backLeftMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(-10.0);
            targetBR =  backRightMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(-10.0);
            encoderDrive(targetFL, targetFR, targetBL, targetBR);
            sleep(300);
            armMotor.setPower(-0.5);
            sleep(1000);
            armMotor.setPower(0);
            servoPinceR.setPower(0);
            servoPinceL.setPower(0);
            targetFL = frontLeftMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(-20.0);
            targetFR = frontRightMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(-20.0);
            targetBL = backLeftMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(-20.0);
            targetBR =  backRightMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(-20.0);
            encoderDrive(targetFL, targetFR, targetBL, targetBR);

            sleep(600);
            servoPinceR.setPower(-1);
            servoPinceL.setPower(1);
            sleep(1000);
            armMotor.setPower(0.5);
            sleep(1000);
            armMotor.setPower(0);
            servoPinceR.setPower(1);
            servoPinceL.setPower(-1);
            sleep(1000);
            servoPinceR.setPower(0);
            servoPinceL.setPower(0);
            targetFL = frontLeftMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(40.0);
            targetFR = frontRightMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(40.0);
            targetBL = backLeftMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(40.0);
            targetBR =  backRightMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(40.0);
            encoderDrive(targetFL, targetFR, targetBL, targetBR);
            elevatorMotor.setPower(ElevatorFunction.moveElevator(0.75, elevatorIn, elevatorOut));
            targetFL = frontLeftMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(-30.0);
            targetFR = frontRightMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(40.0);
            targetBL = backLeftMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(-30.0);
            targetBR =  backRightMotor.getCurrentPosition() + DrivetrainFunction.calculateTicks(40.0);
            encoderDrive(targetFL, targetFR, targetBL, targetBR);
            sleep(800);
            elevatorMotor.setPower(0);
            servoBucket.setPower(-1);
            sleep(1000);
            servoBucket.setPower(1);
            sleep(1000);
            servoBucket.setPower(0);
            elevatorMotor.setPower(ElevatorFunction.moveElevator(-0.85, elevatorIn, elevatorOut));
            sleep(700);
            elevatorMotor.setPower(0);
            sleep(300);

            break;

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

