package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;




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

        waitForStart();

        while (opModeIsActive()){
            double targetFL = RunWithEncoder.calculateTicks(60.0);
            double targetFR = RunWithEncoder.calculateTicks(60.0);
            double targetBL = RunWithEncoder.calculateTicks(60.0);
            double targetBR = RunWithEncoder.calculateTicks(60.0);
            encoderDrive(targetFL, targetFR, targetBL, targetBR);

        }
    }
        public void encoderDrive(double targetFL, double targetFR, double targetBL, double targetBR) {
            while(RunWithEncoder.calculatePower(targetFL, frontLeftMotor.getCurrentPosition()) != 0
                    && RunWithEncoder.calculatePower(targetFR, frontRightMotor.getCurrentPosition()) != 0
                    && RunWithEncoder.calculatePower(targetBL, backLeftMotor.getCurrentPosition()) != 0
                    && RunWithEncoder.calculatePower(targetBR, backRightMotor.getCurrentPosition()) != 0){
                frontLeftMotor.setPower(RunWithEncoder.calculatePower(targetFL, frontLeftMotor.getCurrentPosition()));
                frontRightMotor.setPower(RunWithEncoder.calculatePower(targetFR, frontRightMotor.getCurrentPosition()));
                backLeftMotor.setPower(RunWithEncoder.calculatePower(targetBL, backLeftMotor.getCurrentPosition()));
                backRightMotor.setPower(RunWithEncoder.calculatePower(targetBR, backRightMotor.getCurrentPosition()));
            }


    }




    }

