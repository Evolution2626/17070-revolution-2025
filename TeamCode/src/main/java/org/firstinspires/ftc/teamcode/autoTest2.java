package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;




@Autonomous
public class autoTest2 extends LinearOpMode {



    static final double COUNTS_PER_MOTOR_REV = 536.6 ;
    static final double WHEEL_DIAMETER_CM = 10.0 ;
    static final double COUNTS_PER_Cm = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_CM * 3.1415);

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

        encoderDrive(30.0, 30.0, 30.0, 30.0);
        encoderDrive(30.0, 30.0, 30.0, 30.0);
        encoderDrive(30.0, -30.0, 30.0, -30.0);
    }
    public void encoderDrive(
            double flCm, double frCm, double blCm, double brCm) {
        double newflTarget = 0.0;
        double newfrTarget = 0.0;
        double newblTarget = 0.0;
        double newbrTarget = 0.0;


        if (opModeIsActive()) {

            newflTarget = frontLeftMotor.getCurrentPosition() + (flCm * COUNTS_PER_Cm);
            newfrTarget = frontRightMotor.getCurrentPosition() + (frCm * COUNTS_PER_Cm);
            newblTarget = backLeftMotor.getCurrentPosition() + (blCm * COUNTS_PER_Cm);
            newbrTarget = backRightMotor.getCurrentPosition() + (brCm * COUNTS_PER_Cm);

            frontLeftMotor.setTargetPosition((int) newflTarget);
            frontRightMotor.setTargetPosition((int)newfrTarget);
            backLeftMotor.setTargetPosition((int)newblTarget);
            backRightMotor.setTargetPosition((int)newbrTarget);


            while (opModeIsActive() &&

                    (frontLeftMotor.isBusy() || frontRightMotor.isBusy() || backLeftMotor.isBusy() || backRightMotor.isBusy())){
            }


        }

    }
}
