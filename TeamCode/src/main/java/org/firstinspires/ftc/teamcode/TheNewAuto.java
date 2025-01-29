package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous
public class TheNewAuto extends LinearOpMode {

    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    DcMotor elevatorMotor;

    CRServo servoBucket;
    private ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 536.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.8;
    static final double TURN_SPEED = 0.5;


    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        elevatorMotor = hardwareMap.dcMotor.get("elevator");
        servoBucket = hardwareMap.get(CRServo.class, "servoBucket");


        DigitalChannel elevatorIn = hardwareMap.digitalChannel.get("elevatorIn");
        DigitalChannel elevatorOut = hardwareMap.digitalChannel.get("elevatorOut");


        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            encoderDrive(DRIVE_SPEED, 45, 45, 45, 45, 15.0);
            encoderDrive(TURN_SPEED, 30, -30, -30, 30, 15.0);
            elevatorMotor.setPower(ElevatorFunction.moveElevator(0.75, elevatorIn, elevatorOut));
            encoderDrive(DRIVE_SPEED, 35, 35, 35, 35, 15.0);
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
            elevatorMotor.setPower(ElevatorFunction.moveElevator(-0.75, elevatorIn, elevatorOut));
            sleep(500);
            servoBucket.setPower(0);
            sleep(400);
            elevatorMotor.setPower(0);
            sleep(100000);

        }
    }

    public void encoderDrive(double speed,
                             double FLInches, double FRInches,
                             double BLInches, double BRInches,
                             double timeoutS) {
        int newFLeftTarget;
        int newFRightTarget;
        int newBLeftTarget;
        int newBRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFLeftTarget = frontLeftMotor.getCurrentPosition() + (int) (FLInches * COUNTS_PER_INCH);
            newFRightTarget = frontRightMotor.getCurrentPosition() + (int) (FRInches * COUNTS_PER_INCH);
            newBLeftTarget = backLeftMotor.getCurrentPosition() + (int) (BLInches * COUNTS_PER_INCH);
            newBRightTarget = backRightMotor.getCurrentPosition() + (int) (BRInches * COUNTS_PER_INCH);

            frontLeftMotor.setTargetPosition(newFLeftTarget);
            frontRightMotor.setTargetPosition(newFRightTarget);
            backLeftMotor.setTargetPosition(newBLeftTarget);
            backRightMotor.setTargetPosition(newBRightTarget);

            // Turn On RUN_TO_POSITION
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontLeftMotor.setPower(Math.abs(speed));
            frontRightMotor.setPower(Math.abs(speed));
            backLeftMotor.setPower(Math.abs(speed));
            backRightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and all motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy())) {

            }

            // Stop all motion;
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
}
