package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@Autonomous
public class autoTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 0 ;// TODO add the right value
    static final double WHEEL_DIAMETER_CM = 0 ;
    static final double COUNTS_PER_Cm = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_CM * 3.1415);

    DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
    DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
    DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
    DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");

    int flEncoder = frontLeftMotor.getCurrentPosition();
    int frEncoder = frontRightMotor.getCurrentPosition();
    int blEncoder = backLeftMotor.getCurrentPosition();
    int brEncoder = backRightMotor.getCurrentPosition();

    @Override
    public void runOpMode() throws InterruptedException {
        setup();
        waitForStart();
        driveWithEncoder(0.0, 30.0, 0.0, 0.0);
    }
        public void setup(){
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
        }
        public void driveWithEncoder(double yCm, double xCm, double rxDg, double timeouts){
            double frontLeftCm = (yCm + xCm + rxDg);
            double backLeftCm = (yCm - xCm + rxDg);
            double frontRightCm = (yCm - xCm - rxDg);
            double backRightCm = (yCm + xCm - rxDg);
            encoderDrive(frontLeftCm, frontRightCm, backLeftCm, backRightCm,timeouts);
        }

        public void encoderDrive(
        double flCm, double frCm, double blCm, double brCm,
        double timeoutS) {
            double newflTarget;
            double newfrTarget;
            double newblTarget;
            double newbrTarget;


            if (opModeIsActive()) {

                newflTarget = flEncoder + (int)(flCm * COUNTS_PER_Cm);
                newfrTarget = frEncoder + (int)(frCm * COUNTS_PER_Cm);
                newblTarget = blEncoder + (int)(blCm * COUNTS_PER_Cm);
                newbrTarget = brEncoder + (int)(brCm * COUNTS_PER_Cm);

                double[] target = {Math.abs(newflTarget), Math.abs(newfrTarget), Math.abs(newblTarget), Math.abs(newbrTarget)};
                Arrays.sort(target);
                double maxSpeed = 1.0;
                double flSpeed = minSpeed(newflTarget, map(newflTarget, target[0], target[3], 0.0, maxSpeed));
                double frSpeed = minSpeed(newfrTarget, map(newfrTarget, target[0], target[3], 0.0, maxSpeed));
                double blSpeed = minSpeed(newblTarget, map(newblTarget, target[0], target[3], 0.0, maxSpeed));
                double brSpeed = minSpeed(newbrTarget, map(newbrTarget, target[0], target[3], 0.0, maxSpeed));

                frontLeftMotor.setTargetPosition((int) newflTarget);
                frontRightMotor.setTargetPosition((int)newfrTarget);
                backLeftMotor.setTargetPosition((int)newblTarget);
                backRightMotor.setTargetPosition((int)newbrTarget);

                frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                runtime.reset();
                frontLeftMotor.setPower(Math.abs(flSpeed));
                frontRightMotor.setPower(Math.abs(frSpeed));
                backLeftMotor.setPower(Math.abs(blSpeed));
                backRightMotor.setPower(Math.abs(brSpeed));


                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (frontLeftMotor.isBusy() && frontRightMotor.isBusy()));



                frontLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);

                frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                sleep(250);
            }

}
    public double minSpeed(double position,double speed){
        if(speed >= 0.2) return speed;
        if(position > 0) return 0.2;
        return 0;
    }
    public static double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
}
