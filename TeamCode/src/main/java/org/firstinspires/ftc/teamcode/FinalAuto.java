package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@Autonomous(name = "it's the final auto tututududu", group = "Pinpoint")
//@Disabled

public class FinalAuto extends LinearOpMode {

    DcMotor leftFrontDrive;
    DcMotor rightFrontDrive;
    DcMotor leftBackDrive;
    DcMotor rightBackDrive;
    DcMotor elevatorMotor;
    CRServo servoBucket;
    CRServo servoPinceR;
    CRServo servoPinceL;

    private final ElapsedTime runtime = new ElapsedTime();
    double timeBucket = 0.0;
    double timeArm = 0.0;
    double timeClaw = 0.0;
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class
    boolean bucketOut = false;
    int newTarget = 0;
    boolean readyToGo = true;
    boolean armOut = false;
    boolean clawOut = false;

    enum StateMachine {
        WAITING_FOR_START,
        AT_TARGET,
        DRIVE_TO_TARGET_1,
        DRIVE_TO_TARGET_2,
        DRIVE_TO_TARGET_3,
        DRIVE_TO_TARGET_4,
        DRIVE_TO_TARGET_5
    }

    static final Pose2D TARGET_1 = new Pose2D(DistanceUnit.MM, 200, 0, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_2 = new Pose2D(DistanceUnit.MM, 120, 850, AngleUnit.DEGREES, -45);
    static final Pose2D TARGET_3 = new Pose2D(DistanceUnit.MM, 400, 600, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_4 = new Pose2D(DistanceUnit.MM, 120, 850, AngleUnit.DEGREES, -45);
    static final Pose2D TARGET_5 = new Pose2D(DistanceUnit.MM, 120, 850, AngleUnit.DEGREES, -45);


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        double elevatorPower = 0.0;
        leftFrontDrive = hardwareMap.dcMotor.get("frontLeft");
        rightFrontDrive = hardwareMap.dcMotor.get("frontRight");
        leftBackDrive = hardwareMap.dcMotor.get("backLeft");
        rightBackDrive = hardwareMap.dcMotor.get("backRight");
        elevatorMotor = hardwareMap.dcMotor.get("elevator");
        servoBucket = hardwareMap.get(CRServo.class, "servoBucket");
        DigitalChannel elevatorIn = hardwareMap.digitalChannel.get("elevatorIn");
        DigitalChannel elevatorOut = hardwareMap.digitalChannel.get("elevatorOut");
        DcMotor armMotor = hardwareMap.dcMotor.get("arm");
        servoPinceR = hardwareMap.get(CRServo.class, "servoPinceR");
        servoPinceL = hardwareMap.get(CRServo.class, "servoPinceL");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(0.0, 0.0); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();

        //nav.setXYCoefficients(0.02,0.002,0.0,DistanceUnit.MM,12);
        //nav.setYawCoefficients(1,0,0.0, AngleUnit.DEGREES,2);
        nav.setDriveType(DriveToPoint.DriveType.MECANUM);

        StateMachine stateMachine;
        stateMachine = StateMachine.WAITING_FOR_START;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            odo.update();

            switch (stateMachine) {
                case WAITING_FOR_START:
                    //the first step in the autonomous
                    stateMachine = StateMachine.DRIVE_TO_TARGET_1;
                    break;
                case DRIVE_TO_TARGET_1:
                    if (nav.driveTo(odo.getPosition(), TARGET_1, 0.5, 0)) {
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_2;
                        elevatorPower = 0.75;

                    }


                    break;
                case DRIVE_TO_TARGET_2:
                    //drive to the second target
                    if (nav.driveTo(odo.getPosition(), TARGET_2, 0.5, 0.5)) {
                        telemetry.addLine("at position #2!");

                        servoBucket.setPower(-1.0);
                        bucketOut = true;
                        timeBucket = runtime.time();
                        stateMachine = StateMachine.AT_TARGET;
                        readyToGo = false;
                        newTarget = 3;

                    }

                    break;
                case DRIVE_TO_TARGET_3:
                    if (nav.driveTo(odo.getPosition(), TARGET_3, 0.5, 0.0)) {
                        telemetry.addLine("at position #3");
                        stateMachine = StateMachine.AT_TARGET;
                        armMotor.setPower(-1.0);
                        armOut = true;
                        timeArm = runtime.time();
                        readyToGo = false;
                        newTarget = 4;

                    }

                    break;
                case DRIVE_TO_TARGET_4:
                    if (nav.driveTo(odo.getPosition(), TARGET_4, 0.80, 0.5)) {
                        telemetry.addLine("at position #4");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_5;

                    }

                    break;
                case DRIVE_TO_TARGET_5:
                    if (nav.driveTo(odo.getPosition(), TARGET_5, 0.6, 0.5)) {
                        telemetry.addLine("There!");
                        stateMachine = StateMachine.AT_TARGET;
                        newTarget = 0;

                    }

                    break;
            }
            elevatorMotor.setPower(ElevatorFunction.moveElevator(elevatorPower, elevatorIn, elevatorOut));
            if (timeBucket != 0.0) {
                if (timeBucket + 2.0 <= runtime.time()) {
                    servoBucket.setPower(0);
                    timeBucket = 0.0;
                    if (bucketOut) {
                        readyToGo = true;
                        bucketOut = false;
                        timeBucket = runtime.time();
                        servoBucket.setPower(1.0);
                        elevatorPower = -0.75;
                    }
                }
            }
            if (timeArm != 0.0) {
                if (timeArm + 1.0 <= runtime.time()) {
                    armMotor.setPower(0);
                    timeArm = 0.0;
                    if (armOut && !clawOut) {
                        timeClaw = runtime.time();
                        clawOut = true;
                        servoPinceR.setPower(-1);
                        sleep(5);
                        servoPinceL.setPower(1);
                        sleep(5);
                    }
                    if (armOut && clawOut) {
                        timeClaw = runtime.time();
                        clawOut = false;
                        servoPinceR.setPower(1);
                        sleep(5);
                        servoPinceL.setPower(-1);
                        sleep(5);
                    }

                }
            }
            if (timeClaw != 0.0) {
                if (timeClaw + 2.0 <= runtime.time()) {
                    servoPinceR.setPower(0);
                    servoPinceL.setPower(0);
                    timeClaw = 0.0;
                    if (!armOut && clawOut) {
                        clawOut = false;
                        timeClaw = runtime.time();
                        servoPinceR.setPower(1);
                        servoPinceL.setPower(-1);
                        readyToGo = true;

                    } else if (!armOut && !clawOut) {
                        readyToGo = true;
                    } else if (clawOut && armOut) {
                        armOut = false;
                        timeArm = runtime.time();
                        armMotor.setPower(1);
                    }
                }
            }
            if (stateMachine == StateMachine.AT_TARGET && readyToGo) {
                switch (newTarget) {
                    case 3:
                        stateMachine = StateMachine.DRIVE_TO_TARGET_3;


                        break;
                    case 4:
                        stateMachine = StateMachine.DRIVE_TO_TARGET_4;

                        break;
                    case 5:
                        stateMachine = StateMachine.DRIVE_TO_TARGET_5;

                        break;
                }

            }


            //nav calculates the power to set to each motor in a mecanum or tank drive. Use nav.getMotorPower to find that value.
            leftFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
            rightFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
            leftBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
            rightBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));

            telemetry.addData("current state:", stateMachine);

            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            telemetry.update();

        }
    }
}
