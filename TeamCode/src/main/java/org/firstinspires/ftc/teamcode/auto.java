package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Autonomous(name="auto", group="Robot")
public class auto extends LinearOpMode {
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor backLeft;

    private DcMotor inmotor;

    private DcMotor slide;

    private Servo kickServo;

    private DcMotor upmotor;

    private Servo plane;
    private final static int LED_PERIOD = 10;

    /*
     * Rate limit gamepad button presses to every 500ms.
     */
    protected enum DisplayKind {
        MANUAL,
        AUTO
    }

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    Telemetry.Item patternName;
    Telemetry.Item display;

    private ColorSensor cSensor;

    double r, g, b;
    int total;
    double  bootPosition ;
    double  MIN_POSITION = 0, MAX_POSITION = 1;

    double slidePosition;


    public void runOpMode() throws InterruptedException {
        // declare hardware
        //declare drive motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        backRight = hardwareMap.get(DcMotor.class, "backRight");

        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontRight .setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight .setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Starting at",  "%7d :%7d",
                frontLeft.getCurrentPosition(),
                frontRight.getCurrentPosition(),
                backLeft.getCurrentPosition(),
                backRight.getCurrentPosition());
        telemetry.update();


        inmotor = hardwareMap.get(DcMotor.class, "inmotor");

        inmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        inmotor.setDirection(DcMotor.Direction.FORWARD);
        inmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //declare servos
        kickServo = hardwareMap.get(Servo.class, "kick_servo");
        plane = hardwareMap.get(Servo.class, "plane");
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkinLedDriver");


        //declare sensors
        cSensor = hardwareMap.get(ColorSensor.class, "cSensor");

        //declare mechanism motors
        slide = hardwareMap.get(DcMotor.class, "slide");

        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setDirection(DcMotor.Direction.FORWARD);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        upmotor = hardwareMap.get(DcMotor.class, "upmotor");

        upmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        upmotor.setDirection(DcMotor.Direction.FORWARD);
        upmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        waitForStart();
        //set outtake servo to open
        bootPosition = MIN_POSITION;
        while (opModeIsActive()) {
            r = cSensor.red();
            g = cSensor.green();
            b = cSensor.blue();
            total = cSensor.argb();
            pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
            blinkinLedDriver.setPattern(pattern);
            encoderDrive(1, 36,36,36,36, 0);


        }
    }

    public void drive(double forward, double strafe, double turn) {
        double multiplier = 1;
        double totalmax = (forward + turn + strafe);
        if ((forward - turn + strafe) > totalmax){
            totalmax = (forward - turn + strafe);
        }
        if ((forward - turn - strafe) > totalmax){
            totalmax = (forward - turn - strafe);
        }
        if ((forward + turn - strafe) > totalmax){
            totalmax = (forward + turn - strafe);
        }


        if (totalmax > 1){
            multiplier = 1/totalmax;
        }
        frontLeft.setPower((forward + turn + strafe)*multiplier);
        frontRight.setPower((forward - turn + strafe)*multiplier);
        backRight.setPower((forward - turn - strafe)* multiplier);
        backLeft.setPower((forward + turn - strafe)*multiplier);

    }

    public void colorCheck (double red, double green, double blue){
        // check for White, Black, yellow, Green, Purple
        if (red > 230 && green > 230 && blue > 230) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
        } else if (red < 60 && green < 60 && blue < 60){
            pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
        } else if (red > blue && green > blue) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
        }  else if (green > red && green > blue) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        } else if (blue > green && red > green) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
        } else {
            pattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_RED;
            // Non pixel colors output strobe red for error
        }
    }
    //outtake section

    public void outtake(double lift, boolean kick) {
        //kick and boot are for outtake servo
        if (kick && bootPosition < MAX_POSITION){
            bootPosition = MAX_POSITION;


        }
        if (bootPosition == MAX_POSITION && !kick){
            bootPosition = MIN_POSITION;
        }
        //lift and slide is for linear slide

    }
    public void intake (double in) {
        //kick and boot are for outtake servo
        frontLeft.setPower((in));



    }
    public void encoderDrive(double speed,
                             double frontleftInches, double frontrightInches, double backleftInches, double backrightInches,
                             double timeoutS) {
        int newfrontLeftTarget;
        int newfrontRightTarget;
        int newbackLeftTarget;
        int newbackRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newfrontLeftTarget = frontLeft.getCurrentPosition() + (int)(frontleftInches * COUNTS_PER_INCH);
            newfrontRightTarget = frontRight.getCurrentPosition() + (int)(frontrightInches * COUNTS_PER_INCH);
            frontRight.setTargetPosition(newfrontLeftTarget);
            frontLeft.setTargetPosition(newfrontRightTarget);
            newbackLeftTarget = backLeft.getCurrentPosition() + (int)(backleftInches * COUNTS_PER_INCH);
            newbackRightTarget = backRight.getCurrentPosition() + (int)(backrightInches * COUNTS_PER_INCH);
            backRight.setTargetPosition(newfrontLeftTarget);
            backLeft.setTargetPosition(newfrontRightTarget);

            // Turn On RUN_TO_POSITION
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontRight.setPower(Math.abs(speed));
            frontLeft.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newfrontLeftTarget,  newfrontRightTarget, newbackLeftTarget, newbackRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(), backLeft.getCurrentPosition(),  backRight.getCurrentPosition());
                telemetry.update();

            }

            // Stop all motion;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

}
