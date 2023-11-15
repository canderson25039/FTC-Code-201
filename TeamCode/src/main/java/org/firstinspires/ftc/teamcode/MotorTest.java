package org.firstinspires.ftc.teamcode;

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

@TeleOp(name = "Motor Test")
public class MotorTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor backLeft;

    private DcMotor inmotor;
    private DcMotor upmotor;

    private DcMotor slide;


    private Servo kickServo;

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

    double planePosition;

    double planeMin =0, planeMax =1;
    double  MIN_POSITION = 0, MAX_POSITION = 1;

    double slidePosition;




    public void runOpMode() throws InterruptedException {
        // declare hardware
        //declare drive motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        backRight = hardwareMap.get(DcMotor.class, "backRight");

        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


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
            colorCheck(r,g,b);
            blinkinLedDriver.setPattern(pattern);
            drive(-gamepad1.left_stick_y,-gamepad1.left_stick_x, -gamepad1.right_stick_x, gamepad1.right_stick_button, gamepad1.left_stick_button);
            outtake(gamepad2.left_stick_x, gamepad2.left_bumper);
            intake (gamepad2.left_stick_y);
            endgame(gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.b);
            blinkinLedDriver.setPattern(pattern);
            /*
            controls:
            gamepad 1:
                leftstick forward and strafe
                rightstick turn
                press dwn on either stick to go fast

             gamepad 2:
                left stick to control intake speed
                rightstick for linear slide height
                leftbumper for outtake servo
                dpad up and down for lift arm
                b for shoot plane
             */


        }
    }

    public void drive(double forward, double strafe, double turn, boolean fast0, boolean fast1) {
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
        //push down on either stick to go faster
        if (!fast0 || !fast1){
            multiplier = multiplier *0.75;
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

    //endgame stuff
    public void endgame(boolean up, boolean dwn, boolean shoot){
        if (90 < runtime.seconds()) {
            while (up) {
                upmotor.setPower(1);
            }
            while (dwn) {
                upmotor.setPower(-1);
            }
            if (shoot) {
                planePosition = planeMax;
                sleep(5);
                planePosition = planeMin;
            }
        }
    }
    public void outtake(double lift, boolean kick) {
        //kick and boot are for outtake servo

        if (kick && bootPosition < MAX_POSITION){
            bootPosition = MAX_POSITION;


        }
        if (bootPosition == MAX_POSITION && !kick){
            bootPosition = MIN_POSITION;
        }

        frontLeft.setPower((lift));

        //lift and slide is for linear slide


    }
    public void intake (double in) {
        //kick and boot are for outtake servo
        frontLeft.setPower((in));



    }

}
