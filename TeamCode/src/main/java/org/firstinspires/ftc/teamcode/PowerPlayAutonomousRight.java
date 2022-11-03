package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.EasyOpenCVExamples.SkystoneDeterminationExample;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import com.qualcomm.robotcore.util.Range;

import java.util.Locale;

@Autonomous(name = "Right Autonomous PP", group = "Autonomous")
public class PowerPlayAutonomousRight extends LinearOpMode
{

    OpenCvWebcam webcam;
    Hardwarerobot robot = new Hardwarerobot();
    private final TherePipeline pipeline = new TherePipeline();
    TherePipeline.PowerPlayPosition getAnalysis = TherePipeline.PowerPlayPosition.LEFT;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4.5 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415)/2.29; //find out actual number
    static final double     STRAFE_COUNTS_PER_INCH  = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415)/1.86;
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    static final double     HEADING_THRESHOLD       = .1 ;
    static final double     P_TURN_COEFF            = 0.1;
    static final double     P_DRIVE_COEFF           = 0.05;
    static double ARM_COUNTS_PER_INCH = 114.75; //Figure out right number
    int newTarget=0;
    static double CLAW_CLOSED_POSITION = .28; // flip closed and open
    static double CLAW_OPENED_POSITION = .08;


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        // int urMom;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        composeTelemetry();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity  = imu.getGravity();
        robot.armExtendor.setTargetPosition(newTarget);
        robot.armExtendor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armExtendor.setPower(0);
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        while (opModeInInit()) {
            telemetry.addData("Realtime analysis", pipeline.getAnalysis());
            telemetry.update();
            sleep(50);
             
        }
        robot.armExtendor.setPower(1);
        telemetry.addData("Snapshot post-START analysis", getAnalysis);
        telemetry.update();
        getAnalysis = TherePipeline.PowerPlayPosition.LEFT;
        switch (getAnalysis)
        {
            case LEFT:
            {
                //F5
                closeClaw();
                sleep(500);
                gyroDrive(.4,24,0);
                sleep(1000);
                gyroTurn(.4,90);
                gyroDrive(.4,20,90);
                gyroTurn(.4,45);
                sleep(500);
                goTo3();
                sleep(3000);
                gyroDrive(.4,6,45);
                sleep(500);
                goTo4();
                sleep(500);
                openClaw();
                sleep(500);
                gyroDrive(.4,-6,45);
                goTo0();
                sleep(3000);
                closeClaw();
                gyroTurn(.4,0);
                break;
            }

            case CENTER: {
                closeClaw();
                sleep(500);
                gyroDrive(.4,24,0);
                sleep(1000);
                gyroTurn(.4,90);
                gyroDrive(.4,20,90);
                gyroTurn(.4,45);
                sleep(500);
                goTo3();
                sleep(3000);
                gyroDrive(.4,6,45);
                sleep(500);
                goTo4();
                sleep(500);
                openClaw();
                sleep(500);
                gyroDrive(.4,-6,45);
                goTo0();
                sleep(3000);
                closeClaw();
                gyroTurn(.4,0);
                gyroStrafe(.4,-20,0);
                break;
            }

            case RIGHT: {
                closeClaw();
                sleep(500);
                gyroDrive(.4,24,0);
                sleep(1000);
                gyroTurn(.4,90);
                gyroDrive(.4,20,90);
                gyroTurn(.4,45);
                sleep(500);
                goTo3();
                sleep(3000);
                gyroDrive(.4,6,45);
                goTo4();
                sleep(500);
                sleep(500);
                openClaw();
                sleep(500);
                gyroDrive(.4,-6,45);
                goTo0();
                sleep(3000);
                closeClaw();
                gyroTurn(.4,0);
                gyroStrafe(.4,-44,0);
                break;
            }

        }
    }
    public void encoderDrive(double speed,
                             double distance,
                             double timeoutS) {

        encoderDrive(speed, distance, distance,timeoutS);
    }
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        if (opModeIsActive()) {

            newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.leftBackDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBackDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            robot.leftBackDrive.setTargetPosition(newLeftBackTarget);
            robot.rightFrontDrive.setTargetPosition(newRightFrontTarget);
            robot.rightBackDrive.setTargetPosition(newRightBackTarget);

            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.leftFrontDrive.setPower(Math.abs(speed));
            robot.leftBackDrive.setPower(Math.abs(speed));
            robot.rightFrontDrive.setPower(Math.abs(speed));
            robot.rightBackDrive.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFrontDrive.isBusy() && robot.rightFrontDrive.isBusy() && (robot.leftBackDrive.isBusy() && robot.rightBackDrive.isBusy()))) {

                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget, newLeftBackTarget, newRightFrontTarget, newRightBackTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftFrontDrive.getCurrentPosition(),
                        robot.leftBackDrive.getCurrentPosition(),
                        robot.rightFrontDrive.getCurrentPosition(),
                        robot.rightBackDrive.getCurrentPosition());
                telemetry.update();
            }
            robot.leftFrontDrive.setPower(0);
            robot.leftBackDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);
            robot.rightBackDrive.setPower(0);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftFrontTarget;
        int     newLeftBackTarget;
        int     newRightFrontTarget;
        int     newRightBackTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        if (opModeIsActive()) {

            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() + moveCounts;
            newLeftBackTarget = robot.leftBackDrive.getCurrentPosition() + moveCounts;
            newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() + moveCounts;
            newRightBackTarget = robot.rightBackDrive.getCurrentPosition() + moveCounts;
            robot.leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            robot.leftBackDrive.setTargetPosition(newLeftBackTarget);
            robot.rightFrontDrive.setTargetPosition(newRightFrontTarget);
            robot.rightBackDrive.setTargetPosition(newRightBackTarget);

            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftFrontDrive.setPower(speed);
            robot.leftBackDrive.setPower(speed);
            robot.rightFrontDrive.setPower(speed);
            robot.rightBackDrive.setPower(speed);

            while (opModeIsActive() &&
                    (robot.leftFrontDrive.isBusy() && robot.leftBackDrive.isBusy() && robot.rightFrontDrive.isBusy() && robot.rightBackDrive.isBusy())) {

                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.leftFrontDrive.setPower(leftSpeed);
                robot.leftBackDrive.setPower(leftSpeed);
                robot.rightFrontDrive.setPower(rightSpeed);
                robot.rightBackDrive.setPower(rightSpeed);

                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }
            robot.leftFrontDrive.setPower(0);
            robot.leftBackDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);
            robot.rightBackDrive.setPower(0);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void gyroTurn (  double speed, double angle) {

        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            telemetry.update();
        }
    }
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {

            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }
        robot.leftFrontDrive.setPower(0);
        robot.leftBackDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
    }

    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }
        robot.leftFrontDrive.setPower(leftSpeed);
        robot.leftBackDrive.setPower(leftSpeed);
        robot.rightFrontDrive.setPower(rightSpeed);
        robot.rightBackDrive.setPower(rightSpeed);

        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }
    public double getError(double targetAngle) {

        double robotError;
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity  = imu.getGravity();
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
    void composeTelemetry() {

        telemetry.addAction(new Runnable() { @Override public void run()
        {

            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });
        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });
        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });
        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    public void gyroStrafe ( double speed,
                             double distance,
                             double angle) {

        int     newLeftFrontTarget;
        int     newLeftBackTarget;
        int     newRightFrontTarget;
        int     newRightBackTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        if (opModeIsActive()) {

            moveCounts = (int)(distance * STRAFE_COUNTS_PER_INCH);
            newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() + moveCounts;
            newLeftBackTarget = robot.leftBackDrive.getCurrentPosition() - moveCounts;
            newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() - moveCounts;
            newRightBackTarget = robot.rightBackDrive.getCurrentPosition() + moveCounts;
            robot.leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            robot.leftBackDrive.setTargetPosition(newLeftBackTarget);
            robot.rightFrontDrive.setTargetPosition(newRightFrontTarget);
            robot.rightBackDrive.setTargetPosition(newRightBackTarget);

            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftFrontDrive.setPower(speed);
            robot.leftBackDrive.setPower(speed);
            robot.rightFrontDrive.setPower(speed);
            robot.rightBackDrive.setPower(speed);

            while (opModeIsActive() &&
                    (robot.leftFrontDrive.isBusy() && robot.leftBackDrive.isBusy() && robot.rightFrontDrive.isBusy() && robot.rightBackDrive.isBusy())) {

                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.leftFrontDrive.setPower(leftSpeed);
                robot.leftBackDrive.setPower(leftSpeed);
                robot.rightFrontDrive.setPower(rightSpeed);
                robot.rightBackDrive.setPower(rightSpeed);

                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }
            robot.leftFrontDrive.setPower(0);
            robot.leftBackDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);
            robot.rightBackDrive.setPower(0);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void goTo0(){
        double distance = 0;
        newTarget = (int)(distance * ARM_COUNTS_PER_INCH);
        robot.armExtendor.setTargetPosition(newTarget);
    }
    public void goTo1() {
        double distance = 16;
        newTarget = (int) (distance * ARM_COUNTS_PER_INCH);
        robot.armExtendor.setTargetPosition(newTarget);
    }
    public void goTo2() {
        double distance = 26;
        newTarget = (int) (distance * ARM_COUNTS_PER_INCH);
        robot.armExtendor.setTargetPosition(newTarget);
    }
    public void goTo3() {
        double distance = 35.5;
        newTarget = (int) (distance * ARM_COUNTS_PER_INCH);
        robot.armExtendor.setTargetPosition(newTarget);
    }
    public void goTo4() {
        double distance = 33;
        newTarget = (int) (distance * ARM_COUNTS_PER_INCH);
        robot.armExtendor.setTargetPosition(newTarget);
    }

    public void openClaw() {
        robot.claw.setPosition(CLAW_OPENED_POSITION);
    }
    public void closeClaw() {
        robot.claw.setPosition(CLAW_CLOSED_POSITION);
    }







}
