// Simple autonomous program that uses an IMU to drive in a straight line.
// Uses REV Hub's built in IMU in place of a gyro.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
@Disabled
@Autonomous(name="BlueWobble", group="Test")
public class BlueWobble extends LinearOpMode
{
    //Calls the RobotHardware class
    RobotHardware robot = new RobotHardware();
    private ElapsedTime runtime = new ElapsedTime();

    BNO055IMU             imu;
    Orientation           lastAngles = new Orientation();
    double                globalAngle, power = .60, correction, rotation;
    double newPower;
    double targetPower = power; //power, as defined earlier in the code, will now be named targetPower
    double currentPower;
    boolean goingDown = false;
    boolean goingUp = false;
    final int wobbleTravelDistance = 440; // literally just a guess
    final int encoderTolerance = 5;
    boolean BoxOne = false;
    boolean BoxTwo = false;
    boolean BoxThree = false;
    boolean instruction1 = true;
    boolean instruction2 = false;
    boolean instruction3 = false;
    boolean instruction4 = false;
    boolean instruction5 = false;
    boolean instruction6 = false;
    boolean instruction7 = false;
    boolean instruction8 = false;

    //Calls the PIDHardware class
    PIDHardware           pidRotate, pidDrive;

    // instatiates the autonomous instruction handler class
    AutonomousInstructions Instructions = new AutonomousInstructions();
    OpenCvInternalCamera phoneCam;
    EasyOpenCVExample.SkystoneDeterminationPipeline pipeline;

    //Declares some methods to compress and reduce tediousness of writing repetitive code.
    //Every time the method DriveForward() is called,  it will do the instructions within the method
    public void DriveForward()
    {
        robot.motor1.setPower(power - correction); //sets the motors power
        robot.motor2.setPower(power + correction);
        robot.motor3.setPower(power - correction);
        robot.motor4.setPower(power + correction);
    }
    //Same as DriveForward() but in reverse
    public void DriveBackward()
    {
        robot.motor1.setPower(-power - correction);
        robot.motor2.setPower(-power + correction);
        robot.motor3.setPower(-power - correction);
        robot.motor4.setPower(-power + correction);
    }
    public void StrafeLeft()
    {
        robot.motor1.setPower(-power - correction);
        robot.motor2.setPower(power + correction);
        robot.motor3.setPower(power - correction);
        robot.motor4.setPower(-power + correction);
    }
    public void StrafeRight()
    {
        robot.motor1.setPower(power - correction);
        robot.motor2.setPower(-power + correction);
        robot.motor3.setPower(-power - correction);
        robot.motor4.setPower(power + correction);
    }
    public void TurnLeft()
    {
        robot.motor1.setPower(-power - correction);
        robot.motor2.setPower(power + correction);
        robot.motor3.setPower(-power - correction);
        robot.motor4.setPower(power + correction);
    }
    public void TurnRight()
    {
        robot.motor1.setPower(power - correction);
        robot.motor2.setPower(-power + correction);
        robot.motor3.setPower(power - correction);
        robot.motor4.setPower(-power + correction);
    }
    //Stops all 4 motors
    public void StopDriving()
    {
        robot.motor1.setPower(0);
        robot.motor2.setPower(0);
        robot.motor3.setPower(0);
        robot.motor4.setPower(0);
    }

    public void CalculatePower()
    {
        if (targetPower > currentPower) {
            newPower = currentPower + 3*Math.pow(runtime.seconds(),2); //The main equation: adds a power-curve to whatever the current power was
        }
        else if (targetPower < currentPower) {
            newPower = currentPower + -3*Math.pow(runtime.seconds(),2); //The main equation: adds a power-curve to whatever the current power was
        }
        else {
            newPower = targetPower;
        }

        if (newPower > targetPower && targetPower > currentPower) {
            newPower = targetPower; //caps the power added by the graph to the power we set
        }

        if (newPower < targetPower && targetPower < currentPower) {
            newPower = targetPower; //caps the power added by the graph to the power we set
        }
    }

    public void DropWobble()
    {
        // reset encoder and set to move to the distance at .3 speed
        robot.motor6.setTargetPosition(wobbleTravelDistance);
        robot.motor6.setPower(.3);

        // open finger when going down
        robot.servo4.setPosition(0);
        if (goingDown && Math.abs(robot.motor6.getCurrentPosition() - wobbleTravelDistance) > encoderTolerance) {
            goingDown = false;
        }
        else if (goingUp && Math.abs(robot.motor6.getCurrentPosition()) > encoderTolerance) {
            goingUp = false;
        }
    }

    //This is what happens when the init button is pushed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        // Initializes hardware when init is pressed on the phone
        robot.init(hardwareMap);

        //Makes new methods for naming simplification purposes
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        pidRotate = new PIDHardware(.003, .00003, 0);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidDrive = new PIDHardware(.05, 0, 0);

        //Gives info about what the IMU is doing on the phone
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // When the stop button isn't pushed and the gyro (IMU) isn't calibrated, wait (! means not). This is a loop.
        while (!isStopRequested() && !robot.imu.isGyroCalibrated())
        {
            //do nothing for 50 milliseconds
            sleep(50);
            //idle(); allows the program to perform other necessary tasks in between iterations of the loop.
            idle();
        }
        //Once the past loop finishes and the IMU is calibrated, the rest of the code continues.
        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", robot.imu.getCalibrationStatus().toString());
        telemetry.update();

        // The program will wait for the start button to continue.
        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);

        // Set up parameters for driving in a straight line.
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        // camera code ---------------------------------------------------------
        final int CAMERA_WIDTH = 320;
        final int CAMERA_HEIGHT = 240;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new EasyOpenCVExample.SkystoneDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(CAMERA_WIDTH,CAMERA_HEIGHT, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

        //captures System.currentTimeMillis and saves it as startTime. Subtract the later time from this time to get the change in time.
        boolean started = false;
        long startTime = 0;
        long elapsedTime;

        //Proceeds with the following code as long as the mode is active (returns false when stop button is pushed or power is disconnected).
        //The difference between (opModeIsActive()) and (isStopRequested()) is the first requires the play (not init) button to be pushed
        //the latter does not (this is just my guess).
        while (opModeIsActive())
        {
            // simple switch that sets the time as soon as op mode is started (activates only once and at the beginning of the loop)
            if (!started) {
                sleep(500);
                if (pipeline.position == EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.NONE) {
                    BoxOne = true;
                }
                else if (pipeline.position == EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.ONE) {
                    BoxTwo = true;
                }
                else if (pipeline.position == EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.FOUR){
                    BoxThree = true;
                }
                started = true;
                startTime = System.currentTimeMillis();
            }

            elapsedTime = System.currentTimeMillis() - startTime;

            // Use PID with imu input to drive in a straight line.
            //Gets the correction value based on our current angle
            correction = pidDrive.performPID(getAngle());

            //Displays the realtime heading information on the phone.
            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.addData("4 turn rotation", rotation);
            telemetry.addData("Rings", pipeline.position);
            telemetry.update();

            //The series of instructions the robot will do.
            //if we need to go to the first box
            if (BoxOne = true) {
                //Strafe Left
                if (instruction1 && runtime.milliseconds()>0) {
                    StrafeLeft();
                    instruction2 = true;
                }
                //Stop Driving
                else if (instruction2 && runtime.milliseconds()>500){
                    instruction1 = false;
                    StopDriving();
                    runtime.reset();
                    sleep(200);
                    instruction3 = true;
                    instruction2 = false;
                }
                //Drive Forward
                else if (instruction3 && runtime.milliseconds()>0){
                    DriveForward();
                    instruction4 = true;
                }
                //Stop Driving
                else if (instruction4 && runtime.milliseconds()>3000){
                    instruction3 = false;
                    StopDriving();
                    runtime.reset();
                    sleep(200);
                    instruction5 = true;
                    instruction4 = false;
                }
                //Drop Wobble Goal
                else if (instruction5 && runtime.milliseconds()>0){
                    DropWobble();
                    instruction6 = true;
                }
                //Stop Driving
                else if (instruction6 && runtime.milliseconds()>2000){
                    instruction5 = false;
                    robot.motor6.setPower(0);
                    runtime.reset();
                    sleep(200);
                    instruction7 = true;
                    instruction6 = false;
                }
                //Drive Backward
                else if (instruction7 && runtime.milliseconds()>0){
                    DriveBackward();
                }
                //Stop Driving
                else if (instruction8 && runtime.milliseconds()>500){
                    instruction7 = false;
                    StopDriving();
                    runtime.reset();
                    sleep(200);
                    instruction8 = false;
                }
            }
            else if (BoxTwo = true) {
                DriveForward();
            }
            else if (BoxThree = true){
                StrafeRight();
            }
        }
        //Turn the motors off (this will happen once when "While opModeIsActive" loop is finished).
        StopDriving();
        robot.motor5.setPower(0);
        robot.motor7.setPower(0);
    }

    // Resets the cumulative angle tracking to zero.
    private void resetAngle() {
        //ZYX
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    //Get current cumulative angle rotation from last reset.
    //return Angle in degrees. + = left, - = right.
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //The change (delta) angle is whatever the angle was first minus our current angle.
        //lastAngles is new Orientation() as declared near the beginning.
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        //If the angle goes below -180, start subtracting the change in the angle (so -180 is the greatest change in the angle, -360 would be 0).
        //Returns a positive value (left) if the robot turns more than 180 deg right.
        if (deltaAngle < -180)
            deltaAngle += 360;
            //If the angle goes above 180, start subtracting the change in the angle (so 180 is the greatest change in the angle, 360 would be 0).
            //Returns a negative value (right) if the robot turns more than 180 deg left.
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        //Add the deltaAngle to the globalAngle (which was set to 0 at the start, but will change as the robot moves).
        globalAngle += deltaAngle;

        //Sets the current lastAngles to angles, which is different from what lastAngles was to start.
        lastAngles = angles;

        return globalAngle;
    }


    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        // restart imu angle tracking.
        resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0)
            {
                TurnRight();
                sleep(100);
            }

            do
            {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                TurnLeft();
            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                TurnLeft();
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        StopDriving();

        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }
}