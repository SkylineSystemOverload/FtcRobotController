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

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
@Disabled
@Autonomous(name="ShooterTest", group="Test")
public class ShooterTest extends LinearOpMode
{
    //Calls the RobotHardware class
    RobotHardware robot = new RobotHardware();

    //This is what happens when the init button is pushed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        // Initializes hardware when init is pressed on the phone
        robot.init(hardwareMap);

        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        // The program will wait for the start button to continue.
        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);

        //Proceeds with the following code as long as the mode is active (returns false when stop button is pushed or power is disconnected).
        //The difference between (opModeIsActive()) and (isStopRequested()) is the first requires the play (not init) button to be pushed
        //the latter does not (this is just my guess).
        while (opModeIsActive())
        {
            //The series of instructions the robot will do.
            robot.motor7.setPower(1);
            telemetry.addData("Power", robot.motor7.getPower());
            telemetry.addData("EncoderCounts", robot.motor7.getCurrentPosition());
        }
        //Turn the motors off (this will happen once when "While opModeIsActive" loop is finished).
    }
}