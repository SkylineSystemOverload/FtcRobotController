package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@Autonomous(name="ShooterTest", group="Test")
public class ShooterTest extends LinearOpMode {

    /* Declare OpMode members. */
    RobotHardware robot = new RobotHardware();
    private ElapsedTime     runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            /*
             * Initialize the drive system variables.
             * The init() method of the hardware class does all the work here
             */
            robot.init(hardwareMap);

            // Send telemetry message to signify robot waiting;
            //telemetry.addData("Status", "Resetting Encoders");    //
            //telemetry.update();

            //robot.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //robot.motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Send telemetry message to indicate successful Encoder reset
            //telemetry.addData("Path0",  "Starting at %7d :%7d",
            //robot.motor1.getCurrentPosition());
            //telemetry.update();

            robot.motor1.setPower(1);
        }
    }
}
