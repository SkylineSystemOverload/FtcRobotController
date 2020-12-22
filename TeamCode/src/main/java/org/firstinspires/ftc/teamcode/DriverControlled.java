package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="DriverControlled", group="Test")

public class DriverControlled extends OpMode{

    // DEFINE robot
    RobotHardware robot = new RobotHardware();

    // CONSTANTS

    // RUN ONCE ON init()
    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("STATUS", "Initialized");
    }

    //LOOP ON init()
    @Override
    public void init_loop() {
    }

    //RUN ONCE ON start()
    @Override
    public void start() {
    }

    //LOOP ON start()
    @Override
    public void loop() {
        double G1rightStickY = -gamepad1.right_stick_y;
        double G1leftStickY = -gamepad1.left_stick_y;
        double G1rightStickX = gamepad1.right_stick_x;
        double G1leftStickX = gamepad1.left_stick_x;
        boolean G1a = gamepad1.a;
        boolean G1y = gamepad1.y;
        boolean G1x = gamepad1.x;
        boolean G1b = gamepad1.b;
        double G1LT = gamepad1.left_trigger;
        double G1RT = gamepad1.right_trigger;
        boolean G1dpad_right = gamepad1.dpad_right;
        boolean G1dpad_left = gamepad1.dpad_left;
        boolean G1dpad_up = gamepad1.dpad_up;
        boolean G1dpad_down = gamepad1.dpad_down;
        boolean G1rightBumper = gamepad1.right_bumper;
        boolean G1leftBumper = gamepad1.left_bumper;

        //mecanum drive
        robot.motor2.setPower((G1leftStickY) - (G1rightStickX) - (G1leftStickX));
        robot.motor4.setPower((G1leftStickY) - (G1rightStickX) + (G1leftStickX));
        robot.motor1.setPower((G1leftStickY) + (G1rightStickX) + (G1leftStickX));
        robot.motor3.setPower((G1leftStickY) + (G1rightStickX) - (G1leftStickX));




        telemetry.addData("motor1 Power", robot.motor1.getPower());
        telemetry.addData("motor2 Power", robot.motor2.getPower());
        telemetry.addData("motor3 Power", robot.motor3.getPower());
        telemetry.addData("motor4 Power", robot.motor4.getPower());
    }

    // RUN ONCE ON stop()
    @Override
    public void stop() {
    }
}