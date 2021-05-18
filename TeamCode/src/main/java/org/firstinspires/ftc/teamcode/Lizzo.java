package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

public class Lizzo {
    private ArrayList<GeneralInstruction> InstructionList = new ArrayList<>();





    private static class GeneralInstruction {}


    private static class Vroom  extends GeneralInstruction{
        private ArrayList<DcMotor> Motors;

        // parameters here are what we give the class when we first make it
        public Vroom(ArrayList<DcMotor> dcMotors) {
            this.Motors = dcMotors;
        }

        // method for doing vroom stuff
        private void MotorsGo() {
            Motors.get(0).setPower(1);
            Motors.get(1).setPower(1);
            Motors.get(2).setPower(1);
            Motors.get(3).setPower(1);
        }
    }

    private static class ServoControl extends GeneralInstruction{

    }


    public void AddVroom(ArrayList<DcMotor> DcMotors){
        InstructionList.add(new Vroom (DcMotors));
    }
}
