package org.firstinspires.ftc.teamcode.TeleOp.ScoringFunctions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TeleOpScoringFunctions {


      AnnaPIDslides pidController = new AnnaPIDslides();



      //write your pos here, for example:\



        int slideMinPos = 5000;
        int slideMidPos= 10000;
        int slideMaxPos = 15000;


    public enum robotMachineState{

        SLIDE_MIN,
        SLIDE_MID,

        SLIDE_MAX,

        SLIDE_SCORE,

        DOOR_OPEN,

        DOOR_CLOSE,







    }


    public void doThisThingy(DcMotor slides, Servo larm, Servo rarm, Servo door, ElapsedTime PIDtime, ElapsedTime waitingTime, robotMachineState targetMachineState){


        switch (targetMachineState){

            case SLIDE_MAX:


                //do this
                break;

            case SLIDE_MID:

                //do this
                break;

            case SLIDE_MIN:

                pidController.magicPID(slides, slideMinPos, PIDtime);

                larm.setPosition(0.75);
                rarm.setPosition(0.75);

                if(waitingTime.milliseconds() >= 200){
                    door.setPosition(1.0);
                }

                break;




        }


    }

}
