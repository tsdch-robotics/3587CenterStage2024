package org.firstinspires.ftc.teamcode.TeleOp.ScoringFunctions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TeleOpScoringFunctions {


      AnnaPIDslides pidController = new AnnaPIDslides();



      //write your pos here, for example:\

        int slideMinPos = 770;
        int slideMidPos= 2000;
        int slideMedPos= 2200;
        int slideMaxPos = 3400;
        int slideZeroPos = 0;


    public enum robotMachineState{

        SLIDE_MIN,
        SLIDE_MID,
        SLIDE_MED,
        SLIDE_MAX,
        SLIDE_ZERO,

        SLIDE_SCORE,

        DOOR_OPEN,

        DOOR_CLOSE,







    }


    public void doThisThingy(DcMotor slides, Servo larm, Servo rarm, Servo door, ElapsedTime PIDtime, ElapsedTime waitingTime, robotMachineState targetMachineState){


        switch (targetMachineState){

            case SLIDE_MAX:

                pidController.magicPID(slides, slideMaxPos, PIDtime);

                if(waitingTime.milliseconds() >= 1300){
                    larm.setPosition(0.7);
                    rarm.setPosition(0.7);

                }


                break;

            case SLIDE_MID:

                pidController.magicPID(slides, slideMidPos, PIDtime);

                if(waitingTime.milliseconds() >= 1300){
                    larm.setPosition(0.7);
                    rarm.setPosition(0.7);

                }


                break;

            case SLIDE_MED:

                pidController.magicPID(slides, slideMedPos, PIDtime);

                if(waitingTime.milliseconds() >= 1300){
                    larm.setPosition(0.7);
                    rarm.setPosition(0.7);

                }


                break;

            case SLIDE_MIN:

                pidController.magicPID(slides, slideMinPos, PIDtime);

                if(waitingTime.milliseconds() >= 1300){
                    larm.setPosition(0.7);
                    rarm.setPosition(0.7);

                }

                break;
            case SLIDE_ZERO:
                    larm.setPosition(0.0);
                    rarm.setPosition(0.0);
                    door.setPosition(0.0);

                if(waitingTime.milliseconds() >= 1300) {

                    pidController.magicPID(slides, slideZeroPos, PIDtime);

                }



                break;


            case DOOR_CLOSE:

                door.setPosition(0.0);
                door.setPosition(0.0);

                break;


            case DOOR_OPEN:

                door.setPosition(1.0);
                door.setPosition(1.0);

                break;

        }


    }

}
