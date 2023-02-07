package org.firstinspires.ftc.teamcode;


import android.util.Log;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * CatHW_Jaws.java
 *
 *
 * This class containing common code accessing hardware specific to the movement of the jaws/intake.
 *
 * This is NOT an OpMode.  This class is used to define all the other hardware classes.
 * This hardware class assumes the following device names have been configured on the robot.
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back
 */
public class CatHW_Jaws extends CatHW_Subsystem
{

    // Motors: //
    //public CRServo intakeMotor = null;
    //public DcMotor intakeLift= null;
    public TouchSensor liftBottom = null;
    public DcMotor left_lift = null;
    public DcMotor right_lift = null;
    public DcMotor tilt = null;
    public Servo claw = null;

    public ColorSensor intakeColor = null;
    public DistanceSensor intakeDistance = null;
    public ElapsedTime liftTime = null;

    private int lastLiftEncoder = 0;

    // Timers: //

    /* Constructor */
    public CatHW_Jaws(CatHW_Async mainHardware) {
        super(mainHardware);

    }


    /* Initialize standard Hardware interfaces */
    public void init() {

        // Define and initialize motors: //


        left_lift = hwMap.dcMotor.get("left_lift");
        left_lift.setDirection(DcMotorSimple.Direction.REVERSE);
        left_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_lift.setTargetPosition(0);
        left_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        right_lift = hwMap.dcMotor.get("right_lift");
        right_lift.setDirection(DcMotorSimple.Direction.REVERSE);
        right_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_lift.setTargetPosition(0);
        right_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        tilt = hwMap.dcMotor.get("tilt");
        tilt.setDirection(DcMotorSimple.Direction.REVERSE);
        tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tilt.setTargetPosition(0);
        tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        claw = hwMap.servo.get("claw");

        liftBottom = hwMap.touchSensor.get("touch");
        intakeColor = hwMap.colorSensor.get("intake_color");

        intakeDistance = hwMap.get(DistanceSensor.class, "intake_color");

        liftTime = new ElapsedTime();
    }


    //----------------------------------------------------------------------------------------------
    // Jaw Methods:
    //----------------------------------------------------------------------------------------------

    /**
     * Set the power of intake motor.
     *
     * @param power at which the motors will spin.
     */
    public void setJawPower(double power) {
        // Max of 80% power for VEX 393 motors
        if (power > 0.8) {
            power = 0.8;
        }
        if (power < -0.8) {
            power = -0.8;
        }

        //intakeMotor.setPower(power);
    }

    //Lift mechanism

    // TOP 6189
    public void setLiftBottom(double power){
        left_lift.setTargetPosition(15);
        lastLiftEncoder = -100;
        left_lift.setPower(power);
        if(liftBottom.isPressed()){
            left_lift.setTargetPosition(left_lift.getCurrentPosition());

        }
    }
    public void setLiftGroundJunction(double power){
        right_lift.setTargetPosition(45);
        left_lift.setTargetPosition(45);
        lastLiftEncoder = -100;
        left_lift.setPower(power);
        right_lift.setPower(power);
    }
    public void setLiftLowPole(double power){
        left_lift.setTargetPosition(365);
        lastLiftEncoder = -100;
        left_lift.setPower(power);
    }
    public void setLiftMiddlePole(double power){
        left_lift.setTargetPosition(570);
        lastLiftEncoder = -100;
        left_lift.setPower(power);
    }
    public void setLiftHighPole(double power){
        left_lift.setTargetPosition(745);
        lastLiftEncoder = -100;
        left_lift.setPower(power);
    }

    public void setLiftHeight(int height, double power){
        left_lift.setTargetPosition(height);
        left_lift.setPower(power);

    }
    public void setTiltback(double power){
        tilt.setTargetPosition(185);
        tilt.setPower(power);
    }

    public void setTiltforward(double power){
        tilt.setTargetPosition(0);
        tilt.setPower(power);
    }


    public void bumpTilt(int bumpAmount) {
        if (bumpAmount > 0.5) {
            tilt.setTargetPosition(bumpAmount + tilt.getCurrentPosition());
            tilt.setPower(1);
        } else if (bumpAmount < -0.5) {
            tilt.setTargetPosition(bumpAmount + tilt.getCurrentPosition());
            tilt.setPower(.7);
        }
    }
    public void bumpLift(int bumpAmount) {
        if (bumpAmount > 0.5){
            left_lift.setTargetPosition(bumpAmount + left_lift.getCurrentPosition());
            left_lift.setPower(1);
        }else if(bumpAmount <-0.5){
            left_lift.setTargetPosition(bumpAmount + left_lift.getCurrentPosition());
            left_lift.setPower(.7);
            if(liftBottom.isPressed()){
                left_lift.setTargetPosition(left_lift.getCurrentPosition());

            }
        }
    }

    public void setLiftPower(double power){
        left_lift.setPower(power);
    }

    public void grabPos(){
        claw.setPosition(.3 );
    }
    public void unGrab()  { claw.setPosition(0); }


    //intake color sensor methods
    public boolean haveCone() {
        Log.d("catbot", String.format("Have Freight r/g/b/a %4d %4d %4d %4d",
                intakeColor.red(),intakeColor.green(),intakeColor.blue(),intakeColor.alpha()));

        if(intakeDistance.getDistance(DistanceUnit.INCH)< 1.2 ){
            return true;
        }
        return false;
    }

    public void waitForLift(){
        while (left_lift.isBusy()) {
            int liftPos = left_lift.getCurrentPosition();
            // return if the main hardware's opMode is no longer active.
            if (!(mainHW.opMode.opModeIsActive())) {
                return;
            }
            if(lastLiftEncoder == liftPos){
                return;
            }
            lastLiftEncoder = liftPos;
            mainHW.robotWait(0.05);
        }
    }






    //----------------------------------------------------------------------------------------------
    // isDone Method:
    //----------------------------------------------------------------------------------------------
    @Override
    public boolean isDone() {
        //Log.d("catbot", String.format(" intake power %.2f,", transferMotor.getPower()));
        // turn off lift when it's all the way down.
        if ( (left_lift.getTargetPosition() == 0) && (Math.abs(left_lift.getCurrentPosition()) < 50)) {
            left_lift.setPower(0);
        }

        return false;
    }
}