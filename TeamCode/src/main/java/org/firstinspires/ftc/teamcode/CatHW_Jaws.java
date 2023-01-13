package org.firstinspires.ftc.teamcode;


import android.util.Log;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
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
    public DcMotor lift = null;
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


        lift = hwMap.dcMotor.get("lift");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        tilt = hwMap.dcMotor.get("tilt");
        tilt.setDirection(DcMotorSimple.Direction.FORWARD);
        tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tilt.setTargetPosition(0);
        tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        claw = hwMap.servo.get("claw");

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
        lift.setTargetPosition(15);
        lastLiftEncoder = -100;
        lift.setPower(power);
    }
    public void setLiftGroundJunction(double power){
        lift.setTargetPosition(45);
        lastLiftEncoder = -100;
        lift.setPower(power);
    }
    public void setLiftLowPole(double power){
        lift.setTargetPosition(365);
        lastLiftEncoder = -100;
        lift.setPower(power);
    }
    public void setLiftMiddlePole(double power){
        lift.setTargetPosition(570);
        lastLiftEncoder = -100;
        lift.setPower(power);
    }
    public void setLiftHighPole(double power){
        lift.setTargetPosition(745);
        lastLiftEncoder = -100;
        lift.setPower(power);
    }
    public void setTilt90(double power){
        lift.setTargetPosition(0);
        lastLiftEncoder = -100;
        lift.setPower(power);
    }
    public void setLiftHeight(int height, double power){
        lift.setTargetPosition(height);
        lift.setPower(power);

    }
    public void setTiltPos(int height, double power){
        tilt.setTargetPosition(height);
        tilt.setPower(power);

    }

    public void bumpLift(int bumpAmount) {
        if (bumpAmount > 0.5){
            lift.setTargetPosition(bumpAmount + lift.getCurrentPosition());
            lift.setPower(1);
        }else if(bumpAmount <-0.5){
            lift.setTargetPosition(bumpAmount + lift.getCurrentPosition());
            lift.setPower(.7);
        }
    }

    public void bumpTilt(int bumpAmount) {
        if (bumpAmount > 0.5){
            tilt.setTargetPosition(bumpAmount + tilt.getCurrentPosition());
            tilt.setPower(1);
        }else if(bumpAmount <-0.5){
            tilt.setTargetPosition(bumpAmount + tilt.getCurrentPosition());
            tilt.setPower(1);
        }
    }

    public void setLiftPower(double power){
        lift.setPower(power);
    }

    //public void setDumpPos(double pos){
    //    dump.setPosition(pos);
    //}
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
        while (lift.isBusy()) {
            int liftPos = lift.getCurrentPosition();
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
        if ( (lift.getTargetPosition() == 0) && (Math.abs(lift.getCurrentPosition()) < 50)) {
            lift.setPower(0);
        }

        return false;
    }
}