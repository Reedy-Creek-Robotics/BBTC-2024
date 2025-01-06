package org.firstinspires.ftc.teamcode;

public enum RunStates {
    PICKING(
            0,
            0,
            0,
            -1,
            0,
            true,
            0,
            false
    ),
    GRAB(
            0,
            0,
            0,
            -1,
            0,
            false,
            0,
            false
    ),
    HOLD(0,
            0,
            0,
            0,
            0,
            false,
            0,
            false
    ),
    TRANSFER(0,
            0,
            0,
            0,
            0,
            true,
            0,
            false
    ),
    BASKET_PREPARE(0,
            0,
            0,
            0,
            0,
            false,
            0,
            false
    ),
    BASKET_DROP(0,
            0,
            0,
            0,
            0,
            true,
            0,
            false
    );
// ToDo: Add more states for chambers



    public double armPos;
    public double pincherRotatorPos;
    public double intakeRotatorPos;
    public double intakeSlidePos;
    public double basketPos;
    public boolean pincherOpen;
    public int outtakeSlidePos;
    public boolean clawOpen;

    public double getArmPos(){
        return armPos;
    }
    public double getPincherRotatorPos(){
        return pincherRotatorPos;
    }
    public double getIntakeRotatorPos(){
        return intakeRotatorPos;
    }
    public double getSlidePos(){
        return intakeSlidePos;
    }
    public double getBasketPos(){
        return basketPos;
    }
    public boolean isPincherOpen(){
        return pincherOpen;
    }
    public int getOuttakeSlidePos(){
        return outtakeSlidePos;
    }
    public boolean isClawOpen(){
        return clawOpen;
    }

    RunStates(
            double armPos,
            double pincherRotatorPos,
            double intakeRotatorPos,
            double intakeSlidePos,
            double basketPos,
            boolean pincherOpen,
            int outtakeSlidePos,
            boolean clawOpen
    ){

        this.armPos = armPos;
        this.pincherRotatorPos = pincherRotatorPos;
        this.intakeRotatorPos = intakeRotatorPos;
        this.intakeSlidePos = intakeSlidePos;
        this.basketPos = basketPos;
        this.pincherOpen = pincherOpen;
        this.outtakeSlidePos = outtakeSlidePos;
        this.clawOpen = clawOpen;
    }
}
