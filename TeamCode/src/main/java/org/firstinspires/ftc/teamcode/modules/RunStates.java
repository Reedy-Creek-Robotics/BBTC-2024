package org.firstinspires.ftc.teamcode.modules;

public enum RunStates {
    // Sets the Intake just in front of the bot, so slides can lift safely
    DEFAULT(
    0.54,
    0.63,
    .008,
               0.75,
               0.5,
               false,
               0,
            false
    ),
    // Sit in front of the samples, prepared to pick up.
    PICKING(
            0.5,
            -1,
            0.1,
            -1,
            0.5,
            true,
            0,
            false
    ),
    // Grab the sample.
    GRAB(
            0.58,
            -1,
            0.1,
            -1,
            0.5,
            false,
            0,
            false
    ),
    // Hold the sample above the transfer bucket
    HOLD(0,
            0,
            0,
            0,
            0,
            false,
            0,
            false
    ),
    // Drop the sample into the transfer bucket.
    TRANSFER(0,
            0,
            0,
            0,
            0,
            true,
            0,
            false
    ),
    // Leave the bucket down, lift slides to top basket
    BASKET_PREPARE(0,
            0,
            0,
            0,
            0,
            false,
            0,
            false
    ),
    // Tilt the bucket to slide the sample into the top basket
    BASKET_DROP(0,
            0,
            0,
            0,
            0,
            true,
            0,
            false
    ),
    // Drop the sample into out the front of the bot, for observation zone
    DROP_FRONT(
            0,
            0,
            0,
            0,
            0,
            true,
            0,
            false
    ),
    // Prepare to grab a specimen from the wall
    PREPARE_WALL(
            0,
            0,
            0,
            0,
            0,
            false,
            0,
            true
    ),
    // Pick up a specimen from the wall
    GRAB_WALL(
            0,
            0,
            0,
            0,
            0,
            true,
            0,
            false
    ),
    // Hover above the top chamber
    PREPARE_CHAMBER(
            0,
            0,
            0,
            0,
            0,
            false,
            0,
            false
    ),
    // Score on the top chamber
    SCORE_CHAMBER(
            0,
            0,
            0,
            0,
            0,
            true,
            0,
            false
    );


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
