package ca.mcgill.ecse211.lab1;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 200; 
  private static final int FILTER_OUT = 60; 

  private final int bandCenter;
  private final int bandWidth;
  private final int minBand;
  private final int maxBand;
  
  private int distance;
  private int filterControl; //filters out values too large
  private int checkFilterControl; //ensure filter control does not reset prematurely
  private int turnControl; // counter to control turning sharpness. 

  public PController(int bandCenter, int bandwidth) {
    this.bandCenter = bandCenter + 5;
    this.bandWidth = bandwidth - 4;
    this.filterControl = 0;
    this.checkFilterControl = 0;
    this.turnControl = 0;
    this.minBand = this.bandCenter - this.bandWidth;
    this.maxBand = this.bandCenter + this.bandWidth;

    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  @Override
  public void processUSData(int distance) {
    this.distance = (distance * 7071) / 10000; // Equivalent *(sqrt(2)/2), compensate sensor angle

    boolean err = false; //flag to reset correction

    // rudimentary filter - toss out invalid samples corresponding to null
    // signal.
    // (n.b. this was not included in the Bang-bang controller, but easily
    // could have).
    //
    if (distance >= 98 && filterControl < FILTER_OUT) { 
      filterControl++;
      err = true;
    } else if (distance >= 98) { 
      // We have repeated large values, so there must actually be nothing
      // there: leave the distance alone
      this.distance = distance;
    } else {
      // distance went below 98: reset filter and leave
      // distance alone.
      this.checkFilterControl++;
      if (this.checkFilterControl > 2) { //checks if distance below 98 reading is not random
        filterControl = 0;
        this.checkFilterControl = 0;
      }
      this.distance = distance;
    }

    // TODO: process a movement based on the us distance passed in (P style)
    int correction;
    double PROPCONST = 20;
    int diff = this.bandCenter - this.distance;
    if (diff < 0) { 
      diff = -diff;
    }
    if (err == true) { //if reading is an error have no correction. 
      correction = 0;
    } else {
      correction = (int) (PROPCONST * (double) diff);
      if (correction >= MOTOR_SPEED) {// correction is equal to our motor_speed (200)
        correction = 200;
      }
    }

    if (this.distance >= this.minBand && this.distance <= this.maxBand) { // distance is correct
      WallFollowingLab.leftMotor.setSpeed(this.MOTOR_SPEED); // Start robot moving forward
      WallFollowingLab.rightMotor.setSpeed(this.MOTOR_SPEED);
      WallFollowingLab.leftMotor.forward();
      WallFollowingLab.rightMotor.forward();
    } else if (this.distance < this.minBand) { // Too close to the wall (LHS)
      WallFollowingLab.leftMotor.setSpeed(this.MOTOR_SPEED);
      WallFollowingLab.rightMotor.setSpeed(this.MOTOR_SPEED - correction); // outer motor slow down
      WallFollowingLab.leftMotor.forward(); // Turns right
      WallFollowingLab.rightMotor.forward(); 
    } else if (this.distance > this.maxBand) {// Too far
      this.turnControl++;
      WallFollowingLab.leftMotor.setSpeed(this.MOTOR_SPEED - correction); // Inner motor slow down
      WallFollowingLab.rightMotor.setSpeed(this.MOTOR_SPEED);
      WallFollowingLab.leftMotor.forward();
      WallFollowingLab.rightMotor.forward();
      if (this.turnControl >= 5 && this.turnControl <= 7) { // Every 5 cycles move cart forward
        WallFollowingLab.leftMotor.setSpeed(this.MOTOR_SPEED); // Done to widen turning radius
        WallFollowingLab.rightMotor.setSpeed(this.MOTOR_SPEED);
        WallFollowingLab.leftMotor.forward();
        WallFollowingLab.rightMotor.forward();
      } else if (this.turnControl > 7) {
        this.turnControl = 0;
      }
    }
  }



  @Override
  public int readUSDistance() {
    return this.distance;
  }

}
