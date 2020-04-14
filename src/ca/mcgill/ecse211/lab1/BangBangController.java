package ca.mcgill.ecse211.lab1;

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {

  private final int bandCenter;
  private final int bandwidth;
  private final int motorLow;
  private final int motorHigh;
  private int distance;
  
  private int countOut; //counter for filtering outlier values
  private int countCheck; //Counter to regulate countOut values; 
  private int countRev; //Counter to regulate reverse control 

  public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
    // Default Constructor
    this.bandCenter = bandCenter;
    this.bandwidth = bandwidth;
    this.motorLow = motorLow;
    this.motorHigh = motorHigh;
    WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
    WallFollowingLab.rightMotor.setSpeed(motorHigh);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();

    this.countOut = 0;
    this.countCheck = 0;
    this.countRev = 0;
  }

  @Override
  public void processUSData(int distance) {
    this.distance = (distance * 7071) / 10000; // Equivalent *(sqrt(2)/2), to compensate sensor angle

    int minBand = this.bandCenter - this.bandwidth; //minimum distance of band.
    int maxBand = this.bandCenter + this.bandwidth; //maximum distance of band.



    // TODO: process a movement based on the us distance passed in (BANG-BANG style)
    /*
     * if(this.distance > 700) { //if error go straight. this.countOut++;
     * 
     * }
     */
    if (this.distance >= minBand && this.distance <= maxBand) { // distance is in good band
      WallFollowingLab.leftMotor.setSpeed(this.motorHigh); // Start robot moving forward
      WallFollowingLab.rightMotor.setSpeed(this.motorHigh);
      WallFollowingLab.leftMotor.forward();
      WallFollowingLab.rightMotor.forward();
      
      this.countCheck++;
      if (this.countCheck > 7) { //if in good band for 7 cycles
        this.countOut = 0;
        this.countCheck = 0;
        this.countRev = 0;
      }
      
    } else if (this.distance <= 15) { //Too close to inner wall
      this.countRev++;
      if (this.countRev > 5) { //Turn in place if too close for 5 cycles
        WallFollowingLab.leftMotor.setSpeed(this.motorLow); 
        WallFollowingLab.rightMotor.setSpeed(this.motorLow);
        WallFollowingLab.leftMotor.forward();
        WallFollowingLab.rightMotor.backward(); //turns right
      }

    } else if (this.distance < minBand) { // Turn right slightly, out of bandwidth. 
        WallFollowingLab.leftMotor.setSpeed(this.motorHigh); 
        WallFollowingLab.rightMotor.setSpeed(this.motorLow); //Slow outer motor
        WallFollowingLab.leftMotor.forward();
        WallFollowingLab.rightMotor.forward();
    } else if (this.distance > maxBand && this.distance <= maxBand + 75) {// Turn Left slightly
        WallFollowingLab.leftMotor.setSpeed(this.motorLow + 50); // Inner motor slow down
        WallFollowingLab.rightMotor.setSpeed(this.motorHigh);
        WallFollowingLab.leftMotor.forward();
        WallFollowingLab.rightMotor.forward();

    } else if (this.distance > maxBand + 25) { // either outlier or hard left
      this.countOut++;
      if (this.countOut > 65) {// if not outlier turn hard.
        WallFollowingLab.leftMotor.setSpeed(this.motorLow - 25); // Inner motor slow down
        WallFollowingLab.rightMotor.setSpeed(this.motorHigh);
        WallFollowingLab.leftMotor.forward();
        WallFollowingLab.rightMotor.forward();

      } else { //Outlier thus go straight. 
        WallFollowingLab.leftMotor.setSpeed(this.motorHigh); // Start robot moving forward
        WallFollowingLab.rightMotor.setSpeed(this.motorHigh);
        WallFollowingLab.leftMotor.forward();
        WallFollowingLab.rightMotor.forward();
      }
    }
  }

  @Override
  public int readUSDistance() {
    return this.distance;
  }
}
