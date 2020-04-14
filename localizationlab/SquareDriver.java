/*
 * SquareDriver.java
 */
package ca.mcgill.ecse211.localizationlab;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class SquareDriver extends Thread{
  private static final int FORWARD_SPEED = 200;
  private static final int ROTATE_SPEED = 100;
  private double[] map = new double[10];
  private int pointMap;
 private double Theta,X,Y; //Temp variables, should be passed in the constru
 private Odometer odometer;
 private EV3LargeRegulatedMotor leftMotor,rightMotor;
 private double leftRadius, rightRadius, width;
 
 
  public SquareDriver(Odometer odometer,EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      double leftRadius, double rightRadius, double width) {
   this.odometer = odometer;
   this.pointMap = 0;
   this.map[0] = 0.0;
   this.map[1] = 2.0;
   this.map[2] = 1.0;
   this.map[3] = 1.0;
   this.map[4] = 2.0;
   this.map[5] = 2.0;
   this.map[6] = 2.0;
   this.map[7] = 1.0;
   this.map[8] = 1.0;
   this.map[9] = 0.0;
   this.leftMotor = leftMotor;
   this.rightMotor = rightMotor;
   this.leftRadius = leftRadius;
   this.rightRadius = rightRadius;
   this.width = width;
   //this.deltaTheta = 0;
   for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {this.leftMotor, this.rightMotor}) {
     motor.stop();
     motor.setAcceleration(3000);
   }
  }

  public void run() {
   
    this.Theta = this.odometer.getTheta();
    this.X = this.odometer.getX();
    this.Y = this.odometer.getY();
    travelTo(this.map[this.pointMap], this.map[this.pointMap+1]);
  }
  
  
  private void travelTo(double nextX, double nextY) {
    double expTheta = 0.0; 
        
    // wait 5 seconds
    try {
      Thread.sleep(500);
    } catch (InterruptedException e) {//NOTE: this isn't true anymore. 
      // there is nothing to be done here because it is not expected that
      // the odometer will be interrupted by another thread
    }
 // calculate the theta needed
    expTheta = Math.atan((nextX-this.X)/(nextY-this.Y));
    expTheta *= 180/3.1415; //Converts expTheta to degrees
    
    if(this.Theta >= expTheta-1 && this.Theta <= expTheta+1) {//Drive forward
     // driveDist = Math.sqrt()
      // drive forward two tiles
      leftMotor.setSpeed(FORWARD_SPEED);
      rightMotor.setSpeed(FORWARD_SPEED);

      //leftMotor.rotate(convertDistance(leftRadius, 91.44), true); //91.44 is the d
      //rightMotor.rotate(convertDistance(rightRadius, 91.44), false);
    }else{
      
      turnTo(expTheta);
    }
    
  }
  
  private void turnTo(double xpTheta){
    double deltaTheta = xpTheta - this.Theta;
    if(deltaTheta <= 180.0 && deltaTheta>= -180.0) { 
      if(deltaTheta>0) {
        leftMotor.rotate(convertAngle(leftRadius, width, deltaTheta), true);
        rightMotor.rotate(-convertAngle(rightRadius, width, deltaTheta), false);
      }else {
        leftMotor.rotate(-convertAngle(leftRadius, width, deltaTheta), false);
        rightMotor.rotate(convertAngle(rightRadius, width, deltaTheta), true);
      }
      
      
    }else if(deltaTheta<-180.0) {
      deltaTheta += 360.0; //Updates the correction to minimum angle
      if(deltaTheta>0) {
        leftMotor.rotate(convertAngle(leftRadius, width, deltaTheta), true);
        rightMotor.rotate(-convertAngle(rightRadius, width, deltaTheta), false);
      }else {
        leftMotor.rotate(-convertAngle(leftRadius, width, deltaTheta), false);
        rightMotor.rotate(convertAngle(rightRadius, width, deltaTheta), true);
      }
    }
    else {
      deltaTheta -= 360;
      if(deltaTheta>0) {
        leftMotor.rotate(convertAngle(leftRadius, width, deltaTheta), true);
        rightMotor.rotate(-convertAngle(rightRadius, width, deltaTheta), false);
      }else {
        leftMotor.rotate(-convertAngle(leftRadius, width, deltaTheta), false);
        rightMotor.rotate(convertAngle(rightRadius, width, deltaTheta), true);
      }  
    }
  }

  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
}
