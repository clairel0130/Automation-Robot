package ca.mcgill.ecse211.localizationlab;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.hardware.Sound;// Debugging utility

public class UltrasonicLocalizer extends Thread {
  private static final int ROTATE_SPEED = 50;
  private static final int usMax = 2047483647;
  private double Theta, X, Y; // Temp variables, passed from Odometer
  private Odometer odometer;
  private EV3LargeRegulatedMotor leftMotor, rightMotor;
  private EV3MediumRegulatedMotor sensorMotor;
  private SampleProvider us, avg;
  private final int pad;
  private float[] usData;
  private double wheelRadius, width;
  public boolean rising;

  private final double dCenter = 35.0; //band center
  private final double dOff = 5.0; //band width. 

  public UltrasonicLocalizer(Odometer odometer, EV3LargeRegulatedMotor leftmotor,
      EV3LargeRegulatedMotor rightmotor, EV3MediumRegulatedMotor sensormotor,
      SampleProvider usDistance, SampleProvider average, float[] usData, double wheelRadius,
      double track, int filterPad) {

    this.odometer = odometer;
    this.leftMotor = leftmotor;
    this.rightMotor = rightmotor;
    this.sensorMotor = sensormotor;
    this.us = usDistance;
    this.avg = average;
    this.usData = usData;
    this.wheelRadius = wheelRadius;
    this.width = track;
    this.leftMotor.setSpeed(ROTATE_SPEED);
    this.rightMotor.setSpeed(ROTATE_SPEED);
    pad = filterPad;
  }

  public void run() {

    for (int i = 1; i <= pad; i++) { // Gain initial readings to padd filter
      avg.fetchSample(usData, 0);
    }
    if (this.rising) { // If rising edge method selected
      try {
        System.out.println("Rising");
        risingEdge();
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    } else {
      try {
        fallingEdge();
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }

  }

  public void risingEdge() throws InterruptedException { // Throws InterruptedException is a
                                                         // temporary effect for debugging
    int dist;
    int point = 0;
    int count = 0;
    boolean Oriented = false;
    boolean firstEntry = true;
    double angle1 = 0.0, angle2 = 0.0, alpha = 0.0, beta = 0.0, delta;

    leftMotor.forward(); // Turn in place clockwise
    rightMotor.backward();

    while (!Oriented) {
      this.Theta = odometer.getTheta(); // Get current theta
      avg.fetchSample(usData, 0); // Acquire USdata
      dist = (int) (usData[0] * 100.0); // Extract from buffer and cast to int

      if (point == 2) {// detected both walls
        System.out.printf("Alpha:, %f %n Beta:, %f %n", alpha, beta);
        if (alpha > beta) {
          System.out.println("opt1");
          delta = 45 - ((alpha + beta) / 2);
          odometer.setTheta(this.Theta + delta); // Resets odometer
          this.Theta += delta;
          System.out.printf("Final Theta:, %f %n", this.Theta);
          Sound.twoBeeps();
          leftMotor.stop();
          rightMotor.stop();
          Thread.sleep(8000);
          turnTo(0);
          Oriented = true;
          return;
        } else if (alpha < beta) {
          System.out.println("opt2");
          delta = 225 - ((alpha + beta) / 2);
          odometer.setTheta(this.Theta + delta);
          leftMotor.stop();
          rightMotor.stop();
          Thread.sleep(8000);
          turnTo(0);
          Oriented = true;
          return;
        }
      }

      if (dist <= (dCenter + dOff)) { // Less than Max
        if (dist >= (dCenter - dOff)) {// Within band
          System.out.println("In band");
          if (firstEntry) {
            System.out.println("First Entry");
            angle1 = this.Theta; // Record entering angle to the band
            firstEntry = false;
            count = 0;
          }
        } else {// Beneath minimum bound
          System.out.println("Beneath");
          count++; // Counter to filter errors
          if (!firstEntry && count > 1) { // First exit from band.
            System.out.println("First Exit below");
            angle2 = this.Theta; // Record exiting angle from the band
            firstEntry = true;
            alpha = (angle1 + angle2) / 2; // alpha is the angle detected of the back wall
            // because if getting closer to wall after band and clockwise, must be back right corner
            point++;// counter of number of walls detected
          }
        }
      } else {// Above band max
        count++;
        if (!firstEntry && count > 1) {// first exit from band
          System.out.println("First Exit above");
          angle2 = this.Theta; // Record exiting angle from the band
          firstEntry = true;
          beta = (angle1 + angle2) / 2; // Beta is the angle detected of the left wall
          point++;
          count = 0;
          System.out.printf("Point:, %d %n", point);
          if (point == 1) { // First wall detected is the left wall
            leftMotor.backward(); // Turn so that you detect the back wall as rising edge next
            rightMotor.forward();
            while (odometer.getTheta() > angle1 - 7); // Pad by 7 degrees to ensure no double
                                                      // detection
            for (int j = 0; j < pad + 4; j++) {
              avg.fetchSample(usData, 0); // Acquire USdata to update buffer
            }
          }
        }
      }
    }
  }

  // Got rid of averaging and using the median filter.
  public void fallingEdge() throws InterruptedException {
    int dist;
    int point = 0;
    int count = 0;
    boolean Oriented = false;
    boolean firstEntry = true;
    int where = -2; // -2 implies uninitialized
    double angle1 = 0.0, angle2 = 0.0, alpha = 0.0, beta = 0.0, delta;

    // Put above while loop to not interfere when turning in the opposite direction.
    leftMotor.forward(); // Turn in place clockwise
    rightMotor.backward();

    while (!Oriented) {
      this.Theta = odometer.getTheta(); // Get current theta
      System.out.printf("Theta:, %f %n", this.Theta);
      us.fetchSample(usData, 0); // Acquire USdata
      dist = (int) (usData[0] * 100.0); // Extract from buffer and cast to int
      System.out.printf("Dist:, %d %n", dist); // Print out dist

      if (where == -2) { // If uninitialized position relative to walls
        for (int k = 0; k < pad; k++) {
          avg.fetchSample(usData, 0);
        }
        dist = (int) (usData[0] * 100.0);
        if (dist > (dCenter + dOff)) {
          where = 1; // I.e. Above band.
        } else if (dist < (dCenter - dOff)) {
          where = -1; // I.e. Below band
        } else {
          where = 0; // I.e. in band
        }
      }

      if (point == 2) {// I.e. First detection of the second angle
        System.out.printf("Alpha:, %f %n Beta:, %f %n", alpha, beta);
        if (alpha > beta) {
          delta = 45 - ((alpha + beta) / 2);
          odometer.setTheta(this.Theta + delta); // Resets odometer
          this.Theta += delta;
          System.out.printf("Final Theta:, %f %n", this.Theta);

          leftMotor.stop();
          rightMotor.stop();
          Thread.sleep(8000);// Sleep thread for 8 seconds to manually look at value on screen
          turnTo(0);
          Oriented = true;
          return;
        } else if (alpha < beta) {
          System.out.println("opt2");
          delta = 225 - ((alpha + beta) / 2);
          odometer.setTheta(this.Theta + delta);
          leftMotor.stop();
          rightMotor.stop();
          Thread.sleep(8000);
          turnTo(0);
          Oriented = true;
          return;
        }
        return;
      }

      System.out.printf("Where: %d %n", where);

      if (dist <= (dCenter + dOff)) { // Less than Max
        if (dist >= (dCenter - dOff)) {// Within band
          where = 0;
          if (firstEntry) {
            System.out.println("First Entry in band");

            angle1 = this.Theta; // Record entering angle to the band
            firstEntry = false;
            count = 0;
          }
        } else {// Beneath minimum bound

          System.out.println("Beneath");
          count++; // Filter
          if (where >= 0 && count > 1) { // if first entry to beneath band
            where = -1;
            System.out.println("First Exit below");
            angle2 = this.Theta; // Record exiting angle from the band
            firstEntry = true;
            alpha = odometer.getTheta(); // alpha is the angle detected of the back wall
            // because if getting closer to wall after band and clockwise, must be back right corner
            point++;// counts number of walls seen
            System.out.printf("Point:, %d %n", point);
            Sound.beep();

            if (point == 1) { // If see falling edge and first point turn around.
              System.out.printf("Curr Point:, %d %n", point);
              System.out.println("Turn Around");
              leftMotor.backward();// turn to detect left wall as a falling edge
              rightMotor.forward();
              while (odometer.getTheta() > angle2 - 7); // Pad by 7 degrees to ensure no double
                                                        // detection
              Sound.buzz();
              for (int j = 0; j < pad + 4; j++) {
                us.fetchSample(usData, 0); // Acquire USdata to update buffer
              }
              where = 1;
            }
          } else if (count > 2) {
            where = -1;
          }
        }
      } else {// Above band max
        count++;
        if (where <= 0 && count > 1 && point == 0) {
          where = 1;
          System.out.println("First Exit above");
          firstEntry = true;
          beta = odometer.getTheta();// Beta is the angle detected of the left wall
          System.out.printf("Beta is this: %f %n:", beta);
          point++;
          count = 0;
          System.out.printf("Point:, %d %n", point);
          Sound.beep();
        } else if (count > 2) {
          where = 1;
        }
      }
    }
  }


  // Helper methods from Navigation Lab
  private void turnTo(double xpTheta) {
    xpTheta+=90;
    double deltaTheta = xpTheta - this.Theta;
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);

    // angular correction, aims at doing the minimal angle
    if (deltaTheta <= 180.0 && deltaTheta >= -180.0) {
      leftMotor.rotate(convertAngle(wheelRadius, width, deltaTheta), true);
      rightMotor.rotate(-convertAngle(wheelRadius, width, deltaTheta), false);


    } else if (deltaTheta < -180.0) {
      deltaTheta += 360.0; // Updates the correction to minimum angle
      leftMotor.rotate(convertAngle(wheelRadius, width, deltaTheta), true);
      rightMotor.rotate(-convertAngle(wheelRadius, width, deltaTheta), false);
    } else {
      deltaTheta -= 360;
      leftMotor.rotate(convertAngle(wheelRadius, width, deltaTheta), true);
      rightMotor.rotate(-convertAngle(wheelRadius, width, deltaTheta), false);
    }
  }

  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
}

