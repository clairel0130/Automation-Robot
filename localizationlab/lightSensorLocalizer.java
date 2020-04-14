package ca.mcgill.ecse211.localizationlab;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.hardware.Sound;

public class lightSensorLocalizer extends Thread {
  private static final int ROTATE_SPEED = 100;
  private Odometer odometer;
  private EV3LargeRegulatedMotor leftMotor, rightMotor;
  private double wheelRadius, width, Theta;

  // Instantiate Light Sensor
  private static Port portColor = LocalEV3.get().getPort("S4");
  private static SensorModes myColor = new EV3ColorSensor(portColor);
  private static SampleProvider myColorSample = myColor.getMode("Red");
  static float[] sampleColor = new float[myColor.sampleSize()];

  public lightSensorLocalizer(Odometer odometer, EV3LargeRegulatedMotor leftmotor,
      EV3LargeRegulatedMotor rightmotor, double wheelRadius, double track) {
    this.Theta = 0;
    this.odometer = odometer;
    this.leftMotor = leftmotor;
    this.rightMotor = rightmotor;
    this.wheelRadius = wheelRadius;
    this.width = track;
    this.leftMotor.setSpeed(ROTATE_SPEED);
    this.rightMotor.setSpeed(ROTATE_SPEED);
  }

  public void run() {
    long correctionStart, correctionEnd, sleepTime;
    double x = 0;
    double y = 0;
    double thetaX = 0;
    double thetaY = 0;
    double deltaTheta = 0;
    double[] thetaArray = new double[4];
    boolean firstPoint = false;
    boolean oriented = false;
    float color = 0;

    this.Theta = this.odometer.getTheta();
    turnTo(45); // Initally facing 0 as according to our ultrasonic localization

    leftMotor.forward();
    rightMotor.forward();

    int j = 0;
    int counter = 0;

    while (!oriented) {
      myColorSample.fetchSample(sampleColor, 0);
      color = sampleColor[0] * 1000;

      if (!firstPoint) { // If yet to find first point
        if ((color < 300) && (color > 5)) { // Keep driving until see first point
          x = odometer.getX();
          while (odometer.getX() < (x + 3.5)) {
          } // move sensor past first point

          firstPoint = true;
        }
      } else { // Seen first point already

        leftMotor.forward(); // Turn clockwise
        rightMotor.backward();

        myColorSample.fetchSample(sampleColor, 0);
        color = sampleColor[0] * 1000;

        if ((color < 300) && (color > 5)) { // If detect a line
          counter++;
        }
        if (counter >= 3) { // filter to not double count line
          Sound.beep();
          thetaArray[j] = odometer.getTheta(); // Store angle when detect line
          j++;
          counter = 0;
        }

        if (j == 4) { // If all 4 lines have been detected
          thetaX = thetaArray[2] - thetaArray[0];// angles at which x axis was detected
          thetaY = thetaArray[1] - thetaArray[3];// angles at which y axis was detected

          double thetaXrad = thetaX * Math.PI / 180; // Convert to radians for computation
          double thetaYrad = thetaY * Math.PI / 180;

          // correct x and y positions
          x = - LocalizationLab.Distance_ligntSensor * Math.cos(thetaXrad / 2);
          y = - LocalizationLab.Distance_ligntSensor * Math.cos(thetaYrad / 2);

         

          odometer.setX(x);
          odometer.setY(y);

          deltaTheta = -90 - thetaY / 2 + thetaArray[1];
      
          odometer.setTheta(this.odometer.getTheta() + deltaTheta); // correct theta
          this.Theta = odometer.getTheta();

          leftMotor.stop();
          rightMotor.stop();
          oriented = true; // end loop
        }
      }
    }
    turnTo(0); //Once oriented
    while (true) {
    }
  }


  // Helper methods from the Navigation lab
  private void turnTo(double xpTheta) {
    double deltaTheta1 = xpTheta - odometer.getTheta();
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);

    // angular correction, aims at doing the minimal angle
    if (deltaTheta1 <= 180.0 && deltaTheta1 >= -180.0) {
      leftMotor.rotate(convertAngle(wheelRadius, width, deltaTheta1), true);
      rightMotor.rotate(-convertAngle(wheelRadius, width, deltaTheta1), false);


    } else if (deltaTheta1 < -180.0) {
      deltaTheta1 += 360.0; // Updates the correction to minimum angle
      leftMotor.rotate(convertAngle(wheelRadius, width, deltaTheta1), true);
      rightMotor.rotate(-convertAngle(wheelRadius, width, deltaTheta1), false);
    } else {
      deltaTheta1 -= 360;
      leftMotor.rotate(convertAngle(wheelRadius, width, deltaTheta1), true);
      rightMotor.rotate(-convertAngle(wheelRadius, width, deltaTheta1), false);
    }
  }

  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
}

