// Lab2.java

package ca.mcgill.ecse211.localizationlab;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MedianFilter;
import lejos.hardware.Sound;

public class LocalizationLab {

  //Instantiate motors
  private static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

  private static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

  private static final EV3MediumRegulatedMotor sensorMotor =
      new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));

  private static final Port usPort = LocalEV3.get().getPort("S1");

  public static final double WHEEL_RADIUS = 2.1;
  public static final double TRACK = 15.4;
  public static final double Distance_ligntSensor = 9.5;

  public static void main(String[] args) throws InterruptedException {
    int buttonChoice;

    leftMotor.setAcceleration(3000);
    rightMotor.setAcceleration(3000);
    
    final TextLCD t = LocalEV3.get().getTextLCD();
    Odometer odometer = new Odometer(leftMotor, rightMotor);
    OdometryDisplay odometryDisplay = new OdometryDisplay(odometer, t);


    //Instantiate the Ultrasonic Sensor
    @SuppressWarnings("resource") // Because we don't bother to close this resource
    SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
    SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
    int filtPad = 5; //PRev 6
    SampleProvider average = new MedianFilter(usDistance,filtPad);    //MedianFilter for US data
    float[] usData = new float[average.sampleSize()]; // usData is the buffer in which data are
    
    
    //Instantiate localizer
    UltrasonicLocalizer ultraLoc = new UltrasonicLocalizer(odometer, leftMotor, rightMotor, sensorMotor, 
        usDistance,average,usData, WHEEL_RADIUS, TRACK,filtPad); // returned
    lightSensorLocalizer lightLoc = new lightSensorLocalizer(odometer, leftMotor, rightMotor,WHEEL_RADIUS, TRACK);
    
    do {

      // clear the display
      t.clear();

      // ask the user whether the motors should drive in a square or float
      t.drawString("Select Localizer", 0, 0);
      t.drawString("   Ultrasonic   ", 0, 1);
      t.drawString("                ", 0, 2);
      t.drawString("< Left | Right >", 0, 3);
      t.drawString("       |        ", 0, 4);
      t.drawString("Falling| Rising ", 0, 5);
      t.drawString("Edge   | Edge   ", 0, 6);
     

      buttonChoice = Button.waitForAnyPress();
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT&&buttonChoice != Button.ID_DOWN);
    

    if (buttonChoice == Button.ID_LEFT) { //Falling Edge

      odometer.start();
      odometryDisplay.start();
      ultraLoc.rising = false;
      ultraLoc.start();
      //Start UltrasonicLocalizer's Falling Edge Method
      buttonChoice = Button.waitForAnyPress();
      if(buttonChoice != Button.ID_ESCAPE){
          lightLoc.start();
      }

    } else if(buttonChoice == Button.ID_RIGHT){    //Rising Edge
      
      odometer.start();
      odometryDisplay.start();
      ultraLoc.rising = true;
      ultraLoc.start();
      buttonChoice = Button.waitForAnyPress();
      if(buttonChoice != Button.ID_ESCAPE){
          lightLoc.start();
      }
    }
      else if(buttonChoice == Button.ID_DOWN){
          
          odometer.start();
          odometryDisplay.start();
          lightLoc.start();
      }
  
    //Start UltrasonicLocalizer's Rising Edge Method
        //UltrasonicLocalizer should not return until it has localized thus not be a thread
        //Or alternatively be a thread that updates a boolean. 
      
      //buttonChoice = Button.waitForAnyPress();//Wait for any button press
      
      //Start LightLocalizer.java
    

    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);
  }
}
