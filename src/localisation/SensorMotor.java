package localisation;

import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.addon.OpticalDistanceSensor;

public class SensorMotor {
	private static NXTRegulatedMotor motor = Motor.C;
	private OpticalDistanceSensor sensor;
	
	public SensorMotor(OpticalDistanceSensor sensor){
		this.sensor = sensor;
	}
	
	
	public void turnLeft(){
		motor.rotate(90);
	}
	public void turnRight(){
		motor.rotate(-90);
	}
	
	public float getFrontReading(){
		return (float)sensor.getDistance();
	}
	
	public float getLeftReading(){
		turnLeft();
		float result = (float)sensor.getDistance();
		turnRight();
		return result;
	}
	public float getRightReading(){
		turnRight();
		float result = (float)sensor.getDistance();
		turnLeft();
		return result;
	}
	
	public float getbackReading(){
		turnLeft();
		turnLeft();
		float result = (float)sensor.getDistance();
		turnLeft();
		turnLeft();
		return result;
	}
	
}
