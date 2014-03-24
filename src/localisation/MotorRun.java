package localisation;

import lejos.nxt.Motor;
import lejos.robotics.navigation.DifferentialPilot;

/**The Class that control the robot to moves
 */
public class MotorRun {

	private static DifferentialPilot pilot = new DifferentialPilot(2.1f,4.4f,Motor.B,Motor.A);
	
	
	public MotorRun(){
		pilot.setTravelSpeed(7); //set speed
		pilot.setRotateSpeed(1000); //set rotate speed
		
	}
	
	public void AdjLeft(int degree)
	{
		pilot.rotate(degree); //rotate left with a degree
	}
	public void AdjRight(int degree)
	{
		
		pilot.rotate(-degree);//rotate right with a degree
	}
	
	public void TurnLeft()
	{
		pilot.rotateLeft(); //keep rotating left
	}
	public void TurnRight()
	{
		pilot.rotateRight();//keep rotating right
	}
	public void forward()
	{
		
		pilot.forward(); //forward
	}
	
	public void backward(){
		pilot.backward(); //backward
	}
	
	public void stop(){
		pilot.stop(); //stop
	}
}