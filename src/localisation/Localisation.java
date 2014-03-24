package localisation;
import java.util.ArrayList;

import lejos.nxt.Button;
import lejos.nxt.LightSensor;
import lejos.nxt.SensorPort;
import lejos.nxt.Sound;
import lejos.nxt.UltrasonicSensor;
import lejos.util.Delay;
import rp.robotics.localisation.ActionModel;
import rp.robotics.localisation.DummySensorModel;
import rp.robotics.localisation.GridPositionDistribution;
import rp.robotics.mapping.GridMap;
import rp.robotics.mapping.Heading;
import rp.robotics.mapping.LocalisationUtils;
import rp.robotics.visualisation.GridPoseDistributionVisualisation;
import rp13.search.interfaces.Agenda;
import actionModel.ActualPerfectActionModel;
import agendas.AgendaListA;
import search.SearchingFramework;
import sensorModel.SensorModel;
import Grid.Grid;
import Grid.Grid.RobotMove;
import Grid.GridSuccessorFunction;

	/** the class for part 2 grid
	 * 
	 * @author Nelson
	 * @param light front sensor
	 * @param ligh2 side sensor
	 * @param black black value
	 * @param sensor touchsensor
	 * 
	 * 
	 */
	public class Localisation {
		private static LightSensor light ;
		private static LightSensor light2 ;
		private static MotorRun motor ;
		private static UltrasonicSensor sensor ;
		private final static int black = 400;
		private static int degree =2;
		private static int direction =2;
		private static GridMap gridMap ;
		// The probability distribution over the robot's location
		private static GridPositionDistribution distribution ;
		
		//private static TouchSensor sensor;
		
		public Localisation(){
			light =new LightSensor(SensorPort.S2);
			light2 =new LightSensor(SensorPort.S3);
			motor = new MotorRun();
			sensor = new UltrasonicSensor(SensorPort.S1);
			gridMap = LocalisationUtils.create2014Map1();
			// The probability distribution over the robot's location
			distribution = new GridPositionDistribution(gridMap);
		}
	public static void main(String[] args) {
		Localisation localisation = new Localisation();
		//Initialize the Robot thing
		System.out.println("Grid");
		Button.waitForAnyPress();
		localisation.localisation();
		//first goal
		localisation.searchForGoal(localisation.getLocationX(),localisation.getLocationY(),0,0);
		//second goal
		localisation.searchForGoal(0,0,0,0);
	}		
	private  void localisation(){
		boolean i=true;//turn left then turn right 
		ActionModel actionModel = new ActualPerfectActionModel();
		SensorModel sensorModel = new SensorModel();
		Heading action = Heading.PLUS_X;
		while(!getLocation()){
			//front sensor reads black (on the track)
			if(light.readNormalizedValue()<=black){
				motor.forward();
				Thread.yield();
				degree=2;
				//if side sensor reads black means it is cross turn when going forward
				if(light2.getNormalizedLightValue()<=black)
				{
					//please place this in front of the start a little bit
					motor.stop();
					changeDirection(action);
					distribution = actionModel.updateAfterMove(distribution, action);
					if(sensor.getDistance()>gridMap.getCellSize()){
						motor.forward(); //if front no block forward
					}else if(i){
						motor.AdjLeft(90);
						direction -=1;
						if(direction<0){
							direction =3;
						}
						if(direction>3){
							direction =0;
						}
						i=false;
					}else if(!i){
						motor.AdjRight(90);
						direction +=1;
						if(direction<0){
							direction =3;
						}
						if(direction>3){
							direction =0;
						}
						i=true;
					}
				}
			}////if side reads black
			//front sensor not read black (means not on the track then do some adjustment)
			if(light.readNormalizedValue()>black){
					motor.AdjLeft(degree);
					Thread.yield();
					degree=degree+2;	
			}
			if(light.readNormalizedValue()>black){
					motor.AdjRight(degree);
					Thread.yield();
					degree=degree+2;					
			}
			//// for adj forward movement
		}//end of while loop 
		Sound.beep();//beep when can get current location
		System.out.println("X: "+getLocationX() + "Y: " +getLocationY());
	}
	private void changeDirection(Heading action){
		if(direction==0){
			action = Heading.MINUS_X;
		}else if(direction ==1){
			action = Heading.MINUS_Y;
		}else if(direction ==2){
			action = Heading.PLUS_X;
		}else if(direction ==3){
			action = Heading.PLUS_Y;
		}
	}
	private boolean getLocation(){
		for(int i =0 ;i<gridMap.getGridHeight();i++)
			for(int j =0; j<gridMap.getGridWidth();j++){
				if(distribution.getProbability(i, j)>0.65)
					return true;
			}
		return false;
	}
	private int getLocationX(){
		for(int i =0 ;i<gridMap.getGridHeight();i++)
			for(int j =0; j<gridMap.getGridWidth();j++){
				if(distribution.getProbability(i, j)>0.65)
					return j;
			}
		return 0;
	}
	private int getLocationY(){
		for(int i =0 ;i<gridMap.getGridHeight();i++)
			for(int j =0; j<gridMap.getGridWidth();j++){
				if(distribution.getProbability(i, j)>0.65)
					return i;
			}
		return 0;
	}
	private void searchForGoal(int robotX,int robotY ,int goalX ,int goalY){
	
		GridSuccessorFunction function = new GridSuccessorFunction();
		Grid puzzle = new Grid(10,7);
		//setup some blocks and goal and robot position
		//only works with small grid and less complicated route 
		//because of the less memory of the robot
		puzzle.setRobot(robotX,robotY);
		puzzle.setGoal(goalX,goalY);
		puzzle.setRobotDirection(direction);
		puzzle.setBlock(1,0,2,0);
		puzzle.setBlock(0,1,1,1);
		puzzle.setBlock(0,2,0,3);
		puzzle.setBlock(2,2,2,3);
		puzzle.setBlock(2,3,3,3);
		puzzle.setBlock(1,5,2,5);
		puzzle.setBlock(3,6,4,6);
		puzzle.setBlock(5,5,5,6);
		puzzle.setBlock(4,4,5,4);
		puzzle.setBlock(4,1,5,1);
		puzzle.setBlock(4,2,5,2);
		puzzle.setBlock(5,2,5,3);
		puzzle.setBlock(6,4,7,4);
		puzzle.setBlock(6,5,7,5);
		puzzle.setBlock(5,0,6,0);

		/** doing the searching stuff
		 * works with a small size grid and less complicated route
		 * 
		 */
		
		Agenda<Grid> agenda = new AgendaListA<Grid>();
		SearchingFramework<RobotMove,Grid,GridSuccessorFunction > search 
		= new SearchingFramework<RobotMove,Grid,GridSuccessorFunction >
		(function, puzzle, agenda);
		search.Search();
		ArrayList<RobotMove> solution = new ArrayList<RobotMove>();
		solution.addAll(search.getResult());

		
		

		Delay.msDelay(500);
		int index = 0;//index for the list of moves
		ArrayList<RobotMove> moved = new ArrayList<RobotMove>(); //record moves for detect online
		boolean isGoal = false;
		
		/** the algorithm is more or less the same as line following 
		 * 
		 */
		while(!isGoal){
			try{
				//front sensor reads black (on the track)
				if(light.readNormalizedValue()<=black){
					motor.forward();
					Thread.yield();
					degree=2;
					//if side sensor reads black means it is cross turn when going forward
					if(light2.getNormalizedLightValue()<=black)
					{
						motor.stop();
						motor.AdjLeft(solution.get(index).r_move*90);//left(1)*90 forward(0)*90 right(-1)*90
						motor.forward();
						Delay.msDelay(200);
						index++;//increase index every time robot turns (like a for loop)
						moved.add(solution.get(index));	// add move to the list to store for online detection
					}
				}
				//front sensor not read black (means not on the track then do some adjustment)
				if(light.readNormalizedValue()>black){
						motor.AdjLeft(degree);
						Thread.yield();
						degree=degree+2;	
				}
				if(light.readNormalizedValue()>black){
						motor.AdjRight(degree);
						Thread.yield();
						degree=degree+2;					
				}
			}
			catch(IndexOutOfBoundsException e){ 
				//agenda have no more entry or reaches the end of the solution list
				//then perform all move already then is GOAL YEAHHH
				Sound.beep();
				motor.forward();
				Delay.msDelay(1300);
				motor.stop();
				isGoal = true;
			}	
		}
	}
}


