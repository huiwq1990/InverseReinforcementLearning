package champ2011client;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

import org.apache.commons.math.linear.Array2DRowRealMatrix;
import org.apache.commons.math.linear.ArrayRealVector;
import org.apache.commons.math.linear.RealMatrix;
import org.apache.commons.math.linear.RealVector;

import champ2011client.behaviour.*;

/**
 * Our controller
 * @author Derk
 *
 */
public class Schumacher extends Controller {

    final double targetSpeed = 15;

    protected StandardGearChangeBehaviour gearBehaviour = new StandardGearChangeBehaviour();
    protected ClutchBehaviour clutchBehaviour = new ClutchBehaviour();
    
    protected RealMatrix a, b, actions;
    protected RealVector theta;
    
    protected double previousAngle = 0;
    
    public Schumacher() throws FileNotFoundException {
    	double[][] dataA = readMatrixFromCsv("../TORCS-Dynamics/A.csv");
    	double[][] dataB = readMatrixFromCsv("../TORCS-Dynamics/B.csv");
    	double[][] dataTheta = readMatrixFromCsv("../TORCS-Dynamics/theta.csv");
    	double[][] dataActions = readMatrixFromCsv("../TORCS-Dynamics/actions.csv");
    	
    	a = new Array2DRowRealMatrix(dataA);
    	b = new Array2DRowRealMatrix(dataB);
    	theta = new Array2DRowRealMatrix(dataTheta).getColumnVector(0);
    	actions = new Array2DRowRealMatrix(dataActions);
    }
    
    protected double[][] readMatrixFromCsv(String filename) throws FileNotFoundException {
    	List<String> data = new ArrayList<String>();
    	FileReader fin = new FileReader(filename);
    	Scanner sc = new Scanner(fin);
    	while(sc.hasNext()) {
    		data.add(sc.nextLine());
    	}
    	sc.close();

    	double[][] matrix = new double[data.size()][];
    	int count = -1;
    	for(String s : data) {
    		String[] temp = data.get(++count).split(",");
    		matrix[count] = new double[temp.length];
    		for(int i = 0; i < temp.length; i++) {
    			matrix[count][i] = Double.valueOf(temp[i]);
    		}
    	}
    	
    	return matrix;
    }
    
    public RealVector mapState(RealVector state) {
        double[] f = new double[7];
        
        f[0] = state.getEntry(0);
        f[1] = state.getEntry(1);
        f[2] = state.getEntry(2);
        f[3] = state.getEntry(3);
        f[4] = state.getEntry(3) * Math.cos(state.getEntry(2));
        f[5] = state.getEntry(3) * Math.sin(state.getEntry(2));
        //f[6] = state[4];
        //f[7] = state[4] * Math.cos(state[2]);
        //f[8] = state[4] * Math.sin(state[2]);
        f[6] = state.getEntry(4);;
        
        return new ArrayRealVector(f);
    }
    
    public RealVector mapAction(RealVector action) {
        double[] f = new double[3];
        
        f[0] = action.getEntry(0);
        f[1] = action.getEntry(1);
        f[2] = Math.abs(action.getEntry(1));
        
        return new ArrayRealVector(f);
    }
    
    public double[] phi(double[] state) {
        double[] f = new double[6];
        
        f[0] = state[0];
        f[1] = Math.abs(state[1] / 7.5 - 1);
        f[2] = Math.abs(state[2]);
        f[3] = state[3] * Math.cos(state[2]);
        f[4] = Math.abs(state[3] * Math.sin(state[2]));
        //f[5] = Math.abs(state[4] * Math.cos(state[2]));
        //f[6] = Math.abs(state[4] * Math.sin(state[2]));
        //f[5] = Math.abs(state[4]);
        f[5] = 1;
        
        return f;
    }
    
    public Action control(SensorModel sensorModel) {
        Action action = new Action ();
        
        if (Math.abs(sensorModel.getTrackPosition()) > 1) {
        	action.restartRace = true;
        	return action;
        }
        
        /*
        if (sensorModel.getSpeed () < targetSpeed) {
            action.accelerate = 1;
        }
        if (sensorModel.getAngleToTrackAxis() < 0) {
            action.steering = -0.1;
        }
        else {
            action.steering = 0.1;
        }
        */
        
        double[] dataS = {
        		sensorModel.getDistanceFromStartLine(),
        		(sensorModel.getTrackPosition() + 1) * 7.5,
        		//sensorModel.getTrackPosition() > 0 ? sensorModel.getTrackPosition() : 0,
        		//sensorModel.getTrackPosition() < 0 ? sensorModel.getTrackPosition() * -1 : 0,
        		sensorModel.getAngleToTrackAxis(),
        		//sensorModel.getAngleToTrackAxis() > 0 ? sensorModel.getAngleToTrackAxis() : 0,
        		//sensorModel.getAngleToTrackAxis() < 0 ? sensorModel.getAngleToTrackAxis() * -1 : 0,
        		(sensorModel.getSpeed() * 1000 / 3600),
        		(sensorModel.getLateralSpeed() * 1000 / 3600),
        		sensorModel.getAngleToTrackAxis() - previousAngle
        };
        
        previousAngle = dataS[2];
        
        RealVector s = new ArrayRealVector(dataS);
        //double[] t = a.operate(mapState(s.toArray()));
        //RealVector t1 = new ArrayRealVector(t);
        
        int num_actions = actions.getRowDimension();
        
        //RealVector values = new ArrayRealVector(num_actions);
        double maxValue = Double.MIN_VALUE;
        int maxIndex = 0;
        
        // Select action which results in state with highest estimated value
        for (int i = 0; i < num_actions; i++) {
        	RealVector sPrime = s;
            for (int k = 0; k < 5;k++) {
                sPrime = a.operate(mapState(sPrime)).add(b.operate(mapAction(actions.getRowVector(i))));
            }

        	//RealVector t2 = b.operate(actions.getRowVector(i));
        	//RealVector sPrime = t1.add(t2);
        	
        	double[] data = sPrime.getData();

        	double value = theta.dotProduct(phi(data));
        	if (value > maxValue) {
        		maxIndex = i;
        		maxValue = value;
        	}
        }
        
        RealVector selectedAction = actions.getRowVector(maxIndex);
        
        double velocity = selectedAction.getEntry(0);
        double steering = selectedAction.getEntry(1);
        
        // CORRECT speed if going to fast
        if (sensorModel.getSpeed() > targetSpeed)
        	velocity = -1;
        
        action.accelerate = Math.max(velocity, 0.0);
        action.brake = Math.min(velocity, 0.0);
        action.steering = steering;
        
        gearBehaviour.execute(sensorModel, action);
        clutchBehaviour.execute(sensorModel, action);
        
        return action;
    }

    public void reset() {
		System.out.println("Restarting the race!");
	}

	public void shutdown() {
		System.out.println("Bye bye!");		
	}
}
