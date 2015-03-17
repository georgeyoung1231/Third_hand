package kinematics;

import kinematics.InverseKinematics;
import kinematics.ForwardKinematics;

public class KinematicsTest {

	int motor0, motor1, motor2, motor3, motor4;
	double[] degreesMotor;
	double[] currentDegreesMotor;
	double[] destinationPosRot;
	double[] currentPosRot;
	double[] relativePosRot;
	double[] vectortoCurrentPos;
	double[] vectortoCurrentWrist;
	double[] currentWrist;
	
	InverseKinematics ik = new InverseKinematics();
	ForwardKinematics fk = new ForwardKinematics();
	
	public double[] testIK(double[] testIK)
	{
		degreesMotor = ik.ik(testIK);
		System.out.println("This is test IK:");
		System.out.println(degreesMotor[0] +"  " +degreesMotor[1] +"  " +degreesMotor[2] +"  " +degreesMotor[3] +"  " +degreesMotor[4] );
		return degreesMotor;
	}
	
	public double[] testFK(double[] testFK)
	{
		currentWrist = fk.getVectorWrist(testFK);
		System.out.println("This is test FK:");
		System.out.println(currentWrist[0] +"  " +currentWrist[1] +"  " +currentWrist[2] +"  " +currentWrist[3] +"  " +currentWrist[4]+"  " +currentWrist[5] );
		return currentWrist;
	}

}
