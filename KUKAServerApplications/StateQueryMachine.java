package KUKAServerApplications;

//Copyright: Mohammad SAFEEA, 9th-April-2018

import com.kuka.med.deviceModel.LBRMed;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.roboticsAPI.sensorModel.TorqueSensorData;

// 该类作用：获取机器人当前坐标、力矩、力等信息
public class StateQueryMachine
{
	private final LBRMed LBR_Med_;
	private final String stopCharacter = Character.toString((char)(10));//‘/n’

	StateQueryMachine(LBRMed lbr_Med)
	{
		LBR_Med_ = lbr_Med;
	}

	// This functions sends the measured torques of joints to the client
	public String queryJointMeasuredTorques()
	{
		//getMeasuredTorque()返回所有力作用的力矩
		TorqueSensorData measuredData = LBR_Med_.getMeasuredTorque();
		double[] vals = measuredData.getTorqueValues();

		StringBuilder s = new StringBuilder();
		for (int i = 0; i < vals.length; i++) {
			s.append(Double.toString(vals[i])).append("_");
		}
		s.append(stopCharacter);

		return s.toString();
	}

	// This functions sends the external torques of joints to the client
	public String queryJointExternalTorques()
	{
		//getExternalTorque()只返回外部力作用的力矩
		TorqueSensorData measuredData = LBR_Med_.getExternalTorque();
		double[] vals = measuredData.getTorqueValues();

		StringBuilder s = new StringBuilder();
		for (int i = 0; i < vals.length; i++) {
			s.append(Double.toString(vals[i])).append("_");
		}
		s.append(stopCharacter);

		return s.toString();
	}

	// This functions sends the forces of end effector to the client
	public String queryEEFforces()
	{
		ForceSensorData cforce = LBR_Med_.getExternalForceTorque(
				KUKAServerManager.tool_.getDefaultMotionFrame());

		String cmdforce = Double.toString(cforce.getForce().getX()) + "_"
				+ Double.toString(cforce.getForce().getY()) + "_"
				+ Double.toString(cforce.getForce().getZ()) + "_"
				+ stopCharacter;

		return cmdforce;
	}

	// This functions sends the moments of end effector to the client
	public String queryEEFMoments()
	{
		ForceSensorData cforce = LBR_Med_.getExternalForceTorque(
				KUKAServerManager.tool_.getDefaultMotionFrame());

		String cmdforce = Double.toString(cforce.getTorque().getX()) + "_"
				+ Double.toString(cforce.getTorque().getY()) + "_"
				+ Double.toString(cforce.getTorque().getZ()) + "_"
				+ stopCharacter;

		return cmdforce;
	}

	// This functions sends the positions of joints to the client
	public String queryJointPosition()
	{
		JointPosition jointPosition = new JointPosition(
				LBR_Med_.getCurrentJointPosition());

		StringBuilder s = new StringBuilder();
		for (int i = 0; i < KUKAServerManager.jNum_; i++)
		{
			s.append(Double.toString(jointPosition.get(i))).append("_");
		}
		s.append(stopCharacter);

		return s.toString();
	}

	// This functions sends the positions of end effector to the client
	public String queryEEFPosition() {
		String cmdPos = "";
		// Read Cartesian position data

		Frame EEFPos = LBR_Med_.getCurrentCartesianPosition(
				KUKAServerManager.tool_.getDefaultMotionFrame());
		cmdPos = Double.toString(EEFPos.getX()) + "_"
				+ Double.toString(EEFPos.getY()) + "_"
				+ Double.toString(EEFPos.getZ()) + "_"
				+ Double.toString(EEFPos.getAlphaRad()) + "_"
				+ Double.toString(EEFPos.getBetaRad()) + "_"
				+ Double.toString(EEFPos.getGammaRad()) + "_" + stopCharacter;

		return cmdPos;
	}
}
