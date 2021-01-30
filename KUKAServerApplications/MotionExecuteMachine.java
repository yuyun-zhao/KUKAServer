package KUKAServerApplications;

//Copyright: Mohammad SAFEEA, 9th-April-2018

import static com.kuka.roboticsAPI.motionModel.BasicMotions.circ;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.linRel;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import com.kuka.roboticsAPI.geometricModel.Frame;

public class MotionExecuteMachine
{
	/*private static final String stopCharacter = Character.toString((char) (10));

	public static void PTPMotionJointSpaceTorquesConditional(int n, double[] indices,
															 double[] minTorque, double[] maxTorque)
	{
		if (n == 0)
		{
			// Do not perform motion
			String tempString = "done" + stopCharacter;
			//daback.sendCommand(tempString);
			return;
		}

		ICondition comb = generateTorqueCondition(n, indices, minTorque,
				maxTorque);

		double[] distPos = new double[7];
		for (int i = 0; i < 7; i++) {
			distPos[i] = KUKAServerManager.jPos[i];
		}

		KUKAServerManager._tool.getDefaultMotionFrame().move(
				ptp(distPos[0], distPos[1], distPos[2], distPos[3], distPos[4],
						distPos[5], distPos[6]).setJointVelocityRel(
						KUKAServerManager.jRelVel).breakWhen(comb));

		boolean interruptionFlag = false;
		for (int i = 0; i < 7; i++) {
			double x1 = _lbr_Med.getCurrentJointPosition().get(i);
			double x2 = distPos[i];
			double clearance = 0.1 * Math.PI / 180;
			if (Math.abs(x1 - x2) > clearance) {
				interruptionFlag = true;
				break;
			}
		}
		// Return back the acknowledgment message
		String tempString;
		if (interruptionFlag == true) {
			tempString = "interrupted" + stopCharacter;
		} else {
			tempString = "done" + stopCharacter;
		}

		daback.sendCommand(tempString);

	}

	public static void PTPMotionLineCartesianSpaceTorquesConditional(int n,
																	 double[] indices, double[] minTorque, double[] maxTorque) {

		int num = n;
		if (num == 0) {
			// Do not perform motion
			String tempString = "done" + stopCharacter;
			daback.sendCommand(tempString);
			return;
		}

		ICondition comb = generateTorqueCondition(n, indices, minTorque,
				maxTorque);

		double[] distPos = new double[6];
		for (int i = 0; i < 6; i++) {
			distPos[i] = KUKAServerManager.EEFPos[i];
		}

		Frame daframe = _lbr_Med
				.getCurrentCartesianPosition(KUKAServerManager._tool
						.getDefaultMotionFrame());
		int kkk = 0;
		daframe.setX(distPos[kkk]);
		kkk = kkk + 1;
		daframe.setY(distPos[kkk]);
		kkk = kkk + 1;
		daframe.setZ(distPos[kkk]);
		kkk = kkk + 1;
		daframe.setAlphaRad(distPos[kkk]);
		kkk = kkk + 1;
		daframe.setBetaRad(distPos[kkk]);
		kkk = kkk + 1;
		daframe.setGammaRad(distPos[kkk]);

		IMotionContainer x = KUKAServerManager._tool
				.getDefaultMotionFrame().move(
						lin(daframe).setCartVelocity(
								KUKAServerManager.jRelVel).breakWhen(comb));

		try {
			Thread.sleep(20);
		} catch (InterruptedException e) {
			// TODO Bloco catch gerado automaticamente
			e.printStackTrace();
		}
		double[] reachedPos = new double[6];
		Frame reachedFrame = _lbr_Med
				.getCurrentCartesianPosition(KUKAServerManager._tool
						.getDefaultMotionFrame());
		kkk = 0;
		reachedPos[kkk] = reachedFrame.getX();
		kkk = kkk + 1;
		reachedPos[kkk] = reachedFrame.getY();
		kkk = kkk + 1;
		reachedPos[kkk] = reachedFrame.getZ();
		kkk = kkk + 1;
		reachedPos[kkk] = reachedFrame.getAlphaRad();
		kkk = kkk + 1;
		reachedPos[kkk] = reachedFrame.getBetaRad();
		kkk = kkk + 1;
		reachedPos[kkk] = reachedFrame.getGammaRad();

		boolean interruptionFlag = false;
		if (x.isFinished() == false) {
			interruptionFlag = true;
		}
		for (int i = 0; i < 3; i++) {
			double x1 = reachedPos[i];
			double x2 = distPos[i];
			double clearance = 5 * Math.PI / 180; // angular clearance (rad)
			if (i < 3) {
				clearance = 0.2; // linear clearance (mm)
			}

			if (Math.abs(x1 - x2) > clearance) {
				interruptionFlag = true;
				break;
			}
		}

		// Return back the acknowledgment message
		String tempString;
		if (interruptionFlag == true) {
			tempString = "interrupted" + stopCharacter;
		} else {
			tempString = "done" + stopCharacter;
		}

		daback.sendCommand(tempString);

	}

	public static void PTPmotionCartizianSpaceCircleTorquesConditional(int n,
			double[] indices, double[] minTorque, double[] maxTorque) {

		int num = n;
		if (num == 0) {
			// Do not perform motion
			String tempString = "done" + stopCharacter;
			daback.sendCommand(tempString);
			return;
		}

		ICondition comb = generateTorqueCondition(n, indices, minTorque,
				maxTorque);

		Frame daframe1 = _lbr_Med
				.getCurrentCartesianPosition(KUKAServerManager._tool
						.getDefaultMotionFrame());
		Frame daframe2 = _lbr_Med
				.getCurrentCartesianPosition(KUKAServerManager._tool
						.getDefaultMotionFrame());
		int kkk = 0;
		daframe1.setX(KUKAServerManager.EEFPosCirc1[kkk]);
		kkk = kkk + 1;
		daframe1.setY(KUKAServerManager.EEFPosCirc1[kkk]);
		kkk = kkk + 1;
		daframe1.setZ(KUKAServerManager.EEFPosCirc1[kkk]);
		kkk = kkk + 1;
		daframe1.setAlphaRad(KUKAServerManager.EEFPosCirc1[kkk]);
		kkk = kkk + 1;
		daframe1.setBetaRad(KUKAServerManager.EEFPosCirc1[kkk]);
		kkk = kkk + 1;
		daframe1.setGammaRad(KUKAServerManager.EEFPosCirc1[kkk]);

		kkk = 0;
		daframe2.setX(KUKAServerManager.EEFPosCirc2[kkk]);
		kkk = kkk + 1;
		daframe2.setY(KUKAServerManager.EEFPosCirc2[kkk]);
		kkk = kkk + 1;
		daframe2.setZ(KUKAServerManager.EEFPosCirc2[kkk]);
		kkk = kkk + 1;
		daframe2.setAlphaRad(KUKAServerManager.EEFPosCirc2[kkk]);3
		kkk = kkk + 1;
		daframe2.setGammaRad(KUKAServerManager.EEFPosCirc2[kkk]);

		double[] distPos = new double[6];
		for (int i = 0; i < 6; i++) {
			distPos[i] = KUKAServerManager.EEFPosCirc2[i];
		}

		KUKAServerManager._tool.getDefaultMotionFrame().move(
				circ(daframe1, daframe2).setCartVelocity(
						KUKAServerManager.jRelVel).breakWhen(comb));

		double[] reachedPos = new double[6];
		Frame reachedFrame = _lbr_Med
				.getCurrentCartesianPosition(KUKAServerManager._tool
						.getDefaultMotionFrame());
		kkk = 0;
		reachedPos[kkk] = reachedFrame.getX();
		kkk = kkk + 1;
		reachedPos[kkk] = reachedFrame.getY();
		kkk = kkk + 1;
		reachedPos[kkk] = reachedFrame.getZ();
		kkk = kkk + 1;
		reachedPos[kkk] = reachedFrame.getAlphaRad();
		kkk = kkk + 1;
		reachedPos[kkk] = reachedFrame.getBetaRad();
		kkk = kkk + 1;
		reachedPos[kkk] = reachedFrame.getGammaRad();

		boolean interruptionFlag = false;
		for (int i = 0; i < 6; i++) {
			double x1 = reachedPos[i];
			double x2 = distPos[i];
			double clearance = 0.1 * Math.PI / 180; // angular clearance (rad)
			if (i > 2) {
				clearance = 0.05; // linear clearance (mm)
			}

			if (Math.abs(x1 - x2) > clearance) {
				interruptionFlag = true;
				break;
			}
		}
		// Return back the acknowledgment message
		String tempString;
		if (interruptionFlag == true) {
			tempString = "interrupted" + stopCharacter;
		} else {
			tempString = "done" + stopCharacter;
		}

		daback.sendCommand(tempString);

	}

	public static void PTPmotionCartizianSpaceRelWorldTorquesConditional(int n,
			double[] indices, double[] minTorque, double[] maxTorque) {

		int num = n;
		if (num == 0) {
			// Do not perform motion
			String tempString = "done" + stopCharacter;
			daback.sendCommand(tempString);
			return;
		}

		ICondition comb = generateTorqueCondition(n, indices, minTorque,
				maxTorque);

		Frame daframe = _lbr_Med
				.getCurrentCartesianPosition(KUKAServerManager._tool
						.getDefaultMotionFrame());

		double[] distPos = new double[3];
		distPos[0] = KUKAServerManager.EEFPos[0] + daframe.getX();
		distPos[1] = KUKAServerManager.EEFPos[1] + daframe.getY();
		distPos[2] = KUKAServerManager.EEFPos[2] + daframe.getZ();

		daframe.setX(distPos[0]);

		daframe.setY(distPos[1]);

		daframe.setZ(distPos[2]);

		KUKAServerManager._tool.getDefaultMotionFrame().move(
				lin(daframe).setCartVelocity(KUKAServerManager.jRelVel)
						.breakWhen(comb));

		double[] reachedPos = new double[3];
		Frame reachedFrame = _lbr_Med
				.getCurrentCartesianPosition(KUKAServerManager._tool
						.getDefaultMotionFrame());
		int kkk = 0;
		reachedPos[kkk] = reachedFrame.getX();
		kkk = kkk + 1;
		reachedPos[kkk] = reachedFrame.getY();
		kkk = kkk + 1;
		reachedPos[kkk] = reachedFrame.getZ();

		boolean interruptionFlag = false;
		for (int i = 0; i < 3; i++) {
			double x1 = reachedPos[i];
			double x2 = distPos[i];
			double clearance = 0.05; // linear clearance (mm)
			if (Math.abs(x1 - x2) > clearance) {
				interruptionFlag = true;
				break;
			}
		}
		// Return back the acknowledgment message
		String tempString;
		if (interruptionFlag == true) {
			tempString = "interrupted" + stopCharacter;
		} else {
			tempString = "done" + stopCharacter;
		}

		daback.sendCommand(tempString);

	}

	public static void PTPmotionCartizianSpaceRelEEfTorquesConditional(int n,
			double[] indices, double[] minTorque, double[] maxTorque) {

		double x, y, z;
		x = KUKAServerManager.EEFPos[0];

		y = KUKAServerManager.EEFPos[1];

		z = KUKAServerManager.EEFPos[2];

		KUKAServerManager._tool.getDefaultMotionFrame().move(
				linRel(x, y, z).setCartVelocity(KUKAServerManager.jRelVel));

		String tempString = "done" + stopCharacter;
		daback.sendCommand(tempString);

	}

	//Get condition functions ***********************************
	
	private static ICondition generateTorqueCondition(int n, double[] indices,
			double[] minTorque, double[] maxTorque)
	{
		JointTorqueCondition[] con = new JointTorqueCondition[7];
		JointEnum[] jointNum = new JointEnum[7];
		jointNum[0] = JointEnum.J1;
		jointNum[1] = JointEnum.J2;
		jointNum[2] = JointEnum.J3;
		jointNum[3] = JointEnum.J4;
		jointNum[4] = JointEnum.J5;
		jointNum[5] = JointEnum.J6;
		jointNum[6] = JointEnum.J7;

		for (int i = 0; i < n; i++)
		{
			int index = (int) indices[i];
			con[i] = new JointTorqueCondition(jointNum[index], minTorque[i],
					maxTorque[i]);
		}

		ICondition comb = con[0];

		for (int i = 1; i < n; i++) {
			comb = comb.or(con[i]);
		}

		return comb;

	}*/

	public static void PTPMotionJointSpace()
	{
		// 移动 "工具默认坐标系的原点" 的位姿
		KUKAServerManager.tool_.move(
				ptp(KUKAServerManager.jPos_[0],
						KUKAServerManager.jPos_[1],
						KUKAServerManager.jPos_[2],
						KUKAServerManager.jPos_[3],
						KUKAServerManager.jPos_[4],
						KUKAServerManager.jPos_[5],
						KUKAServerManager.jPos_[6]).
						setJointVelocityRel(KUKAServerManager.jRelVel_));
	}

	public static void PTPMotionCartSpace()
	{
		Frame destination = KUKAServerManager.LBR_Med_.getCurrentCartesianPosition(
				KUKAServerManager.tool_.getDefaultMotionFrame());

		destination.setX(KUKAServerManager.EEFPos_[0]);
		destination.setY(KUKAServerManager.EEFPos_[1]);
		destination.setZ(KUKAServerManager.EEFPos_[2]);
		destination.setAlphaRad(KUKAServerManager.EEFPos_[3]);
		destination.setBetaRad(KUKAServerManager.EEFPos_[4]);
		destination.setGammaRad(KUKAServerManager.EEFPos_[5]);

		// 移动 "工具默认坐标系的原点" 的位姿
		KUKAServerManager.tool_.move(ptp(destination).
				setJointVelocityRel(KUKAServerManager.jRelVel_));
	}

	// TODO 是否需要区分绝对值位置与相对值位置
	public static void LINMotion()
	{
		Frame destination = KUKAServerManager.LBR_Med_.getCurrentCartesianPosition(
				KUKAServerManager.tool_.getDefaultMotionFrame());

		destination.setX(KUKAServerManager.EEFPos_[0]);
		destination.setY(KUKAServerManager.EEFPos_[1]);
		destination.setZ(KUKAServerManager.EEFPos_[2]);
		destination.setAlphaRad(KUKAServerManager.EEFPos_[3]);
		destination.setBetaRad(KUKAServerManager.EEFPos_[4]);
		destination.setGammaRad(KUKAServerManager.EEFPos_[5]);

		// TODO 这里速度应该是绝对值速度才对
		KUKAServerManager.tool_.move(lin(destination).
				setCartVelocity(KUKAServerManager.cVel_));
	}

	public static void LINMotionRelBase()
	{
		// 获得当前的工具坐标系的坐标值，在此基础上移动增量offset
		Frame destination = KUKAServerManager.LBR_Med_.getCurrentCartesianPosition(
				KUKAServerManager.tool_.getDefaultMotionFrame());

		// TODO EEFPos此时代表相对当前位置的增量offset，是否需要区分offset和pos
		destination.setX(KUKAServerManager.EEFOffset_[0] + destination.getX());
		destination.setY(KUKAServerManager.EEFOffset_[1] + destination.getY());
		destination.setZ(KUKAServerManager.EEFOffset_[2] + destination.getZ());

		// 移动的方向是由Base坐标系决定的
		KUKAServerManager.tool_.move(lin(destination).
				setCartVelocity(KUKAServerManager.cVel_));
	}

	public static void LINMotionRelEEF()
	{
		// EEFPos此时代表相对当前位置的增量offset
		double x, y, z;
		x = KUKAServerManager.EEFOffset_[0];
		y = KUKAServerManager.EEFOffset_[1];
		z = KUKAServerManager.EEFOffset_[2];

		// linRel: 相对于当前位置移动Offset(x,y,z)
		// 移动的方向是由TOOL坐标系决定的
		KUKAServerManager.tool_.move(linRel(x, y, z).
				setCartVelocity(KUKAServerManager.cVel_));
	}

	public static void CIRCMotion()
	{
		Frame auxiliary = KUKAServerManager.LBR_Med_.getCurrentCartesianPosition(
				KUKAServerManager.tool_.getDefaultMotionFrame());
		Frame end = KUKAServerManager.LBR_Med_.getCurrentCartesianPosition(
				KUKAServerManager.tool_.getDefaultMotionFrame());

		auxiliary.setX(KUKAServerManager.EEFPosCIRC1_[0]);
		auxiliary.setY(KUKAServerManager.EEFPosCIRC1_[1]);
		auxiliary.setZ(KUKAServerManager.EEFPosCIRC1_[2]);
		auxiliary.setAlphaRad(KUKAServerManager.EEFPosCIRC1_[3]);
		auxiliary.setBetaRad(KUKAServerManager.EEFPosCIRC1_[4]);
		auxiliary.setGammaRad(KUKAServerManager.EEFPosCIRC1_[5]);

		end.setX(KUKAServerManager.EEFPosCIRC2_[0]);
		end.setY(KUKAServerManager.EEFPosCIRC2_[1]);
		end.setZ(KUKAServerManager.EEFPosCIRC2_[2]);
		end.setAlphaRad(KUKAServerManager.EEFPosCIRC2_[3]);
		end.setBetaRad(KUKAServerManager.EEFPosCIRC2_[4]);
		end.setGammaRad(KUKAServerManager.EEFPosCIRC2_[5]);
		
//		Frame cFrame = KUKAServerManager.LBR_Med_.getCurrentCartesianPosition(
//				KUKAServerManager.tool_.getDefaultMotionFrame());	
//		cFrame.setX(610);cFrame.setY(200);cFrame.setZ(400);
//		cFrame.setAlphaRad(Math.PI);cFrame.setBetaRad(0);cFrame.setGammaRad(Math.PI);		
//		KUKAServerManager.LBR_Med_.move(lin(cFrame).setJointVelocityRel(0.1));
//		Frame bFrame = KUKAServerManager.LBR_Med_.getCurrentCartesianPosition(
//				KUKAServerManager.tool_.getDefaultMotionFrame());		
//		bFrame.setX(610);bFrame.setY(0);bFrame.setZ(400);bFrame.setAlphaRad(Math.PI);
//		bFrame.setBetaRad(0);bFrame.setGammaRad(Math.PI);	
//		Frame aFrame = KUKAServerManager.LBR_Med_.getCurrentCartesianPosition(
//				KUKAServerManager.tool_.getDefaultMotionFrame());	
//		aFrame.setX(610);aFrame.setY(0);aFrame.setZ(600);aFrame.setAlphaRad(Math.PI);
//		aFrame.setBetaRad(0);aFrame.setGammaRad(Math.PI);
//		KUKAServerManager.LBR_Med_.move(circ(bFrame,aFrame).setJointVelocityRel(0.1));

		// TODO 这里速度应该是绝对值速度才对
		KUKAServerManager.tool_.move(circ(auxiliary, end).
				setJointVelocityRel(KUKAServerManager.jRelVel_));
	}
}
