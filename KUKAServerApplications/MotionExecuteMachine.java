package KUKAServerApplications;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.circ;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.linRel;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import com.kuka.roboticsAPI.geometricModel.Frame;

public class MotionExecuteMachine
{
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
		
		// TODO 这里速度应该是绝对值速度才对
		KUKAServerManager.tool_.move(circ(auxiliary, end).
				setJointVelocityRel(KUKAServerManager.jRelVel_));
	}
}
