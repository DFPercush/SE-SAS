using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;
using System.Text;
using VRage;
using VRage.Collections;
using VRage.Game;
using VRage.Game.Components;
using VRage.Game.GUI.TextPanel;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.Game.ObjectBuilders.Definitions;
using VRageMath;

/*
TODO:
	- Use y as normal coord when starting orbit calc
    - Handle 0 inclination, Om and w are NaN
	- Watch cockpit indicator for roll and set overrides
*/

namespace IngameScript
{
	public partial class Program : MyGridProgram
	{

		#region MDK Preserve
		// ==== CONFIG ====

		// This is probably the only thing you'll need to change
		const string ControlBlock = "Cockpit";

		readonly Vector3D cockpitLocalForward = new Vector3D (0, 0, -1);
		readonly Vector3D cockpitLocalUp = new Vector3D (0, 1, 0);
		readonly Vector3D cockpitLocalRight = new Vector3D (1, 0, 0);
		const double BaseGyroSensitivity = 0.02;
		const double TorquePerGyroMultiplier = 60000;

		const double MANEUVER_SENSITIVITY = 1;
		const double ORBIT_PRECISION = 500.0; // How far off can the altitude be
		const double THROTTLE_BACK_TIME = 2.0; // Seconds before burn completes to start fine adjustment
		const double MINIMUM_THRUST = 0.01; // Must be > 0, in the range (0, 1]
		const double ROLL_SPEED = 6.0;

		// ==== END CONFIG ====
		#endregion

		static Program prg;
		const double deg2rad = Math.PI / 180.0;
		const double rad2deg = 180.0 / Math.PI;
		enum Mode
		{
			Disabled,
			Prograde,
			Retrograde,
			Normal,
			Antinormal,
			RadialIn,
			RadialOut,
			DisplayOnly
		};
		Mode mode;
		IMyGridTerminalSystem G;
		IMyShipController cockpit;
		List<IMyGyro> gyros = new List<IMyGyro>();
		List<GyroTranslator> gyt = new List<GyroTranslator>();
		PID pidYaw; /* = new PID
		{
			Kp = BaseGyroSensitivity,
			Ki = BaseGyroSensitivity * 0.5,
			Kd = 0, //BaseGyroSensitivity * 0.5,
			limMin = -BaseGyroSensitivity * 10,
			limMax = BaseGyroSensitivity * 10,
			limMinInt = -BaseGyroSensitivity,
			limMaxInt = BaseGyroSensitivity,
			lowPass = 1.0,
			dt = 1.0 / 60,
			setpoint = 0
		};*/
		PID pidPitch; /* = new PID
		{
			Kp = BaseGyroSensitivity,
			Ki = BaseGyroSensitivity * 0.5,
			Kd = 0, //BaseGyroSensitivity * 0.5,
			limMin = -BaseGyroSensitivity * 10,
			limMax = BaseGyroSensitivity * 10,
			limMinInt = -BaseGyroSensitivity,
			limMaxInt = BaseGyroSensitivity,
			lowPass = 1.0,
			dt = 1.0 / 60,
			setpoint = 0
		};*/

		double TorqueToMassRatio = 1.0;
		Orbit orb = new Orbit(
			new Vector3D(1,0,0), // equinox
			new Vector3D(0,0,1), // east
			new Vector3D(0,-1,0) // north
			);
		bool keep;
		double keepAp;
		double keepPe;
		enum ManeuverStatus
		{
			IDLE,
			PLANNED,
			RUNNING
		};
		ManeuverStatus mstat;
		//DateTime mstart;
		//double mTargetSpeed;
		delegate double getd();
		static double get0() { return 0; }
		getd mGetDelay = get0;
		getd mGetTargetVel = get0;
		double mTargetVel;
		//double mTargetAp;
		//double mTargetPe;
		double prevFrameSpeed;
		uint mFrameStartedBurn;
		Base6Directions.Direction mThrustDir = Base6Directions.Direction.Backward;

		List<IMyShipController> shipControllers = new List<IMyShipController>();
		public Program()
		{
			prg = this;
			G = GridTerminalSystem;
			cockpit = G.GetBlockWithName(ControlBlock) as IMyShipController;
			if (cockpit == null)
			{
				G.GetBlocksOfType(shipControllers);
				foreach (var sc in shipControllers)
				{
					if (sc.IsMainCockpit)
					{
						cockpit = sc;
						break;
					}
				}
				if (cockpit == null)
				{
					Echo("Error: No ship control block found.");
					return;
				}
			}
			FindGyros(cockpit);
			if (gyros.Count == 0)
			{
				Echo("Error: No gyros.");
				return;
			}
			TorqueToMassRatio = 0;
			foreach (var g in gyros)
			{
				TorqueToMassRatio += g.GyroPower;
			}
			TorqueToMassRatio /= cockpit.CalculateShipMass().TotalMass;
			double K = BaseGyroSensitivity * TorqueToMassRatio * TorquePerGyroMultiplier;
			pidYaw = new PID
			{
				Kp = K,
				Ki = K * 0.05,
				Kd = 0, //K * 0.5,
				limMin = -K * 10,
				limMax = K * 10,
				limMinInt = -K,
				limMaxInt = K,
				lowPass = 1.0,
				dt = 1.0 / 60,
				setpoint = 0
			};
			pidPitch = new PID
			{
				Kp = K,
				Ki = K * 0.5,
				Kd = 0, //K * 0.5,
				limMin = -K * 10,
				limMax = K * 10,
				limMinInt = -K,
				limMaxInt = K,
				lowPass = 1.0,
				dt = 1.0 / 60,
				setpoint = 0
			};

			var lines = Storage.Split('\n');
			foreach (var line in lines)
			{
				var sp = line.Split('=');
				if (sp.Length != 2) { continue; }
				switch (sp[0])
				{
					//Storage = $"mode={mode}\nkeep={keep}\nkeepApo={keepApo}\nkeepPeri={keepPeri}";
					case "mode": mode = (Mode)Enum.Parse(typeof(Mode), sp[1]); break;
					case "keep": keep = bool.Parse(sp[1]); break;
					case "keepApo": keepAp = double.Parse(sp[1]); break;
					case "keepPeri": keepPe = double.Parse(sp[1]); break;
					default: Echo($"Warning: Unknown data saved in storage: {line}"); break;
				}
			}
			if (mode != Mode.Disabled) { Runtime.UpdateFrequency = UpdateFrequency.Update1; }
			else { Runtime.UpdateFrequency = UpdateFrequency.None; }
		} // constructor

		public void Save()
		{
			Storage = $"mode={mode}\nkeep={keep}\nkeepApo={keepAp}\nkeepPeri={keepPe}";
		} // Save

		//public void ScheduleProgradeManeuver(DateTime when, double targetVel)
		//{
		//	mstart = when;
		//	mTargetSpeed = targetVel;
		//	mstat = ManeuverStatus.PLANNED;
		//}

		public void Main(string arg, UpdateType updateSource)
		{
			frameNum++;
			bool planetPresent = false;
			if (updateSource == UpdateType.Update1)
			{
				UpdateOrbit();
				//if (!validOrbit) { return; }
				Vector3D pp;
				planetPresent = cockpit.TryGetPlanetPosition(out pp);
				if (!planetPresent) // TODO: Consolidate with validOrbit
				{
					// Not around a planet
					Echo("No planet.");
					if (mode == Mode.RadialIn || mode == Mode.RadialOut
						|| mode == Mode.Normal || mode == Mode.Antinormal)
					{
						Echo($"{mode} not available.");
						return;
					}
				}
				if (planetPresent)
				{
					// mu = GM
					// We don't know the exact mass of the planet (or Space Engineers gravitational constant),
					// but we can calculate what we need based on current (natural) gravity reading.
					// accel = GM / r^2
					// GM = accel * r^2 = mu
					Vector3D vel = cockpit.GetShipVelocities().LinearVelocity;
					// position relative to planet
					Vector3D posRtPlanet = cockpit.GetPosition() - pp;
					double r2 = posRtPlanet.LengthSquared();
					double r = Math.Sqrt(r2);
					// Note: mu should be constant everywhere in orbit of the same planet. TODO: Test that.
					double mu = cockpit.GetNaturalGravity().Length() * r2;
					//const double GravConst = 6.674e-11; // TODO: Move up
					orb.UpdateFromPosVel(posRtPlanet, vel, mu);

					Echo($"r = {r:F0}");
					//Echo($"mu: {mu}");
					//Echo($"SMA: {orb.a}");
					//Echo($"Ecc: {orb.e}");
					Echo($"Ap: {orb.Ap:F0} ({timestr(orb.TimeToAp())})");
					Echo($"Pe: {orb.Pe:F0} ({timestr(orb.TimeToPe())})");
					Echo($"Inc: {orb.i*rad2deg:F2}");
					// TODO: Ascending / descending node

					//Echo($"TA: {orb.v}");
					//Echo($"EA: {orb.Ea}");
					//Echo($"MA: {orb.M}");
					//Echo($"nMeanMotion: {orb.nMeanMotion}");
					Echo($"Period: {timestr(orb.T)}");
					//Echo($"Time since Pe:{orb.tpe:F2}");
					Echo($"Maneuver {mstat}");
					if (mstat == ManeuverStatus.PLANNED) { Echo($"Burn in {timestr(mGetDelay())}"); }
					if (mstat == ManeuverStatus.RUNNING) { Echo($"Vel {cockpit.GetShipSpeed():F0} / {mGetTargetVel():F0}"); }
					//Echo($"to Ap:")

					// TODO:
					//cockpit.MoveIndicator
					//cockpit.RollIndicator
					//cockpit.RotationIndicator


				}
				//Vector3D planetMe = cockpit.WorldMatrix.Translation - pp;
				Vector3D planetMe = cockpit.CenterOfMass - pp;
				Vector3D wvFwd = mul(cockpit.WorldMatrix.GetOrientation(), cockpitLocalForward);
				Vector3D vPro = cockpit.GetShipVelocities().LinearVelocity; vPro.Normalize();
				Vector3D vNorm = planetMe.Cross(vPro); vNorm.Normalize();
				Vector3D vOut = vPro.Cross(vNorm); vOut.Normalize();
				QuaternionD qDest = new QuaternionD();
				Vector3D vDest = new Vector3D();
				switch (mode)
				{
					case Mode.Disabled:
						Stop();
						break;
					case Mode.Prograde:
						vDest = vPro;
						asn(ref qDest, vPro);
						break;
					case Mode.Retrograde: vDest = -vPro; asn(ref qDest, -vPro); break;
					case Mode.Normal: vDest = vNorm; asn(ref qDest, vNorm); break;
					case Mode.Antinormal: vDest = -vNorm; asn(ref qDest, -vNorm); break;
					case Mode.RadialIn: vDest = -vOut; asn(ref qDest, -vOut); break;
					case Mode.RadialOut: vDest = vOut; asn(ref qDest, vOut); break;
					default: vDest = wvFwd; break;
				}
				Echo($"Holding {mode} {Spinner()}");
				// wv = world vector
				Vector3 wvUp = mul(cockpit.WorldMatrix.GetOrientation(), cockpitLocalUp);
				Vector3 wvRight = mul(cockpit.WorldMatrix.GetOrientation(), cockpitLocalRight);
				QuaternionD qFacing = v2q(wvFwd);
				QuaternionD qStep = QuaternionD.Slerp(qFacing, qDest, 0.1);
				Vector3D vStep = q2v(qStep); vStep.Normalize(); // vStep.Normalize();
				Vector3D vToward = vStep - wvFwd;
				//Vector3D vToward = wvFacing - vStep;
				double vTowardMag = vToward.Normalize();


				//Vector3 localToward = div(vToward, cockpit.WorldMatrix.GetOrientation());
				//double dYaw = localToward.Dot(cockpitLocalRight);
				//double dPitch = localToward.Dot(cockpitLocalUp);

				double dYaw = vToward.Dot(wvRight);
				double dPitch = vToward.Dot(wvUp);
				
				
				double angleDiff = AngleBetweenDeg(wvFwd, vDest);
				if (double.IsNaN(angleDiff)) { angleDiff = 0; }
				//pidYaw.Update((angleDiff - 90) * dYaw);
				//pidPitch.Update((angleDiff - 90) * dPitch);
				pidYaw.Update(Math.Sign(angleDiff - 90) * angleDiff * dYaw);
				pidPitch.Update(Math.Sign(angleDiff - 90) * angleDiff * dPitch);
				if (mode != Mode.DisplayOnly && mode != Mode.Disabled)
				{
					foreach (var g in gyt)
					{
						g.g.GyroOverride = true;
						g.setYaw(g.g, pidYaw.Output);
						g.setPitch(g.g, pidPitch.Output);
						//g.setYaw(g.g, angleDiff * dYaw * BaseGyroSensitivity);
						//g.setPitch(g.g, angleDiff * dPitch * BaseGyroSensitivity);

						g.setRoll(g.g, cockpit.RollIndicator * ROLL_SPEED);
					}
				}

				//double vTarget;
				//bool thrusting = false;

				if (validOrbit)
				{
					if (keep) { Echo($"Keep: {keepAp:F0} - {keepPe:F0} +/- {ORBIT_PRECISION:F0}"); }

					if (mstat == ManeuverStatus.RUNNING)
					{
						double dv = (cockpit.GetShipSpeed() - prevFrameSpeed) * 60;
						//Echo($"dv = {dv:F6}");
						double dTarget = mTargetVel - cockpit.GetShipSpeed();
						//Echo($"dTarget = {dTarget:F6}");
						Echo($"Vel {cockpit.GetShipSpeed():F0} -> {mTargetVel}");
						// 724 - 901 = -180 ish
						UpdateThrust();
						//Echo($"Thrusters: {dirThrust[mThrustDir].Count} {mThrustDir}");
						//foreach (var t in allThrust)
						//{
						//	if (t.ThrustOverride > 0)
						//	{
						//		Echo($"{t.Name}: {t.ThrustOverride:F2} {Base6Directions.GetDirection(t.GridThrustDirection)}");
						//	}
						//}
						// On the first frame we don't know what previous velocity would be doing. Give it some warm up time.
						if ((mThrustDir == Base6Directions.Direction.Forward && dTarget > 0) ||
							(mThrustDir == Base6Directions.Direction.Backward && dTarget < 0))
						{
							SetThrust(mThrustDir, (dTarget / dv / THROTTLE_BACK_TIME) + MINIMUM_THRUST);
						}
						//if ((frameNum - mFrameStartedBurn >= 2) && (dv * dTarget < 0))
						else
						{
							StopThrust();
							mstat = ManeuverStatus.IDLE;
						}
					}
					else if (mstat == ManeuverStatus.PLANNED)
					{
						//UpdateThrust();
						//if (DateTime.Now >= mstart)
						mGetTargetVel(); // TODO: Debug remove
						if (mGetDelay() < THROTTLE_BACK_TIME)
						{
							//mThrustDir = (mTargetSpeed > cockpit.GetShipSpeed()) ? Base6Directions.Direction.Backward : Base6Directions.Direction.Forward;
							double targetSpeed = mGetTargetVel();
							mThrustDir = (targetSpeed > cockpit.GetShipSpeed()) ? Base6Directions.Direction.Forward : Base6Directions.Direction.Backward;
							//UpdateThrust();
							//Base6Directions.GetOppositeDirection
							//SetThrust(mThrustDir, 1);
							mTargetVel = mGetTargetVel();
							mstat = ManeuverStatus.RUNNING;
							mFrameStartedBurn = frameNum;
						}
						//Echo($"Thrusters: {dirThrust[Base6Directions.Direction.Forward].Count} fwd {dirThrust[Base6Directions.Direction.Backward].Count} back");
					}
					else if (keep)
					{
						UpdateOrbit();
						double ttap = orb.TimeToAp();
						double ttpe = orb.TimeToPe();
						double speed = cockpit.GetShipSpeed();
						//double vPredict;
						//Echo($"time to Ap: {ttap:F2}");
						//Echo($"time to Pe: {ttpe:F2}");
						if ((orb.Ap < keepAp - ORBIT_PRECISION && orb.Pe < keepPe - ORBIT_PRECISION)
							//|| (orb.Ap > keepAp + ORBIT_PRECISION && orb.Pe > keepPe + ORBIT_PRECISION)
							)
						{
							mGetDelay = () => 0;
							//mGetTargetAp = () => keepAp;
							//mGetTargetPe = () => keepPe;
							mGetTargetVel = () => Orbit.RequiredVelocity(keepAp, keepPe, planetMe.Length(), orb.Mu);
							//ScheduleProgradeManeuver(DateTime.Now, Orbit.RequiredVelocity(keepAp + keepPe, planetMe.Length(), orb.Mu));
							mstat = ManeuverStatus.PLANNED;
						}
						else if (ttap < ttpe)
						{
							// Approaching apoapsis, adjust peri
							if (Math.Abs(orb.Pe - keepPe) > ORBIT_PRECISION)
							{
								//vTarget = Orbit.RequiredVelocity((orb.Ap + keepPe) / 2, orb.Ap, orb.Mu);
								//vPredict = orb.GetPosVelAtEccentricAnomaly(Math.PI).vel.Length();
								//ScheduleProgradeManeuver(DateTime.Now + TimeSpan.FromSeconds(ttap), vTarget);
								mGetDelay = () => orb.TimeToAp();
								//mTargetAp = orb.Ap;
								//mTargetPe = keepPe;
								mGetTargetVel = () =>
								{
									Echo($"Man Ap {orb.Ap:F0} + kPe {keepPe:F0} = {orb.Ap + keepPe:F0}");
									Echo($"Man r {GetShipPosRTPlanet().Length():F0}");
									double vel = Orbit.RequiredVelocity(orb.Ap, keepPe, GetShipPosRTPlanet().Length(), orb.Mu);
									Echo($"Man vel {cockpit.GetShipSpeed():F0} -> {vel:F0}");
									return vel;
								};
								mstat = ManeuverStatus.PLANNED;
							}
						}
						else
						{
							// Approaching periapsis, adjust apo
							if (Math.Abs(orb.Ap - keepAp) > ORBIT_PRECISION)
							{
								//vTarget = Orbit.RequiredVelocity((orb.Pe + keepAp) / 2, orb.Pe, orb.Mu);
								//vPredict = orb.GetPosVelAtEccentricAnomaly(0).vel.Length();
								//ScheduleProgradeManeuver(DateTime.Now + TimeSpan.FromSeconds(ttpe), vTarget);
								mGetDelay = () => orb.TimeToPe();
								mGetTargetVel = () =>
								{
									Echo($"M tgt kAp {keepAp:F0} + Pe {orb.Pe:F0} = {keepAp + orb.Pe:F0}");
									Echo($"M r {GetShipPosRTPlanet().Length():F0}");
									double vel = Orbit.RequiredVelocity(keepAp, orb.Pe, GetShipPosRTPlanet().Length(), orb.Mu);
									Echo($"Man vel {cockpit.GetShipSpeed():F0} -> {vel:F0}");
									return vel;
								};
								mstat = ManeuverStatus.PLANNED;
							}
						}
					} // else if keep

					/************************************************
					//Base6Directions.Direction dir;
					//double availableThrust, accel, mtime;
					if (ttap < ttpe)
					{
						Echo($"Manuever at Ap:");
						vTarget = Orbit.RequiredVelocity((orb.Ap + keepPeri) / 2, orb.Ap, orb.Mu);
						vPredict = orb.GetPosVelAtEccentricAnomaly(Math.PI).vel.Length();
						Echo($"vTarget: {vTarget:F2}");
						Echo($"vPredict: {vPredict:F2}");

						if (Math.Abs(vPredict - vTarget) < 1) { StopThrust(); }
						else
						{
							if (vPredict < vTarget) { dir = Base6Directions.Direction.Forward; }
							else { dir = Base6Directions.Direction.Backward; }
							availableThrust = GetAvailableThrust(Base6Directions.Direction.Forward);
							Echo($"Avail thrust: {availableThrust:F2}");
							accel = availableThrust / cockpit.CalculateShipMass().TotalMass;
							Echo($"Max accel: {accel:F2}");
							mtime = Math.Abs((vPredict - vTarget) / accel);
							Echo($"Burn time: {mtime:F2}s");
							Echo($"ttap: {ttap}");
							if (ttap < mtime)
							{
								Echo("Thrusting!");
								thrusting = true;
								SetThrust(dir, Math.Abs(vTarget - speed) / accel / MANEUVER_SENSITIVITY + 0.01);
							}
							else
							{
								StopThrust();
							}
						}
					}
					else
					{
						vTarget = Orbit.RequiredVelocity((orb.Pe + keepApo) / 2, orb.Pe, orb.Mu);
						vPredict = orb.GetPosVelAtEccentricAnomaly(0).vel.Length();

						if (Math.Abs(vPredict - vTarget) < 1) { StopThrust(); }
						else
						{
							if (vPredict < vTarget) { dir = Base6Directions.Direction.Forward; }
							else { dir = Base6Directions.Direction.Backward; }
							availableThrust = GetAvailableThrust(Base6Directions.Direction.Forward);
							accel = availableThrust / cockpit.CalculateShipMass().TotalMass;
							Echo($"Max accel: {accel:F2}");
							mtime = Math.Abs((vPredict - vTarget) / accel);
							Echo($"maneuver: {mtime:F2}s");
							Echo($"ttpe: {ttpe}");
							if (ttpe < mtime)
							{
								Echo("Thrusting!");
								thrusting = true;
								SetThrust(dir, Math.Abs(vTarget - speed) / accel / MANEUVER_SENSITIVITY + 0.01);
							}
							else
							{
								StopThrust();
							}
						}
					}
					***********************************************/

				} // if (validOrbit)

































				//Echo($"vel={ppv(vPro)}");
				//Echo($"fwd={ppv(wvFwd)}");
				//Echo($"up={ppv(wvUp)}");
				//Echo($"right={ppv(wvRight)}");

				//Echo($"vFacing = \r\n{ppv(vFacing)}");
				//Echo($"vDest = \r\n{ppv(vDest)}");
				//Echo($"angleDiff = \r\n{angleDiff}");
				//Echo($"dPitch = {dPitch}");
				//Echo($"dYaw = {dYaw}");
				//Echo($"localToward=\r\n{ppv(localToward)}");





























				//if (!thrusting) { StopThrust(); }

				prevFrameSpeed = cockpit.GetShipSpeed();

			} // if Update1
			else
			{
				if (arg.StartsWith("pro"))
				{
					mode = Mode.Prograde;
					mstat = ManeuverStatus.IDLE;
					keep = false;
					StopThrust();
					Runtime.UpdateFrequency = UpdateFrequency.Update1;
				}
				else if (arg.StartsWith("ret"))
				{
					mode = Mode.Retrograde;
					mstat = ManeuverStatus.IDLE;
					keep = false;
					StopThrust();
					Runtime.UpdateFrequency = UpdateFrequency.Update1;
				}
				else if (arg.StartsWith("nor"))
				{
					mode = Mode.Normal;
					mstat = ManeuverStatus.IDLE;
					keep = false;
					StopThrust();
					Runtime.UpdateFrequency = UpdateFrequency.Update1;
				}
				else if (arg.StartsWith("ant"))
				{
					mode = Mode.Antinormal;
					mstat = ManeuverStatus.IDLE;
					keep = false;
					StopThrust();
					Runtime.UpdateFrequency = UpdateFrequency.Update1;
				}
				else if (arg.StartsWith("in"))
				{
					mode = Mode.RadialIn;
					mstat = ManeuverStatus.IDLE;
					keep = false;
					StopThrust();
					Runtime.UpdateFrequency = UpdateFrequency.Update1;
				}
				else if (arg.StartsWith("out"))
				{
					mode = Mode.RadialOut;
					mstat = ManeuverStatus.IDLE;
					keep = false;
					StopThrust();
					Runtime.UpdateFrequency = UpdateFrequency.Update1;
				}
				else if (arg.StartsWith("off") || arg.StartsWith("stop"))
				{
					Stop();
				}
				// Debug testing commands
				else if (arg=="right")
				{
					foreach (var g in gyt) { g.g.GyroOverride = true; g.setYaw(g.g, 1.0); }
				}
				else if (arg=="left")
				{
					foreach (var g in gyt) { g.g.GyroOverride = true; g.setYaw(g.g, -1.0); }
				}
				else if (arg=="up")
				{
					foreach (var g in gyt) { g.g.GyroOverride = true; g.setPitch(g.g, 1.0); }
				}
				else if (arg=="down")
				{
					foreach (var g in gyt) { g.g.GyroOverride = true; g.setPitch(g.g, -1.0); }
				}
				else if (arg.StartsWith("disp"))
				{
					Stop();
					mode = Mode.DisplayOnly;
					mstat = ManeuverStatus.IDLE;
					keep = false;
					StopThrust();
					Runtime.UpdateFrequency = UpdateFrequency.Update1;
				}
				else if (arg.StartsWith("keep"))
				{
					Echo("arg.StarsWith('keep')");
					var sp = arg.Split(' ');
					double tmpAp, tmpPe;
					Echo($"sp.Length: {sp.Length}");
					if (sp.Length == 3)
					{
						if (double.TryParse(sp[1], out tmpAp) && double.TryParse(sp[2], out tmpPe))
						{
							Echo("Setting Ap/Pe");
							keepAp = tmpAp;
							keepPe = tmpPe;
							keep = true;
						}
						else
						{
							Echo("keep 2-arg form expected [apoapsis] [periapsis]");
						}
						//keepApo = sp[1]
					}
					else if (sp.Length == 2)
					{
						double tmpd;
						if (sp[1] == "on")
						{
							Echo("keep on");
							keep = true;
							UpdateOrbit();
							keepAp = orb.Ap;
							keepPe = orb.Pe;
						}
						else if (sp[1] == "off")
						{
							Echo("keep off");
							StopThrust();
							mstat = ManeuverStatus.IDLE;
							keep = false;
							// TODO: Cancel thrust
						}
						else if (sp[1] == "toggle")
						{
							Echo("keep toggle");
							keep = !keep;
							if (keep)
							{
								Echo("keep on");
								UpdateOrbit();
								keepAp = orb.Ap;
								keepPe = orb.Pe;
							}
							else
							{
								Echo("keep off");
								StopThrust();
								mstat = ManeuverStatus.IDLE;
							}
						}
						else if (double.TryParse(sp[1], out tmpd))
						{
							Echo("keep on");
							keep = true;
							keepAp = keepPe = tmpd;
							mode = Mode.Prograde;
						}
					}
					else if (sp.Length == 1)
					{
						keep = !keep;
						UpdateOrbit();
						keepAp = orb.Ap;
						keepPe = orb.Pe;
					}


					if (keep)
					{
						mode = Mode.Prograde;
						Runtime.UpdateFrequency = UpdateFrequency.Update1;
					}
					Echo($"keep={keep}, Ap={keepAp:0F}, Pe={keepPe:0F}");
					Echo($"mode = {mode}");
				} //"keep"
				//else if (arg == "t")
				//{
				//	// test
				//	UpdateThrust();
				//	SetThrust(Base6Directions.Direction.Backward, 1);
				//}
				//else if (arg.StartsWith("hold"))
				//{
				//	mode = Mode.Disabled
				//}
				//if (mode != Mode.Prograde && keep) { keep = false; }
			} // if update1 / else

			Echo($"CPU: {(100.0f * (float)Runtime.CurrentInstructionCount / (float)Runtime.MaxInstructionCount):F2}%");
		} // Main()

		void Stop()
		{
			mode = Mode.Disabled;
			keep = false;
			mstat = ManeuverStatus.IDLE;
			Runtime.UpdateFrequency = UpdateFrequency.None;
			StopThrust();
			foreach (var g in gyros)
			{
				g.Yaw = 0;
				g.Pitch = 0;
				g.Roll = 0;
				g.GyroOverride = false;
			}
			Echo("Stopped");
		}

		public static string timestr(TimeSpan t) { return $"{t.Hours:00}:{t.Minutes:00}:{t.Seconds:00}"; }
		public static string timestr(double seconds)
		{
			if (double.IsNaN(seconds)) { return "(NaN)"; }
			if (double.IsNegativeInfinity(seconds)) { return "(-infinity)"; }
			if (double.IsPositiveInfinity(seconds)) { return "(+infinity)"; }
			TimeSpan t = TimeSpan.FromSeconds(seconds);
			return $"{t.Hours:00}:{t.Minutes:00}:{t.Seconds:00}";
		}
		public static string timestr(DateTime t) { return $"{t.Hour:00}:{t.Minute:00}:{t.Second:00}"; }

		bool validOrbit = false;
		uint orbitUpdatedFrame = 0;
		uint frameNum = 0;
		void UpdateOrbit()
		{
			if (orbitUpdatedFrame == frameNum) { return; }
			Vector3D pp;
			if (!cockpit.TryGetPlanetPosition(out pp)) { validOrbit = false; }
			else { validOrbit = true; }
			Vector3D shipPosRelativeToPlanet = cockpit.CenterOfMass - pp;
			orb.UpdateFromPosVel(
				shipPosRelativeToPlanet,
				cockpit.GetShipVelocities().LinearVelocity,
				cockpit.GetNaturalGravity().Length() * shipPosRelativeToPlanet.LengthSquared()
			);
			orbitUpdatedFrame = frameNum;
		}

		Vector3D GetShipPosRTPlanet()
		{
			Vector3D pp;
			if (!cockpit.TryGetPlanetPosition(out pp)) { return new Vector3D(0, 0, 0); }
			return cockpit.CenterOfMass - pp;
		}

		// Pretty Print Vector
		string ppv(Vector3D v, int rounding = 2)
		{
			return $"({Math.Round(v.X, rounding)}, {Math.Round(v.Y, rounding)}, {Math.Round(v.Z, rounding)})";
		}
		void asn(ref QuaternionD q, Vector3D v)
		{
			q.X = v.X;
			q.Y = v.Y;
			q.Z = v.Z;
			q.W = 0;
		}
		QuaternionD v2q(Vector3D v)
		{
			return new QuaternionD(v.X, v.Y, v.Z, 0);
		}
		Vector3D q2v(QuaternionD q)
		{
			return new Vector3D(q.X, q.Y, q.Z);
		}



		Dictionary<Base6Directions.Direction, List<IMyThrust>> dirThrust = new Dictionary<Base6Directions.Direction, List<IMyThrust>>();
		uint frameUpdateThrust = 0;
		void UpdateThrust()
		{
			// GridThrustDirection:
			// -z is backwards
			// +z is forwards
			// +y is down
			// -y is up
			// +x is left
			// -x is right
			if (frameUpdateThrust == frameNum) { return; }
			G.GetBlocksOfType(allThrust);
			if (dirThrust.Count == 0)
			{
				dirThrust[Base6Directions.Direction.Backward] = new List<IMyThrust>();
				dirThrust[Base6Directions.Direction.Down] = new List<IMyThrust>();
				dirThrust[Base6Directions.Direction.Forward] = new List<IMyThrust>();
				dirThrust[Base6Directions.Direction.Left] = new List<IMyThrust>();
				dirThrust[Base6Directions.Direction.Right] = new List<IMyThrust>();
				dirThrust[Base6Directions.Direction.Up] = new List<IMyThrust>();
			}
			foreach (var list in dirThrust) { list.Value.Clear(); }
			foreach (var t in allThrust)
			{
				dirThrust[Base6Directions.GetOppositeDirection(Base6Directions.GetDirection(t.GridThrustDirection))].Add(t);
			}
			frameUpdateThrust = frameNum;
		}
		double GetAvailableThrust(Base6Directions.Direction dir)
		{
			double ret = 0;
			UpdateThrust();
			foreach (var t in dirThrust[dir]) { ret += t.MaxEffectiveThrust; }
			return ret;
		}
		void SetThrust(Base6Directions.Direction dir, double overrideFactor)
		{
			overrideFactor = Math.Max(0, Math.Min(1, overrideFactor));
			UpdateThrust();
			//Echo($"{dirThrust[dir].Count} {dir} thrusters");
			foreach (var t in dirThrust[dir])
			{
				//Echo(t.CustomName);
				//t.ThrustOverridePercentage = 100.0f * Math.Max(0, Math.Min(1, (float)overrideFactor));
				t.ThrustOverride = t.MaxEffectiveThrust * (float)overrideFactor;
			}
		}
		void StopThrust()
		{
			G.GetBlocksOfType(allThrust);
			foreach (var t in allThrust) { t.ThrustOverride = 0; }
		}



		const Base6Directions.Direction _up = Base6Directions.Direction.Up;
		const Base6Directions.Direction _down = Base6Directions.Direction.Down;
		const Base6Directions.Direction _left = Base6Directions.Direction.Left;
		const Base6Directions.Direction _right = Base6Directions.Direction.Right;
		const Base6Directions.Direction _forward = Base6Directions.Direction.Forward;
		const Base6Directions.Direction _backward = Base6Directions.Direction.Backward;
		static Base6Directions.Direction OppositeDirection(Base6Directions.Direction d)
		{
			return Base6Directions.GetOppositeDirection(d);
			//switch (d)
			//{
			//	case _up: return _down;
			//	case _down: return _up;
			//	case _left: return _right;
			//	case _right: return _left;
			//	case _forward: return _backward;
			//	case _backward: return _forward;
			//	default: throw new Exception("Unknown Base6Direction");
			//}
		}
		delegate double GyroOverrideGetter(IMyGyro g);
		delegate void GyroOverrideSetter(IMyGyro g, double rpm);
		static double gyroGetPosPitch(IMyGyro g) => g.Pitch;
		static double gyroGetNegPitch(IMyGyro g) => -g.Pitch;
		static double gyroGetPosRoll(IMyGyro g) => g.Roll;
		static double gyroGetNegRoll(IMyGyro g) => -g.Roll;
		static double gyroGetPosYaw(IMyGyro g) => g.Yaw;
		static double gyroGetNegYaw(IMyGyro g) => -g.Yaw;
		static void gyroSetPosPitch(IMyGyro g, double rpm)
		{
			float before = g.Pitch;
			//g.Pitch = (float)rpm;
			g.SetValueFloat("Pitch", (float)rpm);
			//dbg($"gyroSetPosPitch({g.CustomName}, {rpm:0.00}); before={before:0.00} after={g.Pitch:0.00}");
		}
		static void gyroSetNegPitch(IMyGyro g, double rpm)
		{
			//g.Pitch = (float)(-rpm);
			g.SetValueFloat("Pitch", (float)(-rpm));
			//dbg($"gyroSetNegPitch({g.CustomName}, {rpm:0.00})");
		}
		static void gyroSetPosRoll(IMyGyro g, double rpm)
		{
			//g.Roll = (float)rpm;
			g.SetValueFloat("Roll", (float)rpm);
			//dbg($"gyroSetPosRoll({g.CustomName}, {rpm:0.00})");
		}
		static void gyroSetNegRoll(IMyGyro g, double rpm)
		{
			//g.Roll = (float)(-rpm);
			g.SetValueFloat("Roll", (float)(-rpm));
			//dbg($"gyroSetNegRoll({g.CustomName}, {rpm:0.00})");
		}
		static void gyroSetPosYaw(IMyGyro g, double rpm)
		{
			//g.Yaw = (float)rpm;
			g.SetValueFloat("Yaw", (float)rpm);
			//dbg($"gyroSetPosYaw({g.CustomName}, {rpm:0.00})");
		}
		static void gyroSetNegYaw(IMyGyro g, double rpm)
		{
			//g.Yaw = (float)(-rpm);
			g.SetValueFloat("Yaw", (float)(-rpm));
			//dbg($"gyroSetNegYaw({g.CustomName}, {rpm:0.00})");
		}
		static Base6Directions.Direction GridDirectionToBlock(IMyTerminalBlock block, Base6Directions.Direction d)
		{
			if (block == null)
			{
				return d;
			}
			//switch (block.Orientation.Up)
			if (d == block.Orientation.Up) return _up;
			if (d == OppositeDirection(block.Orientation.Up)) return _down;
			if (d == block.Orientation.Left) return _left;
			if (d == OppositeDirection(block.Orientation.Left)) return _right;
			if (d == block.Orientation.Forward) return _forward;
			if (d == OppositeDirection(block.Orientation.Forward)) return _backward;
			throw new Exception("Unknown Base6Direction");
		}
		class GyroTranslator
		{
			public IMyGyro g;
			public GyroOverrideGetter getPitch;
			public GyroOverrideGetter getRoll;
			public GyroOverrideSetter setRoll;
			public GyroOverrideSetter setPitch;
			public GyroOverrideGetter getYaw;
			public GyroOverrideSetter setYaw;
			public GyroTranslator(IMyGyro gyro, IMyTerminalBlock cockpit) //, Program p)
			{
				g = gyro;
				var ds = new StringBuilder();
				ds.Append(g.CustomName);

				//switch (g.Orientation.Forward)
				switch (GridDirectionToBlock(cockpit, g.Orientation.Forward))
				{
					case Base6Directions.Direction.Forward:
						ds.Append(" FF");
						getRoll = gyroGetPosRoll;
						setRoll = gyroSetPosRoll;
						break;
					case Base6Directions.Direction.Backward:
						ds.Append(" FB");
						getRoll = gyroGetNegRoll;
						setRoll = gyroSetNegRoll;
						break;
					case Base6Directions.Direction.Right:
						ds.Append(" FR");
						getPitch = gyroGetPosRoll;
						setPitch = gyroSetPosRoll;
						break;
					case Base6Directions.Direction.Left:
						ds.Append(" FL");
						getPitch = gyroGetNegRoll;
						setPitch = gyroSetNegRoll;
						break;
					case Base6Directions.Direction.Up:
						ds.Append(" FU");
						getYaw = gyroGetNegRoll;
						setYaw = gyroSetNegRoll;
						break;
					case Base6Directions.Direction.Down:
						ds.Append(" FD");
						getYaw = gyroGetPosRoll;
						setYaw = gyroSetPosRoll;
						break;
				}
				switch (GridDirectionToBlock(cockpit, g.Orientation.Up))
				{
					case Base6Directions.Direction.Forward:
						ds.Append(" UF");
						//getRoll = gyroGetPosYaw;
						//setRoll = gyroSetPosYaw;
						getRoll = gyroGetNegYaw;
						setRoll = gyroSetNegYaw;
						break;
					case Base6Directions.Direction.Backward:
						ds.Append(" UB");
						//getRoll = gyroGetNegYaw;
						//setRoll = gyroSetNegYaw;
						getRoll = gyroGetPosYaw;
						setRoll = gyroSetPosYaw;
						break;
					case Base6Directions.Direction.Right:
						ds.Append(" UR");
						//getPitch = gyroGetPosYaw;
						//setPitch = gyroSetPosYaw;
						getPitch = gyroGetNegYaw;
						setPitch = gyroSetNegYaw;
						break;
					case Base6Directions.Direction.Left:
						ds.Append(" UL");
						//getPitch = gyroGetNegYaw;
						//setPitch = gyroSetNegYaw;
						getPitch = gyroGetPosYaw;
						setPitch = gyroSetPosYaw;
						break;
					case Base6Directions.Direction.Up:
						ds.Append(" UU");
						getYaw = gyroGetPosYaw;
						setYaw = gyroSetPosYaw;
						break;
					case Base6Directions.Direction.Down:
						ds.Append(" UD");
						getYaw = gyroGetNegYaw;
						setYaw = gyroSetNegYaw;
						break;
				}
				switch (GridDirectionToBlock(cockpit, g.Orientation.Left))
				{
					case Base6Directions.Direction.Left:
						ds.Append(" LL");
						getPitch = gyroGetPosPitch;
						setPitch = gyroSetPosPitch;
						break;
					case Base6Directions.Direction.Right:
						ds.Append(" LR");
						getPitch = gyroGetNegPitch;
						setPitch = gyroSetNegPitch;
						break;
					case Base6Directions.Direction.Forward:
						ds.Append(" LF");
						//getRoll = gyroGetPosPitch;
						//setRoll = gyroSetPosPitch;
						getRoll = gyroGetNegPitch;
						setRoll = gyroSetNegPitch;
						break;
					case Base6Directions.Direction.Backward:
						ds.Append(" LB");
						//getRoll = gyroGetNegPitch;
						//setRoll = gyroSetNegPitch;
						getRoll = gyroGetPosPitch;
						setRoll = gyroSetPosPitch;
						break;
					case Base6Directions.Direction.Up:
						ds.Append(" LU");
						getYaw = gyroGetPosPitch;
						setYaw = gyroSetPosPitch;
						break;
					case Base6Directions.Direction.Down:
						ds.Append(" LD");
						getYaw = gyroGetNegPitch;
						setYaw = gyroSetNegPitch;
						break;
				}
				//ds.Append("---");
			}
		} // class GyroTranslator
		void FindGyros(IMyShipController cockpit)
		{
			gyros.Clear();
			gyt.Clear();
			G.GetBlocksOfType(gyros);
			foreach (var g in gyros)
			{
				gyt.Add(new GyroTranslator(g, cockpit));
			}

			//double torqueToMassRatio = (double)gyros.Count / (double)activeCockpit.CalculateShipMass().TotalMass;
			//if (activeCockpit.CubeGrid.GridSizeEnum == MyCubeSize.Large) { torqueToMassRatio *= 5.0; }
			////autoResponseFactor = torqueToMassRatio / (1.0 / 5000.0);
			//AutoResponse = torqueToMassRatio * 5000.0;

			ReconfigurePIDs();
		} // FindGyros()

		void ReconfigurePIDs()
		{
			// TODO: Torque to mass ratio
		}

		Vector3 mul(MatrixD m, Vector3 v)
		{
			return new Vector3(
				(m.M11 * v.X) + (m.M21 * v.Y) + (m.M31 * v.Z) + m.M41,
				(m.M12 * v.X) + (m.M22 * v.Y) + (m.M32 * v.Z) + m.M42,
				(m.M13 * v.X) + (m.M23 * v.Y) + (m.M33 * v.Z) + m.M43);
		}

		/*******************************************
		Vector3 div(Vector3 v, MatrixD m)
		{
			return mul(Inverse(m), v);
		}

		MatrixD Inverse(MatrixD m)
		{
			var r = new MatrixD();
			r.M11 = 0
			  + (m.M22 * m.M33 * m.M44) + (m.M32 * m.M43 * m.M24) + (m.M42 * m.M23 * m.M34)
			  - (m.M42 * m.M33 * m.M24) - (m.M32 * m.M23 * m.M44) - (m.M22 * m.M43 * m.M34);
			r.M21 = 0
			  - (m.M21 * m.M33 * m.M44) - (m.M31 * m.M43 * m.M24) - (m.M41 * m.M23 * m.M34)
			  + (m.M41 * m.M33 * m.M24) + (m.M31 * m.M23 * m.M44) + (m.M21 * m.M43 * m.M34);
			r.M31 = 0
			  + (m.M21 * m.M32 * m.M44) + (m.M31 * m.M42 * m.M24) + (m.M41 * m.M22 * m.M34)
			  - (m.M41 * m.M32 * m.M24) - (m.M31 * m.M22 * m.M44) - (m.M21 * m.M42 * m.M34);
			r.M41 = 0
			  - (m.M21 * m.M32 * m.M43) - (m.M31 * m.M42 * m.M23) - (m.M41 * m.M22 * m.M33)
			  + (m.M41 * m.M32 * m.M23) + (m.M31 * m.M22 * m.M43) + (m.M21 * m.M42 * m.M33);

			r.M12 = 0
			  - (m.M12 * m.M33 * m.M44) - (m.M32 * m.M43 * m.M14) - (m.M42 * m.M13 * m.M34)
			  + (m.M42 * m.M33 * m.M14) + (m.M32 * m.M13 * m.M44) + (m.M12 * m.M43 * m.M34);
			r.M22 = 0
			  + (m.M11 * m.M33 * m.M44) + (m.M31 * m.M43 * m.M14) + (m.M41 * m.M13 * m.M34)
			  - (m.M41 * m.M33 * m.M14) - (m.M31 * m.M13 * m.M44) - (m.M11 * m.M43 * m.M34);
			r.M32 = 0
			  - (m.M11 * m.M32 * m.M44) - (m.M31 * m.M42 * m.M14) - (m.M41 * m.M12 * m.M34)
			  + (m.M41 * m.M32 * m.M14) + (m.M31 * m.M12 * m.M44) + (m.M11 * m.M42 * m.M34);
			r.M42 = 0
			  + (m.M11 * m.M32 * m.M43) + (m.M31 * m.M42 * m.M13) + (m.M41 * m.M12 * m.M33)
			  - (m.M41 * m.M32 * m.M13) - (m.M31 * m.M12 * m.M43) - (m.M11 * m.M42 * m.M33);

			r.M13 = 0
			  + (m.M12 * m.M23 * m.M44) + (m.M22 * m.M43 * m.M14) + (m.M42 * m.M13 * m.M24)
			  - (m.M42 * m.M23 * m.M14) - (m.M22 * m.M13 * m.M44) - (m.M12 * m.M43 * m.M24);
			r.M23 = 0
			  - (m.M11 * m.M23 * m.M44) - (m.M21 * m.M43 * m.M14) - (m.M41 * m.M13 * m.M24)
			  + (m.M41 * m.M23 * m.M14) + (m.M21 * m.M13 * m.M44) + (m.M11 * m.M43 * m.M24);
			r.M33 = 0
			  + (m.M11 * m.M22 * m.M44) + (m.M21 * m.M42 * m.M14) + (m.M41 * m.M12 * m.M24)
			  - (m.M41 * m.M22 * m.M14) - (m.M21 * m.M12 * m.M44) - (m.M11 * m.M42 * m.M24);
			r.M43 =
			  -(m.M11 * m.M22 * m.M43) - (m.M21 * m.M42 * m.M13) - (m.M41 * m.M12 * m.M23)
			  + (m.M41 * m.M22 * m.M13) + (m.M21 * m.M12 * m.M43) + (m.M11 * m.M42 * m.M23);

			r.M14 = 0
			  - (m.M12 * m.M23 * m.M34) - (m.M22 * m.M33 * m.M14) - (m.M32 * m.M13 * m.M24)
			  + (m.M32 * m.M23 * m.M14) + (m.M22 * m.M13 * m.M34) + (m.M12 * m.M33 * m.M24);
			r.M24 = 0
			  + (m.M11 * m.M23 * m.M34) + (m.M21 * m.M33 * m.M14) + (m.M31 * m.M13 * m.M24)
			  - (m.M31 * m.M23 * m.M14) - (m.M21 * m.M13 * m.M34) - (m.M11 * m.M33 * m.M24);
			r.M34 = 0
			  - (m.M11 * m.M22 * m.M34) - (m.M21 * m.M32 * m.M14) - (m.M31 * m.M12 * m.M24)
			  + (m.M31 * m.M22 * m.M14) + (m.M21 * m.M12 * m.M34) + (m.M11 * m.M32 * m.M24);
			r.M44 = 0
			  + (m.M11 * m.M22 * m.M33) + (m.M21 * m.M32 * m.M13) + (m.M31 * m.M12 * m.M23)
			  - (m.M31 * m.M22 * m.M13) - (m.M21 * m.M12 * m.M33) - (m.M11 * m.M32 * m.M23);
			// TODO: I think we have to divide all that by the det
			return r;
		}
		// ************************************************************************/

		static double AngleBetweenDeg(Vector3D pa, Vector3D pb) //returns radians 
		{
			return (180.0 / Math.PI) * Math.Acos(pa.Dot(pb) / (pa.Length() * pb.Length()));
		}
		class PID
		{
			/* Controller gains */
			public double Kp;
			public double Ki;
			public double Kd;

			/* Derivative low-pass filter time constant */
			public double lowPass;

			/* Output limits */
			public double limMin;
			public double limMax;

			/* Integrator limits */
			public double limMinInt;
			public double limMaxInt;

			/* Sample time (in seconds) */
			public double dt;

			public double setpoint;
			//public double lowPassOut;

			/* Controller "memory" */
			// TODO: make private again
			public double integrator;
			public double prevError;            /* Required for integrator */
			public double differentiator;
			public double prevMeasurement;      /* Required for differentiator */
			//private double prevOut;

			/* Controller output */
			private double m_output;
			public double Output { get { return m_output; } }

			public double Update(double measurement, bool debugPrint = false)
			{
				double error = setpoint - measurement;
				double proportional = Kp * error;
				//integrator = integrator + 0.5f * Ki * dt * (error + prevError);
				double integrator_delta = 0.5f * Ki * dt * (error + prevError);
				integrator += integrator_delta;
				//integrator = integrator + 0.5f * Ki * dt * (error + prevError);

				/* Anti-wind-up via integrator clamping */
				if (integrator > limMaxInt)
				{
					integrator = limMaxInt;
				}
				else if (integrator < limMinInt)
				{
					integrator = limMinInt;
				}


				// (band-limited differentiator)
				differentiator = -(2.0f * Kd * (measurement - prevMeasurement)   /* Note: derivative on measurement, therefore minus sign in front of equation! */
									+ (2.0f * lowPass - dt) * differentiator)
									/ (2.0f * lowPass + dt);


				//if (proportional == double.NaN) { throw new ArgumentException("proportional NaN"); }
				//if (integrator == double.NaN) { throw new ArgumentException("integrator NaN"); }
				//if (differentiator == double.NaN) { throw new ArgumentException("differentiator NaN"); }
				//if (proportional == double.NaN) { p.Disp("proportional NaN"); }
				//if (integrator == double.NaN) { p.Disp("integrator NaN"); }
				//if (differentiator == double.NaN) { p.Disp("differentiator NaN"); }

				// Compute output and apply limits
				m_output = proportional + integrator + differentiator;

				//if (m_output == double.NaN) { p.Disp("output NaN"); }
				if (m_output > limMax)
				{

					m_output = limMax;

				}
				else if (m_output < limMin)
				{

					m_output = limMin;

				}


				/* Store error and measurement for later use */
				prevError = error;
				prevMeasurement = measurement;

				if (double.IsNaN(integrator)) { integrator = 0; }
				if (double.IsNaN(prevError)) { prevError = 0; }
				if (double.IsNaN(differentiator)) { differentiator = 0; }
				if (double.IsNaN(prevMeasurement)) { prevMeasurement = 0; }
				if (double.IsNaN(m_output)) { m_output = 0; }


				/* Return controller output */
				return m_output;
			}
		} // class PID

		static readonly string[] s_spinnerChars = { "/", "-", "\\", "-" };
		static int s_spinnerIndex;
		string Spinner()
		{
			s_spinnerIndex = (s_spinnerIndex + 1) % 4;
			return s_spinnerChars[s_spinnerIndex];
		}


		public struct PosVel
		{
			public Vector3D pos;
			public Vector3D vel;
		}
		public class Orbit
		{

			// a = semi major axis
			// e = eccentricity
			// i = inclination
			// p = periapsis
			// Om = longitude of ascending node (capital omega)
			// w = argument of periapsis (angle from ascending node) (little omega looks similar to latin w)
			// v = true anomaly

			public double a, e, p, i, Om, w, v;
			public double Ap, Pe, T, b, Area, M, Ea, nMeanMotion, tpe, c, Mu;
			public Vector3D normal;
			public DateTime epoch;

			public Vector3D equinox = new Vector3D(1,0,0);
			public Vector3D eastOfEquinox = new Vector3D(0, 1, 0);
			public Vector3D north = new Vector3D(0, 0, 1);

			//public MatrixD transform;
			public Vector3D basis_x, basis_y, basis_z; // convert from ellipse calculations in the x/y plane to 3d planet space
			public Orbit() { }
			public Orbit(Vector3D vEquinox, Vector3D vEastOfEquinox, Vector3D vNorth)
			{
				equinox = vEquinox;
				eastOfEquinox = vEastOfEquinox;
				north = vNorth;
			}
			static Vector3D mul(double s, Vector3 v)
			{
				return new Vector3(s * v.X, s * v.Y, s * v.Z);
			}
			static Vector3D div(Vector3 v, double s)
			{
				return new Vector3(v.X / s, v.Y / s, v.Z / s);
			}

			public static double RequiredVelocity(double Ap, double Pe, double r, double mu)
			{
				return RequiredVelocity((Ap + Pe) / 2, r, mu);
			}
			public static double RequiredVelocity(double sma, double r, double mu)
			{
				// double E = (v * v / 2) - (mu / r);
				// this.a = -mu / (2 * E);
				// a = -mu / (2 * ((v * v / 2) - (mu / r)))
				// a / -mu = 1 / (2 * ((v * v / 2) - (mu / r)))
				// -mu / a = 2 * ((v * v / 2) - (mu / r))
				// -mu / a = (2 * v*v / 2) - (2*mu / r))
				// -mu/a + (2*mu/r) = v^2
				return Math.Sqrt((-mu / sma) + (2 * mu / r));
			}

			public void UpdateFromPosVel(
					Vector3D pos /*radial vector (position)*/,
					Vector3D vel /*velocity vector*/,
					double mu /* mu = GM, units m^3 s^-2, gravitational constant times mass of planet in kg, or, acceleration due to gravity * r^2 */
				)

			//PQ M = 5.9722e+24_kg,
			////PQ G = PQ("6.6743015e-11 N m^2 / kg^2")
			////PQ G = 6.6743015e-11 * 1_N * 1_m * 1_m / 1_kg / 1_kg
			//PQ G = 6.6743015e-14 * 1_N * 1_m * 1_m / 1_kg / 1_kg
			{
				//Orbit q = new Orbit();
				//Vector3D rvbak = rv;
				//Vector3D vvbak = vv;
				Vector3D rv = new Vector3D(pos.Dot(equinox), pos.Dot(eastOfEquinox), pos.Dot(north));
				Vector3D vv = new Vector3D(vel.Dot(equinox), vel.Dot(eastOfEquinox), vel.Dot(north));

				//prg.Echo($"rv: ({rv.X:F0},{rv.Y:F0},{rv.Z:F0})");
				//prg.Echo($"vv: ({vv.X:F0},{vv.Y:F0},{vv.Z:F0})");


				this.Mu = mu;

				double r = rv.Length(); // radius
				double v = vv.Length(); // velocity

				// angular momentum
				Vector3D hv = rv.Cross(vv);
				normal = hv.Normalized();
				double h = hv.Length();

				// basis vector
				Vector3D khat = new Vector3(0, 0, 1);
				Vector3D ihat = new Vector3(1, 0, 0);

				// node vector (normal?)
				Vector3D nhat = khat.Cross(hv);
				double n = nhat.Length();

				// Eccentricity
				//Vector3D ev = div(mul(v * v - (mu / r), rv) - (rv.Dot(vv) * vv), mu);
				Vector3D ev = ((((v * v) - (mu / r)) * rv) - (rv.Dot(vv) * vv)) / mu;
				this.e = ev.Length();

				// specific mechanical energy
				double E = (v * v / 2) - (mu / r);

				if (this.e < 1.0)
				{
					// semi major axis
					this.a = -mu / (2 * E);

					// periapsis
					this.p = this.a * (1 - this.e * this.e);
				}
				else
				{
					// not an elliptical orbit
					this.a = double.PositiveInfinity;
					this.p = h * h / mu;
				}

				this.i = Math.Acos(hv.Z / h);  // i is inclination

				if (n == 0) { this.Om = 0; }
				else { this.Om = Math.Acos(nhat.X / n); }  // Omega is the longitude of ascending node
														   //if (this.e == 0) { this.w =  }
				if (e == 0) { this.w = -Om; }
				else if (n == 0) { this.w = Math.Acos(ev.X / (this.e)); }
				else { this.w = Math.Acos(nhat.Dot(ev) / (n * this.e)); } // Argument of periapsis (angle from ascending node)
				this.v = Math.Acos(ev.Dot(rv) / (this.e * r)); // True anomaly (at epoch) (angle from focus-periapsis)
				bool descent = (ev.Cross(rv).Dot(normal) < 0);
				if (descent) { this.v = (2 * Math.PI) - this.v; }
				this.epoch = DateTime.Now;
				this.c = this.a * this.e; // c is distance from center to focus
				Ap = (this.a) + c;  // Apoapsis height
				Pe = (this.a) - c;  // Periapsis height
				//T = 2 * Math.PI * Math.Sqrt(a * a * a / mu); // Orbital period (time to complete one full orbit)
				// b2 + c2 = a2
				// b2 = a2 - c2
				b = Math.Sqrt(a*a + c*c); // semi minor axis
				Area = Math.PI * a * b;
				// M is Mean anomaly ;,;
				//double sinf = Math.Sin(v); // f and v are interchangeable for true anomaly
				//double cosf = Math.Cos(v);
				//double e2 = e * e;
				//M = Math.Atan2(-Math.Sqrt(1 - e2) * sinf, -e - cosf) + Math.PI - (e * (Math.Sqrt(1 - e2) * sinf) / (1 + e * cosf));
				// Ea = eccentric anomaly (angle from center of ellipse instead of focus)
				// Note: Ea is both + and -
				//if (e < 1) {
				//Ea = Math.Acos((e + cosf) / (1 + (e * cosf)));
				Ea = TrueToEccentricAnomaly(this.v);
				//}
				//else { Ea = Math.Acosh((e + cosf) / (1 + (e * cosf))) }
				//M = Ea - (e * sinf);
				M = EccentricToMeanAnomaly(Ea);


				// time to peri/apo/any given true anomaly...
				nMeanMotion = Math.Sqrt(mu / Math.Abs(a * a * a));
				T = 2 * Math.PI / nMeanMotion;
				tpe = M / nMeanMotion;


				{
					// TODO: basis_x and basis_y might need to be * -1
					QuaternionD qpe = QuaternionD.CreateFromAxisAngle(normal, w); // rotate about normal by argument of periapsis
					Vector3D van = new Vector3D(Math.Cos(Om), Math.Sin(Om), 0); // ascending node vector
					Vector3D vpe = qpe * van; // i/x basis will be major axis
					Vector3D vlat = -(vpe.Cross(normal)); // j/y basis will be parallel to latus rectum, minor axis, and directrix

					basis_x = vpe; basis_x.Normalize();
					basis_y = vlat; basis_y.Normalize();
					basis_z = normal; basis_z.Normalize();
					//MatrixD m = new MatrixD();
					//m.M11 = vpe.X; m.M21 = vpe.Y; m.M31 = vpe.Z; m.M41 = 0;
					//m.M12 = vlat.X; m.M22 = vlat.Y; m.M32 = vlat.Z; m.M42 = 0;
					//m.M13 = normal.X; m.M23 = normal.Y; m.M33 = normal.Z; m.M43 = 0;
					//m.M14 = m.M24 = m.M34 = 0;
					//m.M44 = 1;
					//transform = m;
				}
			} // Orbit.FromPosVel
			public double EccentricToTrueAnomaly(double E)
			{
				double ret = 2 * Math.Atan(Math.Sqrt((1 + e) / (1 - e)) * Math.Tan(E / 2));
				if (E > Math.PI) { ret += 2 * Math.PI; }
				return ret;
			}
			public double TrueToEccentricAnomaly(double f)
			{
				double cosf = Math.Cos(f);
				double ret = Math.Acos((e + cosf) / (1 + e * cosf));
				if (f > Math.PI) { ret = 2 * Math.PI - ret; }
				return ret;
			}
			public double EccentricToMeanAnomaly(double E)
			{
				double ret = E - (e * Math.Sin(E));
				//if (E>Math.PI) { ret = (2 * Math.PI) - ret; }
				return ret;
			}
			public double TrueToMeanAnomaly(double f)
			{
				return EccentricToMeanAnomaly(TrueToEccentricAnomaly(f));
			}
			// This function can be off by about 5 degrees, but provides a continuous motion
			public double MeanToEccentricAnomaly_estimate(double M)
			{
				if (M <= Math.PI)
				{
					return 1.96 * Math.Pow(M, 0.41213689);
				}
				else
				{
					return (2 * Math.PI) - (1.96 * Math.Pow(2 * Math.PI - M, 0.41213689));
				}
			}
			public double MeanToTrueAnomaly_estimate(double M)
			{
				return EccentricToTrueAnomaly(MeanToEccentricAnomaly_estimate(M));
			}
			//double MeanToEccentricAnomaly_estimate(double M)
			//{
			//	// M = E - (e sin E)
			//	double d = Math.Pow(M / Math.PI, 1.46);
			//	// M / E ~~ (x/pi)^1.46 - x/18                :  0 <= x < 0.5
			//	// M / E ~~ (x/pi)^1.46 - 0.033011 sin 2x     :  0.5 <= x < pi/2
			//	// M / E ~~ (x/pi)^1.46 - 0.044 sin 2x        :  pi/2 <= x <= pi
			//	double n;
			//	bool flip = false;
			//	while (M < 0) { M += Math.PI * 2; }
			//	while (M > 2 * Math.PI) { M -= 2 * Math.PI; }
			//	if (M > Math.PI)
			//	{
			//		M = 2 * Math.PI - M;
			//		flip = true;
			//	}
			//	if (M < 0.5) { n = d - (M / 18.0); }
			//	else if (M < Math.PI / 2.0) { n = d - (0.033011 * Math.Sin(2 * M)); }
			//	else { n = d - (0.044 * Math.Sin(2 * M)); }
			//	double ret = n / M;
			//	if (flip)
			//	{
			//		ret = 2 * Math.PI - ret;
			//	}
			//	return ret;
			//}


			public double GetMeanAnomaly(DateTime when)
			{
				double t = (when - epoch).TotalSeconds;
				double ret = M + (t * 2 * Math.PI / T);
				while (ret > 2*Math.PI) { ret -= 2 * Math.PI; }
				while (ret < 0) { ret += 2 * Math.PI; }
				return ret;
			}
			public double MeanToEccentricAnomaly_expensive(double M)
			{
				return BinarySolveMonotonic((double x) => EccentricToMeanAnomaly(x) - M, 0, 2 * Math.PI, 0.0001);
			}
			public double GetEccentricAnomaly_expensive(DateTime when)
			{
				double M = GetMeanAnomaly(when);
				//double E = BinarySolveMonotonic((double x) => EccentricToMeanAnomaly(x) - M, 0, 2 * Math.PI, 0.0001);
				double E = MeanToEccentricAnomaly_expensive(M);
				return E;
			}
			public double GetTrueAnomaly_expensive(DateTime when)
			{
				return EccentricToTrueAnomaly(GetEccentricAnomaly_expensive(when));
			}
			public double TimeToMeanAnomaly(double Mdest)
			{
				double Mdiff = Mdest - M;
				while (Mdiff < 0) { Mdiff += 2 * Math.PI; }
				while (Mdiff > 2 * Math.PI) { Mdiff -= 2 * Math.PI; }
				return (Mdiff * T / (2 * Math.PI));
			}
			public double TimeToEccentricAnomaly(double E)
			{
				double M2 = EccentricToMeanAnomaly(E);
				return TimeToMeanAnomaly(M2);
			}
			public double TimeToTrueAnomaly(double f)
			{
				double M2 = TrueToMeanAnomaly(f);
				return TimeToMeanAnomaly(f);
			}
			public double TimeToAp()
			{
				double r = (T / 2) - tpe;
				while (r < 0) { r += T; }
				while (r > T) { r -= T; }
				return r;
			}
			public double TimeToPe()
			{
				return T - tpe;
			}
			public void UpdateFromTime_expensive(Orbit orb, DateTime when)
			{
				double dt = (when - orb.epoch).TotalSeconds;
				a = orb.a;
				e = orb.e;
				w = orb.w;
				Om = orb.Om;
				i = orb.i;
				this.Ap = orb.Ap;
				this.Pe = orb.Pe;
				this.Area = orb.Area;
				this.b = orb.b;
				this.epoch = when;
				this.nMeanMotion = orb.nMeanMotion;
				this.normal = orb.normal;
				this.p = orb.p;
				this.T = orb.T;
				this.basis_x = orb.basis_x;
				this.basis_y = orb.basis_y;
				this.basis_z = orb.basis_z;

				this.M = orb.GetMeanAnomaly(when); //orb.M + (2 * Math.PI * dt / T);
				while (M > 2*Math.PI) { this.M -= 2 * Math.PI; }
				this.Ea = MeanToEccentricAnomaly_expensive(this.M); //orb.GetEccentricAnomaly_expensive(when);
				this.v = EccentricToTrueAnomaly(Ea);
				tpe = M / nMeanMotion;
			}
			//MatrixD GetTransform()
			//{
			//	QuaternionD qpe = QuaternionD.CreateFromAxisAngle(normal, w); // rotate about normal by argument of periapsis
			//	Vector3D van = new Vector3D(Math.Cos(Om), Math.Sin(Om), 0); // ascending node vector
			//	Vector3D vpe = qpe * van; // i/x basis will be major axis
			//	Vector3D vlat = -(vpe.Cross(normal).Normalized()); // j/y basis will be parallel to latus rectum, minor axis, and directrix
			//	MatrixD m = new MatrixD();
			//	m.M11 = vpe.X; m.M21 = vpe.Y; m.M31 = vpe.Z; m.M41 = 0;
			//	m.M12 = vlat.X; m.M22 = vlat.Y; m.M32 = vlat.Z; m.M42 = 0;
			//	m.M13 = normal.X; m.M23 = normal.Y; m.M33 = normal.Z; m.M43 = 0;
			//	m.M14 = m.M24 = m.M34 = 0;
			//	m.M44 = 1;
			//	return m;
			//}
			//public Vector3D GetPositionFromTrueAnomaly(double f)
			//{
			//	double E = TrueToEccentricAnomaly(f);
			//	return GetPositionFromTrueAndEcc(f, E);
			//}
			//public Vector3D GetPositionFromEccentricAnomaly(double E)
			//{
			//	double f = EccentricToTrueAnomaly(E);
			//	return GetPositionFromTrueAndEcc(f, E);
			//}
			//public Vector3D x_GetPositionFromTrueAndEcc(double f, double E)
			//{
			//	// solve in the x/y plane before transforming to real 3d coords
			//	// origin is the right focus, i.e. the planet
			//	// equation for E in point slope form, point at (-c, 0) and slope angle sin(E)/cos(E)
			//	double Eslope = Math.Tan(E);
			//	double fslope = Math.Tan(f);
			//	double x = (Eslope * c) / (fslope - Eslope);
			//	double y = fslope * x;
			//	return (x * basis_x) + (y * basis_y);
			//}
			public Vector3D GetPositionFromEccentricAnomaly(double E)
			{
				//Vector3D ret = new Vector3D();
				double oa2 = 1.0 / a; oa2 *= oa2;
				double ob2 = 1.0 / b; ob2 *= ob2;
				double sinE = Math.Sin(E);
				double cosE = Math.Cos(E);
				double r = Math.Sqrt(1.0 / ((oa2 * cosE * cosE) + (ob2 * sinE * sinE)));
				double x = r * cosE;
				double y = r * sinE;
				// TODO: Translate from focus to center
				Vector3D local = (x * basis_x) + (y * basis_y);
				Vector3D ret = (local.X * equinox) + (local.Y * eastOfEquinox) + (local.Z * north);
				return ret;
			}
			public PosVel GetPosVelAtEccentricAnomaly(double E)
			{
				PosVel ret = new PosVel();
				//double f = EccentricToTrueAnomaly(E);
				Vector3D pa = GetPositionFromEccentricAnomaly(E);
				ret.pos = pa;
				Vector3D pb = GetPositionFromEccentricAnomaly(E + 0.0001);
				double ta = TimeToEccentricAnomaly(E);
				double tb = TimeToEccentricAnomaly(E + 0.0001);
				double dt = tb - ta;
				if (dt < 0) { dt += T; }
				ret.vel = (pb - pa) / dt;
				return ret;
			}
			public PosVel GetPosVelAtTrueAnomaly(double f)
			{
				double E = TrueToEccentricAnomaly(f);
				return GetPosVelAtEccentricAnomaly(E);
			}
		} // class Orbit


		delegate double MonotonicFunction(double x);
		static double BinarySolveMonotonic(MonotonicFunction f, double min, double max, double tolerance)
		{
			double x = (min + max) / 2;
			//double step = (max - min) / 2;
			double y;
			double slope = (f(x+((max-min)/1000.0)) - f(x)) * 1000;
			while (max - min > tolerance)
			{
				x = (min + max) / 2;
				y = f(x);
				if (Math.Sign(y) == Math.Sign(slope)) { max = x; }
				else { min = x; }
			}
			return x;
		}


		public struct ThrustRatio
		{
			public IMyThrust thruster;
			public float ratio;
		}
		
		List<IMyThrust> allThrust = new List<IMyThrust>();
		List<IMyThrust> remThrust = new List<IMyThrust>();
		Vector3D xhat = new Vector3D(1, 0, 0);
		Vector3D yhat = new Vector3D(0, 1, 0);
		Vector3D zhat = new Vector3D(0, 0, 1);
		public void BalanceThrust(Vector3I directionRelativeToCockpit, ref List<ThrustRatio> outlist)
		{
			G.GetBlocksOfType(allThrust);
			remThrust.Clear();
			foreach (var t in allThrust)
			{
				if (t.GridThrustDirection.Dot(ref directionRelativeToCockpit) > 0.3) { }
			}
		}

		public delegate double Operation(double lhs, double rhs);
		//public static bool rowOp(double[,] m, int iRow, double[] rhs, Operation op)
		//{
		//	int len = m.Length; //.GetLength(1);
		//	// live dangerously, who needs bounds checks, it's slow
		//	//if (len != rhs.GetLength(1)) { return false; }
		//	for (int ic = 0; ic < len; ic++)
		//	{
		//		//m[iRow][ic] = op(m[iRow][ic], rhs[ic]);
		//		m[iRow,ic] = op(m[iRow,ic], rhs[ic]);
		//	}
		//	return true;
		//}
		//public static void scalarOp(double[] v, double s, Operation op)
		//{
		//	int len = v.Length;
		//	for (int ic = 0; ic < len; ic++)
		//	{
		//		v[ic] = op(v[ic], s);
		//	}
		//}
		public static double op_div(double lhs, double rhs) { return lhs / rhs; }
		public static double op_mul(double lhs, double rhs) { return lhs * rhs; }
		public static double op_add(double lhs, double rhs) { return lhs + rhs; }
		public static double op_sub(double lhs, double rhs) { return lhs - rhs; }
		public static double op_assign(double lhs, double rhs) { return rhs; }

		// copy array
		//void cpa(ref double[] src, ref double[] dest)
		//{
		//	for (int i = 0; i < src.Length; i++) { dest[i] = src[i]; }
		//}
		//static double[] tmprow = new double[1];
		//public static void getRow(double [,] m, int iRow, double[] vout)
		//{
		//	int len = m.GetLength(1);
		//	for (int ic = 0; ic < len; ic++) { vout[ic] = m[iRow,ic]; }
		//}
		//public static void putRow(double[,] m, int iRow, double[] vin)
		//{
		//	int len = m.GetLength(1);
		//	for (int ic = 0; ic < len; ic++) { m[iRow, ic] = vin[ic]; }
		//}
		//static double[] tmprow2 = new double[1];
		public static void rowOp(double[,] m, int iDestRow, Operation op, int iSrcRow, double scalarMultiplier)
		{
			int len = m.GetLength(1);
			//if (len != tmprow2.Length) { tmprow2 = new double[len]; }
			//getRow(m, iSrcRow, tmprow2);
			//for (int i = 0; i < len; i++) { tmprow2[i] *= smul; }
			for (int i = 0; i < len; i++) { m[iDestRow,i] = op(m[iDestRow,i], m[iSrcRow,i] * scalarMultiplier); }
		}
		public delegate void ReportProgress(string s);
		public static void NullReportFn(string s) { }
		public static readonly double[] primes10 = new double[]{ 2,3,5,7,11,13,17,19,23,29 };
		public static bool ReducedRowEchelonForm(double[,] m, ReportProgress pr) // [R][C]
		{
			//Matrix m{ *this};
			int R = m.GetLength(0);
			int C = m.GetLength(1);
			string prs;
			//if (tmprow.Length != C) { tmprow = new double[C]; }

			// Check for zeroes on the diagonal
			for (int ir = 0; ir < R; ir++)
			{
				if (m[ir,ir] == 0)
				{
					bool found = false;
					for (int ir2 = 0; ir2 < R; ir2++)
					{
						if (m[ir2,ir] != 0)
						{
							prs = $"M{ir+1} += M{ir2+1} (zero on diagonal check)";
							rowOp(m, ir, op_add, ir2, primes10[ir2]);
							pr(prs);
							found = true;
							//break;
						}
					}
					if (!found)
					{
						pr("Matrix contains a column of all zeroes, rref can not be solved.");
						return false;
					}
				}
			}

			// Lower triangle
			for (int ir = 0; ir < R; ir++)
			{
				//m.row(ir) /= m.at(ir, ir);
				
				//getRow(m, ir, tmprow);
				//scalarOp(tmprow, m[ir,ir], op_div);
				//putRow(m, ir, tmprow);

				// First divide by the diagonal element so it becomes 1
				prs =$"M{ir+1} /= {m[ir,ir]}";
				rowOp(m, ir, op_assign, ir, 1.0 / m[ir, ir]);
				pr(prs);

				for (int sr = ir + 1; sr < R; sr++)
				{
					// lower triangle
					//m.row(sr) -= m.row(ir) * m.at(sr, ir);

					//getRow(m, ir, tmprow);
					//scalarOp(tmprow, m[sr,ir], op_mul);
					//rowOp(m, sr, tmprow, op_sub);

					// M[sr] - M[ir]*M[sr,ir]
					//if (m[ir, sr] * m[sr, ir] != m[sr, sr])
					{
						prs = $"M{sr + 1} -= M{ir + 1} * {m[sr, ir]} (M{sr + 1},{ir + 1})";
						rowOp(m, sr, op_sub, ir, m[sr, ir]);
						pr(prs);
					}
					//else
					{
						// TODO: We're about to set a diagonal element to 0...
					}
				}
				for (int sr = ir + 1; sr < R; sr++)
				{
					if (m[sr, sr] == 0)
					{
						// We just made a diagonal zero, we need to fix that somehow
						bool found = false;
						for (int sr2 = sr + 1; sr2 < R; sr2++)
						{
							if (m[sr2, sr] != 0)
							{
								prs = $"M{sr + 1} += M{sr2 + 1} (diagonal zero fix 2)";
								rowOp(m, sr, op_add, sr2, 1);
								pr(prs);
								found = true;
								break;
							}
						}
						if (!found)
						{
							// TODO: Try other strategies?
							pr($"Don't know how to fix the zero in diagonal at M{sr + 1}");
							return false;
						}
					}
				}
			}
			for (int ir = R - 1; ir >= 0; ir--)
			{
				for (int sr = ir - 1; sr >= 0; sr--)
				{
					// upper triangle
					//m.row(sr) -= m.row(ir) * m.at(sr, ir);

					//getRow(m, ir, tmprow);
					//scalarOp(tmprow, m[sr,ir], op_mul);
					//rowOp(m, sr, tmprow, op_sub);

					prs = ($"M{sr+1} -= M{ir+1} * {m[sr, ir]} (M{sr+1},{ir+1})");
					rowOp(m, sr, op_sub, ir, m[sr, ir]);
					pr(prs);
				}
			}
			return true;
			//return m;
		} // ReducedRowEchelonForm()

		void GetThrusterRatios(Vector3D direction, ref List<ThrustRatio> outlist)
		{
			direction.Normalize();
			outlist.Clear();
			G.GetBlocksOfType(allThrust);
			MatrixD ori = cockpit.WorldMatrix.GetOrientation();
			Vector3D shipx = mul(ori, xhat);
			Vector3D shipy = mul(ori, yhat);
			Vector3D shipz = mul(ori, zhat);

			// sum( c*[x, y, z] ) / sqrt(x^2 + y^2 + z^2) = [direction]
			// sum( c*[x, y, z] ) = sqrt(x^2 + y^2 + z^2) * [direction]
			Vector3D CenterOfThrustStart = new Vector3D(0,0,0);
			//Vector3D ThrustDir = new Vector3D(0, 0, 0);
			Vector3D ThrustTotal = new Vector3D(0,0,0);
			Vector3D ThrustTotalDir = new Vector3D(0, 0, 0);
			Vector3D am = new Vector3D(0, 0, 0); // angular momentum
			double ThrustMagnitudeTotal = 0;
			Vector3D com = cockpit.CenterOfMass;
			foreach (var t in allThrust)
			{
				//if (t.GridThrustDirection)
				if (!t.IsWorking || !t.Enabled) { continue; }
				// TODO: Use world matrix * local ... z?
				Vector3D tdir = (t.GridThrustDirection.X * shipx) + (t.GridThrustDirection.Y * shipy) + (t.GridThrustDirection.Z * shipz);
				tdir.Normalize();
				Vector3D tv = t.MaxEffectiveThrust * tdir;
				double tdot = tdir.Dot(direction);
				ThrustRatio tr = new ThrustRatio();
				tr.thruster = t;
				tr.ratio = 0;
				if (tdot > 0.3) // A tetrahedral arrangement of 109.5 degrees (though I don't know how in tf you would set that up in SE)
					// results in a dot product of at least .34
				{
					ThrustMagnitudeTotal += t.MaxEffectiveThrust;
					ThrustTotal += tv;
					//CenterOfThrustStart += (t.WorldMatrix.Translation - cockpit.WorldMatrix.Translation) * t.MaxEffectiveThrust;
					CenterOfThrustStart += t.WorldMatrix.Translation * t.MaxEffectiveThrust;
					tr.ratio = 1;
					Vector3D tpos = t.WorldMatrix.Translation;
					am += (tpos - com).Cross(tv);
				}
				outlist.Add(tr);
			}
			ThrustTotalDir = ThrustTotal.Normalized();
			if (ThrustMagnitudeTotal > 0)
			{
				CenterOfThrustStart /= ThrustMagnitudeTotal;
				//ThrustDir.Normalize();
			}
			else
			{
				// TODO: Alarm, unable to keep station, orbital decay detected.
				// Bail out at this point
				Echo("Warning: Can not find adequate thrust for station keeping maneuver!");
				return;
			}

			/*
			[ x1 x2 x3 x4 x5 ]   [c1]   [tx]
			[ y1 y2 y3 y4 y5 ] * [c2] = [ty]
			[ z1 z2 z3 z4 z5 ]   [c3]   [tz]
			                     [c4]
			                     [c5]

			[M]^-1 * [M] * [c] = [c] = [M]^-1 * [t]
			... except you can't invert a non square matrix
			Could we just fill with zeroes to make square and do a rref?

			[ x1 x2 x3 x4 x5   tx ]
			[ y1 y2 y3 y4 y5   ty ]
			[ z1 z2 z3 z4 z5   tz ]
			[  0  0  0  0  0    0 ]
			[  0  0  0  0  0    0 ]

			 | rref -->

			[ 1 0 0 0 0    c1 ]
			[ 0 1 0 0 0    c2 ]
			[ 0 0 1 0 0    c3 ]
			[ 0 0 0 1 0    c4 ]
			[ 0 0 0 0 1    c5 ]

			*/
			//double[][] x = new double[5][5];
			//x.GetLength(0);

			foreach (var tr in outlist)
			{

			}











			while (ThrustTotalDir.Dot(direction) < 0.99)
			{
				Vector3D adjust = direction - ThrustTotalDir;
				IMyThrust matchThruster;
				double matchDot = 0;
				Vector3D matchTv = new Vector3(0,0,0);
				// Find the thruster which makes the biggest correction towards the direction we want to go, and throttle it back
				foreach (var tr in outlist)
				{
					// TODO: DRY?
					IMyThrust t = tr.thruster;
					if (!t.IsWorking || !t.Enabled) { continue; }
					Vector3D tdir = (t.GridThrustDirection.X * shipx) + (t.GridThrustDirection.Y * shipy) + (t.GridThrustDirection.Z * shipz);
					tdir.Normalize();
					Vector3D tv = t.MaxEffectiveThrust * tdir;
					double tdot = tdir.Dot(direction);
					//Vector3D = cockpit.CenterOfMass -
					double test = tv.Dot(adjust);
					if (test > matchDot)
					{
						matchDot = test;
						matchThruster = t;
						matchTv = tv;
					}
				}
				// Now, how much to throttle it... find max of dot product
				// maximize: ((ThrustTotal - matchTv) + (s * matchTv)).normalized() DOT direction
				// T = (ThrustTotal - matchTv)
				// M = matchTv
				// D = direction
				// s = derivative parameter, solve for this
				// d/ds * sqrt((M.x*s + T.x)^2 + (M.x*s + T.x)^2)
				// Screw all that, I think just a linear interpolation is fine
				Vector3D A = ThrustTotal - matchTv;
				Vector3D B = ThrustTotal;
				Vector3D D = direction;
				double Adot = A.Dot(D) / A.Length();
				double Bdot = B.Dot(D) / B.Length();
				Vector3D ABHalf = (A + 0.5 * (B - A));
				double ABHalfDot = ABHalf.Dot(D) / ABHalf.Length();
				//if (ABHalf)
				// maximize: A + s*(B-A) DOT direction (D) /  <-- d/ds
				// d/ds  (Ax + s*Bx - s*Ax)*Dx + (Ay + s*By - s*Ay)*Dy + (Az + s*Bz - s*Az)*Dz = 0
				double P = (D.X * B.X) - (D.X * A.X) + (D.Y * B.Y) - (D.Y * A.Y) + (D.Z * B.Z) - (D.Z * A.Z);
				double Q = (D.X * A.X) + (D.Y * A.Y) + (D.Z + A.Z);
				// d/ds (P*s + Q) = P
				// d/ds  Ax*Dx + Dx*Bx*s
			}
		}



	} // class Program
} // namespace
