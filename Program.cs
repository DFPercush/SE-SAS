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

namespace IngameScript
{
	partial class Program : MyGridProgram
	{

		#region MDK Preserve
		// ==== CONFIG ====
		// This is probably the only thing you'll need to change
		const string ControlBlock = "Fighter Cockpit";

		readonly Vector3D cockpitLocalForward = new Vector3D (0, 0, -1);
		readonly Vector3D cockpitLocalUp = new Vector3D (0, 1, 0);
		readonly Vector3D cockpitLocalRight = new Vector3D (1, 0, 0);
		const double BaseGyroSensitivity = 0.2;
		const double TorquePerGyroMultiplier = 60000;

		// ==== END CONFIG ====
		#endregion

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
		public Program()
		{
			G = GridTerminalSystem;
			cockpit = G.GetBlockWithName(ControlBlock) as IMyShipController;
			if (cockpit == null)
			{
				Echo("Error: No ship control block found.");
				return;
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

		} // constructor

		public void Save()
		{
		} // Save

		public void Main(string arg, UpdateType updateSource)
		{
			bool planetPresent = false;
			if (updateSource == UpdateType.Update1)
			{
				Vector3D pp;
				planetPresent = cockpit.TryGetPlanetPosition(out pp);
				if (!planetPresent)
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
				Vector3D planetMe = cockpit.WorldMatrix.Translation - pp;
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
				if (mode != Mode.DisplayOnly)
				{
					foreach (var g in gyt)
					{
						g.g.GyroOverride = true;
						g.setYaw(g.g, pidYaw.Output);
						g.setPitch(g.g, pidPitch.Output);
						//g.setYaw(g.g, angleDiff * dYaw * BaseGyroSensitivity);
						//g.setPitch(g.g, angleDiff * dPitch * BaseGyroSensitivity);
					}
				}

































				Echo($"vel={ppv(vPro)}");
				Echo($"fwd={ppv(wvFwd)}");
				Echo($"up={ppv(wvUp)}");
				Echo($"right={ppv(wvRight)}");
				//Echo($"vFacing = \r\n{ppv(vFacing)}");
				//Echo($"vDest = \r\n{ppv(vDest)}");
				//Echo($"angleDiff = \r\n{angleDiff}");
				//Echo($"dPitch = {dPitch}");
				//Echo($"dYaw = {dYaw}");
				//Echo($"localToward=\r\n{ppv(localToward)}");































			}
			else
			{
				if (arg.StartsWith("pro"))
				{
					mode = Mode.Prograde;
					Runtime.UpdateFrequency = UpdateFrequency.Update1;
				}
				else if (arg.StartsWith("ret"))
				{
					mode = Mode.Retrograde;
					Runtime.UpdateFrequency = UpdateFrequency.Update1;
				}
				else if (arg.StartsWith("nor"))
				{
					mode = Mode.Normal;
					Runtime.UpdateFrequency = UpdateFrequency.Update1;
				}
				else if (arg.StartsWith("ant"))
				{
					mode = Mode.Antinormal;
					Runtime.UpdateFrequency = UpdateFrequency.Update1;
				}
				else if (arg.StartsWith("in"))
				{
					mode = Mode.RadialIn;
					Runtime.UpdateFrequency = UpdateFrequency.Update1;
				}
				else if (arg.StartsWith("out"))
				{
					mode = Mode.RadialOut;
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
					Runtime.UpdateFrequency = UpdateFrequency.Update1;
				}
				//else if (arg.StartsWith("hold"))
				//{
				//	mode = Mode.Disabled
				//}
			}
		} // Main

		void Stop()
		{
			mode = Mode.Disabled;
			Runtime.UpdateFrequency = UpdateFrequency.None;
			foreach (var g in gyros)
			{
				g.Yaw = 0;
				g.Pitch = 0;
				g.Roll = 0;
				g.GyroOverride = false;
			}
			Echo("Stopped");
		}

		// Pretty Print Vector
		string ppv(Vector3D v)
		{
			return $"({Math.Round(v.X, 2)}, {Math.Round(v.Y, 2)}, {Math.Round(v.Z, 2)})";
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








		const Base6Directions.Direction _up = Base6Directions.Direction.Up;
		const Base6Directions.Direction _down = Base6Directions.Direction.Down;
		const Base6Directions.Direction _left = Base6Directions.Direction.Left;
		const Base6Directions.Direction _right = Base6Directions.Direction.Right;
		const Base6Directions.Direction _forward = Base6Directions.Direction.Forward;
		const Base6Directions.Direction _backward = Base6Directions.Direction.Backward;
		static Base6Directions.Direction OppositeDirection(Base6Directions.Direction d)
		{
			switch (d)
			{
				case _up: return _down;
				case _down: return _up;
				case _left: return _right;
				case _right: return _left;
				case _forward: return _backward;
				case _backward: return _forward;
				default: throw new Exception("Unknown Base6Direction");
			}
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

	} // class Program
} // namespace
