using System;
using IngameScript;
using VRageMath;

namespace SasTest
{
	class TestConsole
	{

		static double[,] m = new double[5, 6];
		static double[,] morig = new double[5,6];

		static void progress(string s)
		{
			Console.WriteLine(s);
			//for (int r = 0; r < 5; r++)
			//{
			//	for (int c = 0; c < 6; c++)
			//	{
			//		Console.Write($"{m[r,c],12:F3}, ");
			//	}
			//	Console.Write("\n");
			//}
			printMatrix(m);
			Console.Write("\n");
		}
		static void printMatrix(double[,] m)
		{
			for (int r = 0; r < 5; r++)
			{
				for (int c = 0; c < 6; c++)
				{
					Console.Write($"{m[r, c],12:F3}, ");
				}
				Console.Write("\n");
			}
		}
		static void Main(string[] args)
		{
			TimeSpan t = TimeSpan.FromSeconds(182);
			Console.WriteLine($"{t.Hours:00}:{t.Minutes:00}:{t.Seconds:00}");
			return;
			//Console.WriteLine(Math.Max(0, Math.Min(1, (float)1)).ToString());
			//return;

			Random rand = new Random();
			m[0, 0] = 1;
			m[1, 0] = 2;
			m[2, 0] = 3;
			m[3, 0] = rand.NextDouble();
			m[4, 0] = rand.NextDouble();

			m[0, 1] = 4;
			m[1, 1] = 5;
			m[2, 1] = 6;
			m[3, 1] = rand.NextDouble();
			m[4, 1] = rand.NextDouble();

			m[0, 2] = 7;
			m[1, 2] = 8;
			m[2, 2] = 9;
			m[3, 2] = rand.NextDouble();
			m[4, 2] = rand.NextDouble();

			m[0, 3] = -1;
			m[1, 3] = -2;
			m[2, 3] = -3;
			m[3, 3] = rand.NextDouble();
			m[4, 3] = rand.NextDouble();

			m[0, 4] = -4;
			m[1, 4] = -5;
			m[2, 4] = -6;
			m[3, 4] = rand.NextDouble();
			m[4, 4] = rand.NextDouble();

			m[0, 5] = -7;
			m[1, 5] = -8;
			m[2, 5] = -9;
			m[3, 5] = rand.NextDouble();
			m[4, 5] = rand.NextDouble();


			for (int r = 0; r < 3; r++)
			{
				for (int c = 0; c < 6; c++)
				{
					m[r, c] = rand.NextDouble();
				}
			}
			for (int r = 3; r < 5; r++)
			{
				for (int c = 0; c < 6; c++)
				{
					m[r, c] = 0;
				}
			}

			Vector3D[] v = new Vector3D[5];
			for (int c = 0; c < 5; c++)
			{
				v[c] = new Vector3D(m[0, c], m[1, c], m[2, c]);
				Console.WriteLine($"v[{c}] = {v[c]}");
			}


			Array.Copy(m, morig, 30);

			Vector3D target = new Vector3D(m[0, 5], m[1, 5], m[2, 5]);
			Console.WriteLine($"target = {target}");
			Console.WriteLine("m = ");
			printMatrix(m);
			Console.WriteLine("\n");

			Program.ReducedRowEchelonForm(m, progress);
			Console.WriteLine("Hello World!");

			// Now test the results
			//Console.WriteLine(" s    *   v      =    sub      |  total");

			Vector3D vsum = new Vector3D(0, 0, 0);
			for (int r = 0; r < 5; r++)
			{
				vsum += v[r] * m[r, 5];
			}
			Console.WriteLine("Results:");
			Console.WriteLine($"  vsum = {vsum}");
			Console.WriteLine($"target = {target}");
			Console.WriteLine($"   diff: {vsum - target}");



			Program.Orbit orb = new Program.Orbit();
			double rtest = 90000;
			orb.UpdateFromPosVel(new Vector3D(rtest, 0, 0), new Vector3D(0, 745, 10), (.73 * 9.81 * rtest * rtest));
			Program.PosVel pv;
			pv = orb.GetPosVelAtEccentricAnomaly(Math.PI);
			Console.WriteLine($"{pv.vel}");
		} // Main()
	} // class TestConsole
} // namespace SasTest
