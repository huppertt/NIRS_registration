using System;
using MathNet.Numerics.LinearAlgebra;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra.Factorization;
/*
Iterative Closest Point

Performs the iterative closest point algorithm. A point cloud 'dynamicPointCloud' is transformed such that
it best "matches" the point cloud 'staticPointCloud'.

this program uses the method outlined in [1].

1. for each point in the dynamic point cloud, find the closest point in the static point cloud.
2. solve for an affine transform which minimizes the distance between all dynamic points and their respective static points.
3. transform the dynamic points.
4. goto -> 1 until stopping conditions are met.

adapted from https://github.com/Gregjksmith/Iterative-Closest-Point/

[1] Arun, K. Somani, Thomas S. Huang, and Steven D. Blostein. "Least-squares fitting of two 3-D point sets." 
	IEEE Transactions on pattern analysis and machine intelligence 5 (1987): 698-700.

*/

namespace CloudMapping
{
	public class Head3D
	{

		private List<Vector<double>> staticPointCloud;
		private List<Vector<double>[]> rawpoints;
		private List<Matrix<double>> rotations;
		private Matrix<double> cumrotation;

		public Head3D()
		{
			reset();
		}

		public void reset()
        {
			staticPointCloud = new List<Vector<double>>();
			rawpoints = new List<Vector<double>[]>();
			rotations = new List<Matrix<double>>();
			cumrotation = Matrix<double>.Build.DenseIdentity(4);
			rotations.Add(cumrotation);
		}

		public void AddPoints(List<Vector<double>> newdata)
        {
			AddPoints(newdata.ToArray());
        }




		public void AddPoints(Vector<double>[] newdata)
		{
			rawpoints.Add(newdata);

			for (int i = 0; i < newdata.Length; i++)
			{
				newdata[i] = applyTform(newdata[i], cumrotation);
			}

			Matrix<double> localR = iterClosestPt(staticPointCloud, newdata);
			rotations.Add(localR);

			cumrotation = cumrotation * localR;
			for (int i = 0; i < newdata.Length; i++)
			{
				staticPointCloud.Add(applyTform(newdata[i], localR));
			}



		}

		private Vector<double>[] applyTform(Vector<double>[] v, Matrix<double> m)
        {
			for(int i=0; i < v.Length; i++)
            {
				v[i] = applyTform(v[i], m);
            }
			return v;
        }

		private Vector<double> applyTform(Vector<double> v, Matrix<double> m)
		{
			Vector<double> v2 = Vector<double>.Build.Dense(4);
			v2[0] = v[0];
			v2[1] = v[1];
			v2[2] = v[2];
			v2[3] = 0;

			v2 = v2 * m;
			v[0] = v2[0];
			v[1] = v2[1];
			v[2] = v2[2];
			return v;


		}

		private Vector<double> computecloudmean(List<Vector<double>> staticPointCloud)
		{
			Vector<double> mean = Vector<double>.Build.Dense(new double[] { 0, 0, 0 });
			for (int i = 0; i < staticPointCloud.Count; i++)
			{
				mean = mean + staticPointCloud[i];
			}
			return mean / staticPointCloud.Count;
		}

		private Vector<double> findclosest(List<Vector<double>> olddata, Vector<double> newdata)
		{
			int index = 0;
			double cost = Math.Sqrt((olddata[0] - newdata).L2Norm());
			for (int i = 0; i < olddata.Count; i++)
			{
				double c = Math.Sqrt((olddata[i] - newdata).L2Norm());
				if (cost > c)
				{
					cost = c;
					index = i;
				}
			}

			return olddata[index];

		}

		private Vector<double> computecloudmean(Vector<double>[] dynamicPointCloud)
		{
			Vector<double> mean = Vector<double>.Build.Dense(new double[] { 0, 0, 0 });
			for (int i = 0; i < dynamicPointCloud.Length; i++)
			{
				mean = mean + dynamicPointCloud[i];
			}
			return mean / dynamicPointCloud.Length;
		}


		private Matrix<double> iterClosestPt(List<Vector<double>> oldata, Vector<double>[] newdata)
		{

			const int maxIterations = 400;
			const int numRandomSamples = 400;
			const double eps = 1e-8;
			double cost = 1.0;

			Vector<double> midOld = computecloudmean(oldata);
			Vector<double> midNew;
			Random r = new Random();

			Matrix<double> rotationMatrix = Matrix<double>.Build.DenseIdentity(4);

			for (int iter = 0; iter < maxIterations && Math.Abs(cost) > eps; iter++)
			{
				midNew = computecloudmean(newdata);
				Matrix<double> U = Matrix<double>.Build.Dense(3, 3, 0);

				for (int i = 0; i < numRandomSamples; i++)
				{
					// sample the dynamic point cloud
					Vector<double> p = newdata[r.Next(0, newdata.Length)];

					// get the closest point in the static point cloud
					Vector<double> x = findclosest(oldata, p);

					Vector<double> qd = p - midNew;
					Vector<double> qs = x - midOld;

					Matrix<double> w = qd.OuterProduct(qs);

					U = U + w;

					cost += Math.Sqrt((x - applyTform(p, rotationMatrix)).L2Norm());
				}

				Svd<double> svd = U.Svd();
				Matrix<double> V = svd.VT;

				midNew = applyTform(midNew, rotationMatrix);

				Matrix<double> matrix = U * V;
				rotationMatrix[0, 0] = matrix[0, 0];
				rotationMatrix[0, 1] = matrix[0, 1];
				rotationMatrix[0, 2] = matrix[0, 2];
				rotationMatrix[0, 3] = 0;
				rotationMatrix[1, 0] = matrix[1, 0];
				rotationMatrix[1, 1] = matrix[1, 1];
				rotationMatrix[1, 2] = matrix[1, 2];
				rotationMatrix[1, 3] = 0;
				rotationMatrix[2, 0] = matrix[2, 0];
				rotationMatrix[2, 1] = matrix[2, 1];
				rotationMatrix[2, 2] = matrix[2, 2];
				rotationMatrix[2, 3] = 0;
				rotationMatrix[3, 0] = midOld[0] - midNew[0];
				rotationMatrix[3, 1] = midOld[1] - midNew[1];
				rotationMatrix[3, 2] = midOld[2] - midNew[2];
				rotationMatrix[3, 3] = 1;

				newdata = applyTform(newdata, rotationMatrix);

			}

			return rotationMatrix;
		}






	}
}


	


