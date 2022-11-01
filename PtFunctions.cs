using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using Grasshopper;
using Grasshopper.Kernel.Data;
using System.Linq;
using System.IO;
using System.Threading.Tasks;
using System.Diagnostics;
using System.Threading;
using System.Runtime.Serialization;
using System.Runtime.Serialization.Formatters.Binary;
using Extensions;
using MathWorks.MATLAB.Engine;
using MathWorks.MATLAB.Types;
using Accord.Math;
using System.Xml;
using MathNet.Numerics;

namespace TOtracing
{
    class PtFunctions
    {

        public static double r_min = 1e-6;
        private int x, y;
        private List<Point3d> AllPts;
        Tuple<int, int> MinIdx = new Tuple<int, int>(0, 0);
        private Point3d Spt;
        private int[,] TraceMat;
        private Vector3d unitX = Vector3d.XAxis;
        public void MatFEM(double x, double y, int iter, double volfrac, out List<NurbsCurve> Curves)
        {
            Curves = new List<NurbsCurve>();
            int counter = 1;

            this.x = (int)x;
            this.y = (int)y;

            using (dynamic matlab = MATLABEngine.StartMATLAB())
            {
                double[,] resultMat;
                resultMat = matlab.IntTop88(x, y, 0.5, 3.0, 3.5, 1.0);
                double[,] sense = ParticleTrace(resultMat, volfrac, out Curves);

                while (counter < iter)
                {
                    resultMat = matlab.IterTop88(sense, x, y, 0.5, 3.0, 3.5, 1.0);
                    sense = ParticleTrace(resultMat, volfrac, out Curves);
                    counter++;
                }


            }
        }

        private double[,] ParticleTrace(double[,] SensMat, double volfrac, out List<NurbsCurve> CurveList)
        {
            //string path = "SenDire";
            List<NurbsCurve> CorCVS = new List<NurbsCurve>();

            int maxV = x * y;
            double V = maxV * volfrac;

            double[,] PTSenseMat = MatToLoc2D(SensMat);

            MinIdx = PTSenseMat.ArgMin();

            Spt = new Point3d(MinIdx.Item1, MinIdx.Item2, 0);

            TraceMat = new int[x + 2, y + 2];
            TraceMat.SetEdges(1);

            int counter = 0;
            CurveList = new List<NurbsCurve>();
            while (true)
            {
                if (counter != 0)
                {
                    //Spt = new Point3d(299, 1, 0);
                    //Spt = ExplorTracPt(CurveList, SensMat, TraceMat, PTMat);
                }
                List<NurbsCurve> CCuve = SubParTrace(PTSenseMat, Spt, 1);
                V = TraceMat.Sum();

                CorCVS.AddRange(CCuve);

                Interval inter = new Interval(0, 1);
                var crvs = Curve.JoinCurves(CCuve);
                foreach (var curve in crvs)
                {
                    var A = curve.ToNurbsCurve();
                    A.Domain = inter;
                    CurveList.Add(A);
                }


                if (V >= 55)
                    break;
                counter++;
            }

            var TraceMatrix = DenseSet(TraceMat);

            return TraceMatrix;

        }

        private static double[,] DenseSet(int[,] TraceMatrix, double r_max = 1)
        {
            var n = TraceMatrix.n();
            var m = TraceMatrix.m();
            double[,] TrMat = new double[n, m];
            for (int i = 0; i < n; i++)
            {
                for (int j = 0; j < m; j++)
                {
                    if (TraceMatrix[i, j] == 1)
                        TrMat[i, j] = r_max;
                    else
                        TrMat[i, j] = r_min;
                }
            }
            return TrMat;
        }

        private double[,] MatToLoc2D(double[,] SensMatrix)
        {
            AllPts = new List<Point3d>();

            double[,] PTArr = new double[x + 2, y + 2];

            for (int i = 0; i < x; i++)
            {
                for (int j = 0; j < y; j++)
                {
                    PTArr[i + 1, j + 1] = SensMatrix[y - 1 - j, i];
                    Point3d MatLoc = new Point3d(y - 1 - j, i, 0);
                    //LocPt = MatLoc + new Point3d(0.5, 0.5, 0);
                    AllPts.Add(MatLoc);
                }
            }
            return PTArr;
        }

        private List<NurbsCurve> SubParTrace(double[,] SensMatrix, Point3d SourcePt, double Radii)
        {
            List<NurbsCurve> CoreCurves = new List<NurbsCurve>();

            bool Pathfound = false;
            int counter = 0;
            List<Tuple<int, int, double>> BoundarySens = new List<Tuple<int, int, double>>();

            while (!Pathfound)
            {
                Vector3d PreDir = new Vector3d();
                if (CoreCurves.Count != 0)
                    PreDir = CoreCurves[CoreCurves.Count - 1].TangentAtStart;

                int[,] NeighbourMat = GetNeighbours(TraceMat, SourcePt);
                double[,] LocalSens = GetNeighbours(SensMatrix, SourcePt);

                //Calculate local costs
                var CostMat = ComputeCostForNeighbours(NeighbourMat, LocalSens, PreDir);

                if (CostMat.Sum() == 0)
                {
                    Pathfound = true;
                    break;
                }

                //Find Min and set in the trace matrix
                MinIdx = CostMat.ArgMin();
                Line L1 = new Line(SourcePt, SourcePt + new Point3d(MinIdx.Item1 - 1, MinIdx.Item2 - 1, 0));
                NurbsCurve C1 = L1.ToNurbsCurve(); C1.Reparameterize();
                CoreCurves.Add(C1);
                var TurnAngle = Vector3d.VectorAngle(L1.UnitTangent, unitX) * 180.0 / Math.PI;
                var n1 = L1.Direction;
                var n2 = L1.Direction;

                n1.Rotate(Math.PI / 2.0, Vector3d.ZAxis);
                n2.Rotate(-Math.PI / 2.0, Vector3d.ZAxis);
                if ((int)TurnAngle % 90 == 0)
                {
                    BoundarySens.Add(new Tuple<int, int, double>((int)(L1.ToX + n1.X), (int)(L1.ToY + n1.Y), SensMatrix[(int)(L1.ToX + n1.X), (int)(L1.ToY + n1.Y)]));
                    BoundarySens.Add(new Tuple<int, int, double>((int)(L1.ToX + n2.X), (int)(L1.ToY + n2.Y), SensMatrix[(int)(L1.ToX + n2.X), (int)(L1.ToY + n2.Y)]));
                }
                else
                {
                    BoundarySens.Add(new Tuple<int, int, double>((int)(L1.ToX + n1.X), (int)(L1.ToY + n1.Y), SensMatrix[(int)L1.ToX, (int)L1.ToY]));
                    BoundarySens.Add(new Tuple<int, int, double>((int)(L1.ToX + n2.X), (int)(L1.ToY + n2.Y), SensMatrix[(int)L1.ToX, (int)L1.ToY]));
                    BoundarySens.Add(new Tuple<int, int, double>((int)((L1.ToX + L1.FromX) / 2 + n1.X / 2), (int)((L1.ToY + L1.FromY) / 2 + n1.Y / 2),
                        SensMatrix[(int)((L1.ToX + L1.FromX) / 2 + n1.X / 2), (int)((L1.ToY + L1.FromY) / 2 + n1.Y / 2)]));
                    BoundarySens.Add(new Tuple<int, int, double>((int)((L1.ToX + L1.FromX) / 2 + n2.X / 2), (int)((L1.ToY + L1.FromY) / 2 + n2.Y / 2),
                        SensMatrix[(int)((L1.ToX + L1.FromX) / 2 + n2.X / 2), (int)((L1.ToY + L1.FromY) / 2 + n2.Y / 2)]));
                }
                

                TraceMat[(int)L1.FromX, (int)L1.FromY] = 1;
                TraceMat[(int)L1.ToX, (int)L1.ToY] = 1;

                SourcePt = C1.PointAtEnd;
                counter++;

            }
            return CoreCurves;
        }

        private void SetBoundaryPoints(Line line, int thickness)
        {



        }

        private T[,] GetNeighbours<T>(T[,] Matrix, Point3d idx)
        {
            int i1 = (int)idx.X - 1;
            int i2 = (int)idx.X + 1;
            int j1 = (int)idx.Y - 1;
            int j2 = (int)idx.Y + 1;

            T[,] NeighbourMat = Matrix.Get(i1, i2 + 1, j1, j2 + 1);
            return NeighbourMat;
        }

        //private double[,] GetNeighbours(double[,] Matrix, Point3d idx)
        //{
        //    int i1 = (int)idx.X - 1;
        //    int i2 = (int)idx.X + 1;
        //    int j1 = (int)idx.Y - 1;
        //    int j2 = (int)idx.Y + 1;

        //    double[,] NeighbourMat = Matrix.Get(i1, i2 + 1, j1, j2 + 1);
        //    return NeighbourMat;
        //}


        //private int[,] GetNeighbours(int[,] Matrix, Point3d idx)
        //{
        //    int i1 = (int)idx.X - 1;
        //    int i2 = (int)idx.X + 1;
        //    int j1 = (int)idx.Y - 1;
        //    int j2 = (int)idx.Y + 1;

        //    int[,] NeighbourMat = Matrix.Get(i1, i2 + 1, j1, j2 + 1);
        //    return NeighbourMat;
        //}

        private static double[,] ComputeCostForNeighbours(int[,] NeighbourMat, double[,] SensMatrix, Vector3d PreDir)
        {

            //Trace Cost
            double[,] TraceCost = new double[NeighbourMat.GetLength(0), NeighbourMat.GetLength(1)];

            for (int i = 0; i < NeighbourMat.GetLength(0); i++)
            {
                for (int j = 0; j < NeighbourMat.GetLength(1); j++)
                {
                    double alpha = 0;
                    if (i == 1 && j == 1)
                        continue;
                    if (NeighbourMat[i, j] == 1)
                        continue;
                    if (SensMatrix[i, j] == 0)
                        continue;

                    #region  angle cost
                    Vector3d dir = new Point3d(i, j, 0) - new Point3d(1, 1, 0);
                    double VecAng = Vector3d.VectorAngle(dir, PreDir) * (180 / Math.PI);
                    if (VecAng >= 90)
                    {
                        continue;
                    }

                    if (VecAng < 90 && VecAng > 40)
                    {
                        alpha = 0.8;
                    }
                    if (VecAng < 40)
                    {
                        alpha = 1;
                    }

                    TraceCost[i, j] = alpha * SensMatrix[i, j];
                    #endregion


                    #region Neighbour Sensitivity Gradient Cost
                    #endregion
                }
            }

            return TraceCost;
        }
    }
}

//private double[,,] MatToLoc3D(double[,] SensMatrix)
//{
//    AllPts = new List<Point3d>();
//    double[,,] PTArr = new double[x, y, 1];

//    for (int i = 0; i < x; i++)
//    {
//        for (int j = 0; j < y; j++)
//        {
//            double PtMat = SensMatrix[j, (x - 1) - i];
//            Point3d MatLoc = new Point3d(j, (x - 1) - i, 0);
//            //LocPt = MatLoc + new Point3d(0.5, 0.5, 0);
//            PTArr[i, j, 0] = PtMat;
//            AllPts.Add(MatLoc);
//        }
//    }
//    return PTArr;
//}