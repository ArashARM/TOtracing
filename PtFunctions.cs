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
using Accord.Math.Geometry;
using Line = Rhino.Geometry.Line;

namespace TOtracing
{
    class PtFunctions
    {

        public static double r_min = 1e-6;
        private int x, y; //n,m
        private List<Point3d> AllPts;
        Tuple<int, int> MinIdx = new Tuple<int, int>(0, 0);
        private Point3d Spt;
        private int[,] TraceMat;
        private double[,] TopMat;
        private double[,] SensMatrix;
        private double[,] resultMat;
        private Vector3d unitX = Vector3d.XAxis;
        List<Tuple<Point3d, double>> BoundarySens = new List<Tuple<Point3d, double>>();

        public void MatFEM(double x, double y, int iter, double volfrac, out List<NurbsCurve> Curves, out double[,] TopologyMat)
        {
            Curves = new List<NurbsCurve>();
            int counter = 1;

            this.x = (int)x;
            this.y = (int)y;
            string[] names = MATLABEngine.FindMATLAB();

            using (dynamic matlab = MATLABEngine.ConnectMATLAB(names[0]))
            {
                resultMat = matlab.IntTop88(x, y, 0.5, 3.0, 3.5, 1.0);
                double[,] sense = ParticleTrace(resultMat, volfrac, out Curves);

                while (counter < iter)
                {
                    resultMat = matlab.IterTop88(resultMat, x, y, 0.5, 3.0, 3.5, 1.0);
                    sense = ParticleTrace(resultMat, volfrac, out Curves);
                    counter++;
                }
            }
            TopologyMat = TopMat;
        }

        private double[,] ParticleTrace(double[,] SensMat, double volfrac, out List<NurbsCurve> CurveList)
        {
            //string path = "SenDire";
            List<NurbsCurve> CorCVS = new List<NurbsCurve>();
            BoundarySens = new List<Tuple<Point3d, double>>();

            int maxV = (x + 2) * (y + 2);
            double Volume = maxV * volfrac;
            double V = 0;
            SensMatrix = MatToLoc2D(SensMat);

            MinIdx = SensMatrix.ArgMin();

            Spt = new Point3d(MinIdx.Item1, MinIdx.Item2, 0);

            TraceMat = new int[x + 2, y + 2];
            TraceMat.SetEdges(1);

            int counter = 0;
            CurveList = new List<NurbsCurve>();
            while (true)
            {
                if (counter != 0)
                {
                    Spt = BoundarySens[BoundarySens.Count - 1].Item1;
                    BoundarySens.RemoveAt(BoundarySens.Count - 1);
                }
                List<NurbsCurve> CCuve = SubParTrace(Spt, 1);
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
                if (V >= Volume)
                    break;
                counter++;
            }

            var TraceMatrix = DenseSet(TraceMat);
            TopMat = TraceMatrix.Get(1,x+1 ,1 ,y+1);
            resultMat = LocToMat2D(TraceMatrix);


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
        private double[,] LocToMat2D(double[,] SensMatrix)
        {
            AllPts = new List<Point3d>();

            var Sen = SensMatrix.Get(1, x + 1, 1, y + 1);

            double[,] PTArr = new double[y, x];

            for (int i = 0; i < y; i++)
            {
                for (int j = 0; j < x; j++)
                {
                    PTArr[i, j] = SensMatrix[x - 1 - j, i];
                    Point3d MatLoc = new Point3d(y - 1 - j, i, 0);
                    //LocPt = MatLoc + new Point3d(0.5, 0.5, 0);
                    AllPts.Add(MatLoc);
                }
            }
            return PTArr;
        }

        private List<NurbsCurve> SubParTrace(Point3d SourcePt, double Radii)
        {
            List<NurbsCurve> CoreCurves = new List<NurbsCurve>();
            List<Tuple<Point3d, double>> BoundarySen = new List<Tuple<Point3d, double>>();
            bool Pathfound = false;
            int counter = 0;

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

                TraceMat[(int)L1.FromX, (int)L1.FromY] = 1;
                TraceMat[(int)L1.ToX, (int)L1.ToY] = 1;

                BoundarySen.AddRange(SetBoundaryPoints(L1, 1));

                SourcePt = C1.PointAtEnd;
                counter++;
            }
            BoundarySens.AddRange(BoundarySen);
            BoundarySens = BoundarySens.OrderByDescending(x => x.Item2).ToList();
            return CoreCurves;
        }

        private List<Tuple<Point3d, double>> SetBoundaryPoints(Line line, int thickness)
        {
            List<Tuple<Point3d, double>> BoundarySens = new List<Tuple<Point3d, double>>();
            List<Tuple<Point3d, double>> FinalBoundarySens = new List<Tuple<Point3d, double>>();


            var TurnAngle = Vector3d.VectorAngle(line.UnitTangent, unitX) * 180.0 / Math.PI;
            var n1 = line.Direction;
            var n2 = line.Direction;

            n1.Rotate(Math.PI / 2.0, Vector3d.ZAxis);
            n2.Rotate(-Math.PI / 2.0, Vector3d.ZAxis);
            for (int t = 1; t <= thickness + 1; t++)
            {
                List<int> x = new List<int>();
                List<int> y = new List<int>();
                x.Add((int)(line.ToX + t * n1.X)); y.Add((int)(line.ToY + t * n1.Y));
                x.Add((int)(line.ToX + t * n2.X)); y.Add((int)(line.ToY + t * n2.Y));
                x.Add((int)((line.ToX + line.FromX) / 2 + t * n1.X / 2)); y.Add((int)((line.ToY + line.FromY) / 2 + t * n1.Y / 2));
                x.Add((int)((line.ToX + line.FromX) / 2 + t * n2.X / 2)); y.Add((int)((line.ToY + line.FromY) / 2 + t * n2.Y / 2));
                if ((int)TurnAngle % 90 == 0)
                {
                    for (int i = 0; i < x.Count - 2; i++)
                        if (x[i] < TraceMat.GetLength(0) && y[i] < TraceMat.GetLength(1) && x[i] >= 0 && y[i] >= 0 && TraceMat[x[i], y[i]] != 1)
                        {
                            if (t == thickness + 1)
                                BoundarySens.Add(new Tuple<Point3d, double>(new Point3d(x[i], y[i], 0), SensMatrix[x[i], y[i]]));
                            else
                                TraceMat[x[i], y[i]] = 1;
                        }
                }
                else
                {
                    for (int i = 0; i < x.Count; i++)
                    {
                        if (x[i] < TraceMat.GetLength(0) && y[i] < TraceMat.GetLength(1) && x[i] >= 0 && y[i] >= 0 && TraceMat[x[i], y[i]] != 1)
                        {
                            if (t == thickness + 1)
                                BoundarySens.Add(new Tuple<Point3d, double>(new Point3d(x[i], y[i], 0), SensMatrix[x[i], y[i]]));
                            else
                                TraceMat[x[i], y[i]] = 1;
                        }
                    }
                }
            }
            return BoundarySens;
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