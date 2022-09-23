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


namespace TOtracing
{
    class PtFunctions
    {
        public static List<Tuple<Tuple<double, double>, Point3d>> MatToLoc2D(double[,] SensMatrix, out List<Point3d> AllPts)
        {
            List<Point3d> PTts = new List<Point3d>();

            int Xrange = SensMatrix.GetLength(0);
            int Yrenge = SensMatrix.GetLength(1);

            List<Tuple<Tuple<double, double>, Point3d>> ElementPoints = new List<Tuple<Tuple<double, double>, Point3d>>();
            for (int i = 0; i < Xrange; i++)
            {
                for (int j = 0; j < Yrenge; j++)
                {

                    Tuple<double, double> MatInd = new Tuple<double, double>(i, j);
                    Point3d MatLoc = new Point3d(j, (Xrange - 1) - i, 0);

                    Tuple<Tuple<double, double>, Point3d> Elementpoint = new Tuple<Tuple<double, double>, Point3d>(MatInd, MatLoc);

                    PTts.Add(MatLoc);
                    ElementPoints.Add(Elementpoint);


                }

            }

            AllPts = PTts;
            return ElementPoints;
        }

        public static List<Tuple<Tuple<double, double, double>, Point3d>> MatToLoc3D(double[,,] SensMatrix, out List<Point3d> AllPts)
        {
            List<Point3d> PTts = new List<Point3d>();
            double Xrange = SensMatrix.GetLength(0);
            double Yrange = SensMatrix.GetLength(1);
            double Zrange = SensMatrix.GetLength(2);

            List<Tuple<Tuple<double, double, double>, Point3d>> ElementPoints = new List<Tuple<Tuple<double, double, double>, Point3d>>();
            for (int i = 0; i < Xrange; i++)
            {
                for (int j = 0; j < Yrange; j++)
                {
                    for (int k = 0; k < Zrange; k++)
                    {
                        Tuple<double, double, double> MatInd = new Tuple<double, double, double>(i, j, k);
                        Point3d MatLoc = new Point3d(j, (Yrange - 1) - i, (Zrange - 1) - k);

                        Tuple<Tuple<double, double, double>, Point3d> Elementpoint = new Tuple<Tuple<double, double, double>, Point3d>(MatInd, MatLoc);
                        PTts.Add(MatLoc);
                        ElementPoints.Add(Elementpoint);
                    }
                }

            }
            AllPts = PTts;
            return ElementPoints;
        }

        public static List<Point3d> Exploreneighbours(Point3d CNPts, List<Point3d> AllPts, double Maxdis)
        {
            List<Point3d> NeighbourPts = new List<Point3d>();

            for (int i = 0; i < AllPts.Count; i++)
            {
                if (CNPts.Equals(AllPts[i]))
                    continue;
                if (CNPts.DistanceTo(AllPts[i]) <= Maxdis)
                    NeighbourPts.Add(AllPts[i]);
            }

            return NeighbourPts;
        }

        public static double ComputeCostForNeighbours(Point3d Pt0, Vector3d PreDir, Point3d Pt1, double[,] SensMatrix, List<Tuple<Tuple<double, double>, Point3d>> MatINFO, int[,] TraceMatrix)
        {
            double Cost = 0;
            double alpha=1;
            double betha;

            //Avoid backword move
            if (PreDir != Vector3d.Zero)
            {
                Vector3d TrVec = Pt1 - Pt0;
                double VecAng = Vector3d.VectorAngle(PreDir, TrVec) * (180 / Math.PI);
                if(VecAng>90)
                {
                    return Cost;
                }

                if (VecAng < 90 && VecAng > 40)
                {
                    alpha=0.8;
                }
                if (VecAng < 40 )
                {
                    alpha =1;
                }
            }

            Tuple<double, double> PtIndex = FindSenMatIndex(Pt1, MatINFO);

            if (TraceMatrix[(int)PtIndex.Item1, (int)PtIndex.Item2] == 1)
            {
                Cost = 0;
                return Cost;
            }    


            double Sen = SensMatrix[(int)PtIndex.Item1, (int)PtIndex.Item2];

            Cost = Sen * alpha;
            return Cost;
        }

        public static bool InterseCtionWithTraced(Point3d Pt0, Point3d Pt1, double[,] SensMatrix)
        {
            bool intersected = false;

            return intersected;
        }

        public static Tuple<double, double> FindSenMatIndex(Point3d Pt, List<Tuple<Tuple<double, double>, Point3d>> MatINFO)
        {
            Tuple<double, double> Index = new Tuple<double, double>(0, 0);
            for (int i = 0; i < MatINFO.Count; i++)
            {
                if (Pt.DistanceTo(MatINFO[i].Item2) < 0.00000001)
                {
                    Index = MatINFO[i].Item1;
                }
            }
            return Index;
        }

        public static List<NurbsCurve> ParTrace(double[,] SensMatrix, List<Tuple<Tuple<double, double>, Point3d>> MatINFO, Point3d SourcePt, double Radii, double MaxDis,out int[,] TraceMatrix)
        {

            TraceMatrix = new int[SensMatrix.GetLength(0), SensMatrix.GetLength(1)];

            List<NurbsCurve> CoreCurves = new List<NurbsCurve>();
            List<Point3d> AllPts = new List<Point3d>();
            bool Pathfound = false;
            int counter = 0;
            Point3d Ps = SourcePt;

            for (int i = 0; i < MatINFO.Count; i++)
            {
                AllPts.Add(MatINFO[i].Item2);
            }
            while (!Pathfound)
            {
                Vector3d PreDir = new Vector3d();
                if (CoreCurves.Count != 0)
                    PreDir = CoreCurves[CoreCurves.Count - 1].TangentAtStart;

                List<Tuple<Point3d, double>> NeighbourCost = new List<Tuple<Point3d, double>>();
                double SumCost = 0;
                var NeiPts = Exploreneighbours(Ps, AllPts, MaxDis);
                for (int i = 0; i < NeiPts.Count; i++)
                {
                    double Cost = ComputeCostForNeighbours(Ps, PreDir, NeiPts[i], SensMatrix, MatINFO, TraceMatrix);
                    SumCost += Cost;
                    Tuple<Point3d, double> CostPt = new Tuple<Point3d, double>(NeiPts[i], Cost);
                    NeighbourCost.Add(CostPt);
                }

                if (SumCost == 0)
                {
                    Pathfound = true;
                    break;
                }
                NeighbourCost.Sort((x, y) => y.Item2.CompareTo(x.Item2));


                Line L1 = new Line(Ps, NeighbourCost[NeighbourCost.Count - 1].Item1);
                NurbsCurve C1 = L1.ToNurbsCurve();
                C1.Domain = new Interval(0, 1);
                CoreCurves.Add(C1);

                Tuple<double, double> PtSourceIND1 = FindSenMatIndex(Ps, MatINFO);

                TraceMatrix[(int)PtSourceIND1.Item1, (int)PtSourceIND1.Item2] = 1;

                Tuple<double, double> PtSourceIND3 = FindSenMatIndex(C1.PointAtEnd, MatINFO);

                TraceMatrix[(int)PtSourceIND3.Item1, (int)PtSourceIND3.Item2] = 1;

                for (int i = 0; i < AllPts.Count; i++)
                {
                    C1.ClosestPoint(AllPts[i], out double t);
                    var Pt000 = C1.PointAt(t);
                    if (Pt000.DistanceTo(AllPts[i]) <= Radii)
                    {
                        Vector3d TestVec = AllPts[i]-Pt000;
                        double Angle = Vector3d.VectorAngle(TestVec, C1.TangentAtEnd) * (180 / Math.PI);
                        if (Angle-90<0.001 && Angle - 90 >= 0)
                        {
                            Tuple<double, double> PtSourceIND2 = FindSenMatIndex(AllPts[i], MatINFO);
                            TraceMatrix[(int)PtSourceIND2.Item1, (int)PtSourceIND2.Item2] = 1;
                        }
                    }
                }

                Ps = C1.PointAtEnd;
                counter++;

            }

            return CoreCurves;
        }

        public static void PrintMatrix(int[,] TrMatrix)
        {
            int rowLength = TrMatrix.GetLength(0);
            int colLength = TrMatrix.GetLength(1);

            for (int i = 0; i < rowLength; i++)
            {
                for (int j = 0; j < colLength; j++)
                {
                    Console.Write(string.Format("{0} ", TrMatrix[i, j]));
                }
                Console.Write(Environment.NewLine + Environment.NewLine);
            }
            Console.ReadLine();
        }

        public static void PrintMatrix2(double[,] TrMatrix)
        {
            int rowLength = TrMatrix.GetLength(0);
            int colLength = TrMatrix.GetLength(1);

            for (int i = 0; i < rowLength; i++)
            {
                for (int j = 0; j < colLength; j++)
                {
                    Console.Write(string.Format("{0} ", TrMatrix[i, j]));
                }
                Console.Write(Environment.NewLine + Environment.NewLine);
            }
            Console.ReadLine();
        }
    }
}
