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

namespace TOtracing
{
    class PtFunctions
    {

        public static double r_min = 1e-6;
        private int x, y;
        private List<Point3d> AllPts;
        private double Spt;

        public void MatFEM(double x, double y, int iter, double volfrac, out List<NurbsCurve> Curves)
        {
            Curves = new List<NurbsCurve>();
            int counter = 0;
            this.x = (int)x;
            this.y = (int)y;

            using (dynamic matlab = MATLABEngine.StartMATLAB())
            {
                double[,] resultMat;
                resultMat = matlab.IntTop88(x, y, 0.5, 3.0, 3.5, 1.0);

                while (counter < iter)
                {
                    double[,] sense = PtFunctions.ParticleTrace(resultMat, volfrac, out Curves);
                    resultMat = matlab.IterTop88(sense, x, y, 0.5, 3.0, 3.5, 1.0);
                    counter++;
                }


            }
        }

        private double[,] ParticleTrace(double[,] SensMat, double volfrac, out List<NurbsCurve> CurveList)
        {
            //string path = "SenDire";
            List<NurbsCurve> CorCVS = new List<NurbsCurve>();

            int maxV = SensMat.GetLength(0) * SensMat.GetLength(1);
            double V = maxV * volfrac;

            double[,] PTMat = MatToLoc2D(SensMat);
            Tuple< int , int> idx;

            Spt = PTMat.Min(out idx);
           
            int[,] TrMat = new int[SensMat.GetLength(0), SensMat.GetLength(1)];
            int counter = 0;
            CurveList = new List<NurbsCurve>();
            while (true)
            {
                if (counter != 0)
                {
                    //Spt = new Point3d(299, 1, 0);
                    Spt = ExplorTracPt(CurveList, SensMat, TrMat, PTMat);
                }
                List<NurbsCurve> CCuve = ParTrace(SensMat, PTMat, Spt, TrMat, 1, Math.Sqrt(2), AllPts);
                V = ComputeVol2d(TrMat);

                CorCVS.AddRange(CCuve);
                Interval inter = new Interval(0, 1);
                var crvs = Curve.JoinCurves(CCuve);
                foreach (var curve in crvs)
                {
                    var A = curve.ToNurbsCurve();
                    A.Domain = inter;
                    CurveList.Add(A);
                }


                if (V >= 20000)
                    break;
                counter++;
            }

            var TraceMatrix = DenseSet(TrMat);

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
            double[,] PTArr = new double[x, y];

            for (int i = 0; i < x; i++)
            {
                for (int j = 0; j < y; j++)
                {
                    double PtMat = SensMatrix[j, (x - 1) - i];
                    Point3d MatLoc = new Point3d(j, (x - 1) - i, 0);
                    //LocPt = MatLoc + new Point3d(0.5, 0.5, 0);
                    PTArr[i, j] = PtMat;
                    AllPts.Add(MatLoc);
                }
            }
            return PTArr;
        }

        //private Point3d[,] MatToLoc2D(double[,] SensMatrix)
        //{
        //    AllPts = new List<Point3d>();
        //    Point3d[,] PTArr = new Point3d[x, y];

        //    for (int i = 0; i < x; i++)
        //    {
        //        for (int j = 0; j < y; j++)
        //        {
        //            Point3d LocPt = new Point3d(j, (x - 1) - i, 0);
        //            //LocPt = LocPt + new Point3d(0.5, 0.5, 0);
        //            PTArr[i, j] = LocPt;
        //            AllPts.Add(LocPt);
        //        }
        //    }
        //    return PTArr;
        //}

        //public static List<Tuple<Tuple<double, double>, Point3d>> MatToLoc2D(double[,] SensMatrix, out List<Point3d> AllPts)
        //{
        //    List<Point3d> PTts = new List<Point3d>();

        //    int Xrange = SensMatrix.GetLength(0);
        //    int Yrenge = SensMatrix.GetLength(1);

        //    List<Tuple<Tuple<double, double>, Point3d>> ElementPoints = new List<Tuple<Tuple<double, double>, Point3d>>();
        //    for (int i = 0; i < Xrange; i++)
        //    {
        //        for (int j = 0; j < Yrenge; j++)
        //        {

        //            Tuple<double, double> MatInd = new Tuple<double, double>(i, j);
        //            Point3d MatLoc = new Point3d(j, (Xrange - 1) - i, 0);
        //            MatLoc = MatLoc + new Point3d(0.5, 0.5, 0);
        //            Tuple<Tuple<double, double>, Point3d> Elementpoint = new Tuple<Tuple<double, double>, Point3d>(MatInd, MatLoc);

        //            PTts.Add(MatLoc);
        //            ElementPoints.Add(Elementpoint);


        //        }

        //    }

        //    AllPts = PTts;
        //    return ElementPoints;
        //}

        private static List<Tuple<Tuple<double, double, double>, Point3d>> MatToLoc3D(double[,,] SensMatrix, out List<Point3d> AllPts)
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

        private static List<Point3d> Exploreneighbours0(Point3d CNPts, List<Point3d> AllPts, double Maxdis)
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

        private static List<Point3d> Exploreneighbours(Point3d CNPts, double MaxX, double MaxY, double MaxZ)
        {

            List<Point3d> NeighbourPts = new List<Point3d>();

            //Add procedural ring neighbour finding
            NeighbourPts.Add(new Point3d(CNPts.X + 1, CNPts.Y, CNPts.Z));
            NeighbourPts.Add(new Point3d(CNPts.X - 1, CNPts.Y, CNPts.Z));
            NeighbourPts.Add(new Point3d(CNPts.X, CNPts.Y + 1, CNPts.Z));
            NeighbourPts.Add(new Point3d(CNPts.X, CNPts.Y - 1, CNPts.Z));
            NeighbourPts.Add(new Point3d(CNPts.X + 1, CNPts.Y + 1, CNPts.Z));
            NeighbourPts.Add(new Point3d(CNPts.X + 1, CNPts.Y - 1, CNPts.Z));
            NeighbourPts.Add(new Point3d(CNPts.X - 1, CNPts.Y + 1, CNPts.Z));
            NeighbourPts.Add(new Point3d(CNPts.X - 1, CNPts.Y - 1, CNPts.Z));
            NeighbourPts.Add(new Point3d(CNPts.X + 1, CNPts.Y, CNPts.Z + 1));
            NeighbourPts.Add(new Point3d(CNPts.X - 1, CNPts.Y, CNPts.Z + 1));
            NeighbourPts.Add(new Point3d(CNPts.X, CNPts.Y + 1, CNPts.Z + 1));
            NeighbourPts.Add(new Point3d(CNPts.X, CNPts.Y - 1, CNPts.Z + 1));
            NeighbourPts.Add(new Point3d(CNPts.X + 1, CNPts.Y + 1, CNPts.Z - 1));
            NeighbourPts.Add(new Point3d(CNPts.X + 1, CNPts.Y - 1, CNPts.Z - 1));
            NeighbourPts.Add(new Point3d(CNPts.X - 1, CNPts.Y + 1, CNPts.Z - 1));
            NeighbourPts.Add(new Point3d(CNPts.X - 1, CNPts.Y - 1, CNPts.Z - 1));

            while (true)
            {
                bool found = true;

                for (int i = 0; i < NeighbourPts.Count; i++)
                {
                    if (NeighbourPts[i].X < 0 || NeighbourPts[i].X >= MaxY || NeighbourPts[i].Y < 0 || NeighbourPts[i].Y > MaxX || NeighbourPts[i].Z < 0 || NeighbourPts[i].Z > MaxZ)
                    {
                        NeighbourPts.RemoveAt(i);
                        found = false;
                        break;
                    }
                }

                if (found)
                    break;
            }

            return NeighbourPts;
        }

        private static double ComputeCostForNeighbours(Point3d Pt0, Vector3d PreDir, Point3d Pt1, double[,] SensMatrix, List<Tuple<Tuple<double, double>, Point3d>> MatINFO, int[,] TraceMatrix)
        {
            double Cost = 0;
            double alpha = 1;
            double betha;

            //Avoid backword move
            if (PreDir != Vector3d.Zero)
            {
                Vector3d TrVec = Pt1 - Pt0;
                double VecAng = Vector3d.VectorAngle(PreDir, TrVec) * (180 / Math.PI);
                if (VecAng >= 90)
                {
                    return Cost;
                }

                if (VecAng < 90 && VecAng > 40)
                {
                    alpha = 0.8;
                }
                if (VecAng < 40)
                {
                    alpha = 1;
                }
            }

            Tuple<double, double> PtIndex = FindSenMatIndex(Pt1, SensMatrix);

            if (TraceMatrix[(int)PtIndex.Item1, (int)PtIndex.Item2] == 1)
            {
                Cost = 0;
                return Cost;
            }


            double Sen = SensMatrix[(int)PtIndex.Item1, (int)PtIndex.Item2];

            Cost = Sen * alpha;
            return Cost;
        }

        private static bool InterseCtionWithTraced(Point3d Pt0, Point3d Pt1, double[,] SensMatrix)
        {
            bool intersected = false;

            return intersected;
        }

        private static Tuple<double, double> FindSenMatIndex(Point3d Pt, double[,] SensMatrix)
        {

            Point3d PP = new Point3d(Pt.X - 0.5, Pt.Y - 0.5, 0);
            double j = PP.X;
            double i = SensMatrix.GetLength(0) - 1 - PP.Y;

            Tuple<double, double> Index = new Tuple<double, double>(i, j);
            if (i < 0 || i >= SensMatrix.GetLength(0) || j < 0 || j >= SensMatrix.GetLength(1))
                return null;
            else
                return Index;
        }

        private static List<NurbsCurve> ParTrace(double[,] SensMatrix, List<Tuple<Tuple<double, double>, Point3d>> MatINFO, Point3d SourcePt, int[,] TraceMatrix, double Radii, double MaxDis, List<Point3d> AllPts)
        {

            List<NurbsCurve> CoreCurves = new List<NurbsCurve>();

            bool Pathfound = false;
            int counter = 0;
            Point3d Ps = SourcePt;

            while (!Pathfound)
            {
                Vector3d PreDir = new Vector3d();
                if (CoreCurves.Count != 0)
                    PreDir = CoreCurves[CoreCurves.Count - 1].TangentAtStart;

                List<Tuple<Point3d, double>> NeighbourCost = new List<Tuple<Point3d, double>>();
                double SumCost = 0;
                var NeiPts = Exploreneighbours(Ps, SensMatrix.GetLength(0), SensMatrix.GetLength(1), 0);
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

                Tuple<double, double> PtSourceIND1 = FindSenMatIndex(Ps, SensMatrix);

                TraceMatrix[(int)PtSourceIND1.Item1, (int)PtSourceIND1.Item2] = 1;

                Tuple<double, double> PtSourceIND3 = FindSenMatIndex(C1.PointAtEnd, SensMatrix);

                TraceMatrix[(int)PtSourceIND3.Item1, (int)PtSourceIND3.Item2] = 1;

                var LMatR = new List<Point3d>();
                int numi = 5, numj = 5;
                var Pst = C1.PointAtStart;
                for (int i = 0; i < numi; i++)
                {
                    for (int j = 0; j < numj; j++)
                    {

                        if (Pst.X - 2 + i < 0 || Pst.Y - 2 + j < 0 || Pst.X - 2 + i > SensMatrix.GetLength(1) || Pst.Y - 2 + j > SensMatrix.GetLength(0))
                            continue;
                        else
                            LMatR.Add(new Point3d(Pst.X - 2 + i, Pst.Y - 2 + j, Pst.Z));

                    }

                }

                for (int i = 0; i < LMatR.Count; i++)
                {
                    C1.ClosestPoint(LMatR[i], out double t);
                    var Pt000 = C1.PointAt(t);
                    if (Pt000.DistanceTo(LMatR[i]) <= Radii)
                    {
                        Vector3d TestVec = LMatR[i] - Pt000;
                        double Angle = Vector3d.VectorAngle(TestVec, C1.TangentAtEnd) * (180 / Math.PI);
                        if (Angle - 90 < 0.001 && Angle - 90 >= 0)
                        {
                            Tuple<double, double> PtSourceIND2 = FindSenMatIndex(LMatR[i], SensMatrix);
                            TraceMatrix[(int)PtSourceIND2.Item1, (int)PtSourceIND2.Item2] = 1;
                        }
                    }
                }

                Ps = C1.PointAtEnd;
                counter++;

            }


            return CoreCurves;
        }

        private static void RadiCom(NurbsCurve C, double radii)
        {
            Point3d P1 = C.PointAtStart;
            Point3d P2 = C.PointAtEnd;
            List<Point3d> RadPt = new List<Point3d>();
            List<Point3d> RadPtPER = new List<Point3d>();
            for (int i = 1; i < radii; i++)
            {
                RadPtPER.Add(new Point3d(P1.X + i, P1.Y, P1.Z));
                RadPtPER.Add(new Point3d(P1.X - i, P1.Y, P1.Z));
                RadPtPER.Add(new Point3d(P1.X, P1.Y + i, P1.Z));
                RadPtPER.Add(new Point3d(P1.X, P1.Y - i, P1.Z));
                RadPtPER.Add(new Point3d(P1.X + i, P1.Y + i, P1.Z));
                RadPtPER.Add(new Point3d(P1.X + i, P1.Y - i, P1.Z));
                RadPtPER.Add(new Point3d(P1.X - i, P1.Y + i, P1.Z));
                RadPtPER.Add(new Point3d(P1.X - i, P1.Y - i, P1.Z));

                RadPtPER.Add(new Point3d(P2.X + i, P1.Y, P1.Z));
                RadPtPER.Add(new Point3d(P2.X - i, P1.Y, P1.Z));
                RadPtPER.Add(new Point3d(P2.X, P1.Y + i, P1.Z));
                RadPtPER.Add(new Point3d(P2.X, P1.Y - i, P1.Z));
                RadPtPER.Add(new Point3d(P2.X + i, P1.Y + i, P1.Z));
                RadPtPER.Add(new Point3d(P2.X + i, P1.Y - i, P1.Z));
                RadPtPER.Add(new Point3d(P2.X - i, P1.Y + i, P1.Z));
                RadPtPER.Add(new Point3d(P2.X - i, P1.Y - i, P1.Z));

            }
        }
        private static void PrintMatrix(int[,] TrMatrix)
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

        private static void PrintMatrix2(double[,] TrMatrix)
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

        private static double ComputeVol2d(int[,] TraceMatrix)
        {
            double SolidVox = 0;
            for (int i = 0; i < TraceMatrix.GetLength(0); i++)
            {
                for (int j = 0; j < TraceMatrix.GetLength(1); j++)
                {
                    if (TraceMatrix[i, j] == 1)
                        SolidVox++;
                }


            }
            return SolidVox;

        }

        private static Point3d ExplorTracPt(List<NurbsCurve> CorCVS, double[,] SensMatrix, int[,] TraceMatrix, List<Tuple<Tuple<double, double>, Point3d>> MatINFO)
        {
            Random random = new Random();
            Point3d Pts = new Point3d();
            double Radii = 1;
            while (true)
            {
                int n1 = random.Next(0, CorCVS.Count - 1);
                double n2 = random.NextDouble();

                NurbsCurve CChosen = CorCVS[n1];
                Point3d PtChosen = new Point3d((Math.Ceiling(CChosen.PointAt(n2).X) + Math.Floor(CChosen.PointAt(n2).X)) / 2,
                    (Math.Ceiling(CChosen.PointAt(n2).Y) + Math.Floor(CChosen.PointAt(n2).Y)) / 2, (Math.Ceiling(CChosen.PointAt(n2).Z) + Math.Floor(CChosen.PointAt(n2).Z)) / 2);

                Line NLine = new Line(PtChosen, CorCVS[n1].TangentAt(n2) * Radii);
                Vector3d DirVec = new Vector3d(NLine.PointAt(1) - NLine.PointAt(0));
                var DirvecU = DirVec;
                var DirvecD = DirVec;
                // make ıt random for further updates
                // change this code for 3d topopt

                DirvecU.Rotate(Math.PI / 2, Vector3d.ZAxis);
                DirvecD.Rotate(-Math.PI / 2, Vector3d.ZAxis);


                var PtU = PtChosen + DirvecU;
                var PtD = PtChosen + DirvecD;
                PtU = PtU.PtRound();
                PtD = PtD.PtRound();

                //if (PtU.X < 0 || PtD.X < 0 || PtU.Y < 0 || PtD.Y < 0 || PtD.Z < 0 || PtU.Z < 0)
                //    continue;

                var IdU = FindSenMatIndex(PtU, SensMatrix);
                var IdD = FindSenMatIndex(PtD, SensMatrix);

                if (IdU != null)
                {
                    if (TraceMatrix[(Int32)IdU.Item1, (Int32)IdU.Item2] == 1)
                    {
                        var NeiPts = Exploreneighbours(PtU, SensMatrix.GetLength(0), SensMatrix.GetLength(1), 0);
                        double SumCost = 0;
                        for (int i = 0; i < NeiPts.Count; i++)
                        {
                            double Cost = ComputeCostForNeighbours(PtU, Vector3d.Zero, NeiPts[i], SensMatrix, MatINFO, TraceMatrix);
                            SumCost += Cost;
                        }
                        if (SumCost == 0)
                            goto here;
                        return PtU;
                    }

                }

            here:

                if (IdD != null)
                {
                    if (TraceMatrix[(Int32)IdD.Item1, (Int32)IdD.Item2] == 1)
                    {
                        var NeiPts = Exploreneighbours(PtD, SensMatrix.GetLength(0), SensMatrix.GetLength(1), 0);

                        double SumCost = 0;
                        for (int i = 0; i < NeiPts.Count; i++)
                        {
                            double Cost = ComputeCostForNeighbours(PtD, Vector3d.Zero, NeiPts[i], SensMatrix, MatINFO, TraceMatrix);
                            SumCost += Cost;
                        }
                        if (SumCost == 0)
                            continue;
                        return PtD;
                    }

                }
            }



            //var LMatR = new List<Point3d>();
            //int numi = 5, numj = 5;
            //var Pst = PtChosen;
            //for (int i = 0; i < numi; i++)
            //{
            //    for (int j = 0; j < numj; j++)
            //    {

            //        if (Pst.X - 2 + i < 0 || Pst.Y - 2 + j < 0 || Pst.X - 2 + i > SensMatrix.GetLength(1) || Pst.Y - 2 + j > SensMatrix.GetLength(0))
            //            continue;
            //        else
            //            LMatR.Add(new Point3d(Pst.X - 2 + i, Pst.Y - 2 + j, Pst.Z));

            //    }

            //}








        }
    }
}
