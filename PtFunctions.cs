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
using static Extensions.PointExtensions;

namespace TOtracing
{
    class PtFunctions
    {

        public static double r_min = 1e-6;
        public int m_border = 1;
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

            int maxV = (x + 2 * m_border) * (y + 2 * m_border);

            double Volume = maxV * volfrac;
            double V = 0;

            SensMatrix = SensMat.MatToLoc2D(x, y, m_border);

            MinIdx = SensMatrix.ArgMin();

            Spt = new Point3d(MinIdx.Item1, MinIdx.Item2, 0);

            TraceMat = new int[x + 2 * m_border, y + 2 * m_border];
            TraceMat.SetEdges(m_border);

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
            TopMat = TraceMatrix.GetSubMatrix(1, x, 1, y);
            resultMat = TraceMatrix.LocToMat2D(x, y);


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

                int[,] NeighbourMat = TraceMat.GetNeighbours(SourcePt);
                double[,] LocalSens = SensMatrix.GetNeighbours(SourcePt);

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

                BoundarySen.AddRange(SetBoundaryPoints(L1, 2));

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

            var pt1 = new Point3d(0,0,0);
            var pt2 = new Point3d(0,0,0);

            var pt3 = pt1 /3;

            //Get Line left and right sides
            var n1 = line.Direction;
            var n2 = line.Direction;
            n1.Rotate(Math.PI / 2.0, Vector3d.ZAxis);
            n2.Rotate(-Math.PI / 2.0, Vector3d.ZAxis);

            //Fill blocks with thickness value and set tracable boundary 
            for (int t = 1; t <= thickness + 1; t++)
            {
                List<Point2i> list = new List<Point2i>();

                //Find endpoint locations of path and get possible thickness directions and length
                //                                  X                                        Y
                //                         --------------------                ----------------------------
                list.Add(new Point2i((int)(line.ToX + t * n1.X), (int)(line.ToY + t * n1.Y)));
                list.Add(new Point2i((int)(line.ToX + t * n2.X), (int)(line.ToY + t * n2.Y)));
                list.Add(new Point2i((int)(line.MidPoint().X + t * n1.X / 2), (int)(line.MidPoint().Y + t * n1.Y / 2)));
                list.Add(new Point2i((int)(line.MidPoint().X + t * n2.X / 2), (int)(line.MidPoint().Y + t * n2.Y / 2)));

                #region old
                //List<int> xList = new List<int>();
                //List<int> yList = new List<int>();
                ////X                                                     //Y
                //xList.Add((int)(line.ToX + t * n1.X)); yList.Add((int)(line.ToY + t * n1.Y));
                //xList.Add((int)(line.ToX + t * n2.X)); yList.Add((int)(line.ToY + t * n2.Y));
                //xList.Add((int)(line.MidPoint().X + t * n1.X / 2)); yList.Add((int)(line.MidPoint().Y + t * n1.Y / 2));
                //xList.Add((int)(line.MidPoint().X + t * n2.X / 2)); yList.Add((int)(line.MidPoint().Y + t * n2.Y / 2));
                #endregion

                //Straight path operations
                if ((int)TurnAngle % 90 == 0)
                {
                    for (int i = 0; i < list.Count - 2; i++)
                        if (list[i].X < TraceMat.n() && list[i].Y < TraceMat.m() && list[i].X >= 0 && list[i].Y >= 0 && TraceMat[list[i].X, list[i].Y] != 1)
                        {
                            //Set start boundary
                            if (t == thickness + 1)
                                BoundarySens.Add(new Tuple<Point3d, double>(new Point3d(list[i].X, list[i].Y, 0), SensMatrix[list[i].X, list[i].Y]));
                            //Fill all blocks in the radius range
                            else
                                TraceMat[list[i].X, list[i].Y] = 1;
                        }
                }
                //Diagonal path operations
                else
                {
                    for (int i = 0; i < list.Count; i++)
                    {
                        if (list[i].X < TraceMat.n() && list[i].Y < TraceMat.m() && list[i].X >= 0 && list[i].Y >= 0 && TraceMat[list[i].X, list[i].Y] != 1)
                        {                      
                            //Set start boundary
                            if (t == thickness + 1)
                                BoundarySens.Add(new Tuple<Point3d, double>(new Point3d(list[i].X, list[i].Y, 0), SensMatrix[list[i].X, list[i].Y]));
                            //Fill all blocks in the radius range
                            else
                                TraceMat[list[i].X, list[i].Y] = 1;
                        }
                    }
                }

                #region old
                //if ((int)TurnAngle % 90 == 0)
                //{
                //    for (int i = 0; i < xList.Count - 2; i++)
                //        if (xList[i] < TraceMat.n() && yList[i] < TraceMat.m() && xList[i] >= 0 && yList[i] >= 0 && TraceMat[xList[i], yList[i]] != 1)
                //        {
                //            if (t == thickness + 1)
                //                BoundarySens.Add(new Tuple<Point3d, double>(new Point3d(xList[i], yList[i], 0), SensMatrix[xList[i], yList[i]]));
                //            else
                //                TraceMat[xList[i], yList[i]] = 1;
                //        }
                //}
                ////Diagonal path operations
                //else
                //{
                //    for (int i = 0; i < xList.Count; i++)
                //    {
                //        if (xList[i] < TraceMat.n() && yList[i] < TraceMat.m() && xList[i] >= 0 && yList[i] >= 0 && TraceMat[xList[i], yList[i]] != 1)
                //        {
                //            if (t == thickness + 1)
                //                BoundarySens.Add(new Tuple<Point3d, double>(new Point3d(xList[i], yList[i], 0), SensMatrix[xList[i], yList[i]]));
                //            else
                //                TraceMat[xList[i], yList[i]] = 1;
                //        }
                //    }
                //}
                #endregion

            }
            return BoundarySens;
        }

        private static double[,] ComputeCostForNeighbours(int[,] NeighbourMat, double[,] SensMatrix, Vector3d PreDir)
        {

            //Trace Cost
            double[,] TraceCost = new double[NeighbourMat.GetLength(0), NeighbourMat.GetLength(1)];

            for (int i = 0; i < NeighbourMat.n(); i++)
            {
                for (int j = 0; j < NeighbourMat.m(); j++)
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
