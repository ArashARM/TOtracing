using Grasshopper;
using Grasshopper.Kernel.Data;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.Serialization.Formatters.Binary;
using System.Text;
using System.Threading.Tasks;
using Accord.Math;
using Grasshopper.GUI;
using System.Collections;
using Rhino.Collections;
using static ICSharpCode.SharpZipLib.Zip.ExtendedUnixData;
using System.Globalization;
using System.Runtime.Serialization;
using System.Security.Permissions;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.Runtime.InteropServices;
using System.Runtime.Serialization;
using System.Security.Permissions;
using Rhino.Collections;
using Rhino.Runtime;


namespace Extensions
{
    public static class SurfaceExtensions
    {
        public static void Reparameterize(this NurbsSurface surface)
        {
            var dm = new Interval(0, 1);
            surface.SetDomain(0, dm);
            surface.SetDomain(1, dm);
        }
        public static void Reparameterize(this NurbsCurve curve)
        {
            var dm = new Interval(0, 1);
            curve.Domain = dm;
        }

        public static void Reparameterize(this Curve curve)
        {
            var dm = new Interval(0, 1);
            curve.Domain = dm;
        }

        //public static Curve Reparameterize(this Curve curve)
        //{
        //    var dm = new Interval(0, 1);
        //    curve.Domain = dm;
        //    return curve;
        //}

        public static void Reparameterize(this IEnumerable<NurbsCurve> curve)
        {
            var dm = new Interval(0, 1);
            foreach (var item in curve)
                item.Domain = dm;
        }
        public static void Reparameterize(this IEnumerable<Curve> curve)
        {
            var dm = new Interval(0, 1);
            foreach (var item in curve)
                item.Domain = dm;
        }

        public static NurbsSurface TransposeToNurbs(this NurbsSurface surface)
        {
            Surface newsurf = surface;
            surface = newsurf.Transpose().ToNurbsSurface();
            return surface;
        }

        public static NurbsSurface ReverseToNurbs(this NurbsSurface surface, int direction)
        {
            Surface newsurf = surface;

            if (direction == 0)
                surface = newsurf.Reverse(0).ToNurbsSurface();
            else
                surface = newsurf.Reverse(1).ToNurbsSurface();

            return surface;

        }

        public static void EqualizeKnots(ref NurbsSurface A, ref NurbsSurface B)
        {
            A.Reparameterize();
            B.Reparameterize();

            var distknotsA_U = A.KnotsU.DistinctKnots(B.KnotsU);
            var distknotsB_U = B.KnotsU.DistinctKnots(A.KnotsU);

            var distknotsA_V = A.KnotsV.DistinctKnots(B.KnotsV);
            var distknotsB_V = B.KnotsV.DistinctKnots(A.KnotsV);

            foreach (var item in distknotsA_U)
                B.KnotsU.InsertKnot(item);
            foreach (var item in distknotsB_U)
                A.KnotsU.InsertKnot(item);
            foreach (var item in distknotsA_V)
                B.KnotsV.InsertKnot(item);
            foreach (var item in distknotsB_V)
                A.KnotsV.InsertKnot(item);
        }

        public static void EqualizeKnots(ref Surface ASurf, ref Surface BSurf)
        {

            var A = ASurf.ToNurbsSurface();
            A.Reparameterize();
            var B = BSurf.ToNurbsSurface();
            B.Reparameterize();
            var distknotsA_U = A.KnotsU.DistinctKnots(B.KnotsU);
            var distknotsB_U = B.KnotsU.DistinctKnots(A.KnotsU);

            var distknotsA_V = A.KnotsV.DistinctKnots(B.KnotsV);
            var distknotsB_V = B.KnotsV.DistinctKnots(A.KnotsV);

            foreach (var item in distknotsA_U)
                B.KnotsU.InsertKnot(item);
            foreach (var item in distknotsB_U)
                A.KnotsU.InsertKnot(item);
            foreach (var item in distknotsA_V)
                B.KnotsV.InsertKnot(item);
            foreach (var item in distknotsB_V)
                A.KnotsV.InsertKnot(item);

            ASurf = A;
            BSurf = B;
        }

    }

    public static class LineExtensions
    {
        public static double ParalelPoint(this Rhino.Geometry.Line line)
        {
            if (line.FromX == line.ToX)
                return line.ToX;
            else if (line.FromY == line.ToY)
                return line.ToY;
            else
                return 0;
        }

        public static Rhino.Geometry.Line MovetoSide(this Rhino.Geometry.Line line, double t, bool left)
        {
            Transform move = new Transform();
            if (left)
                t = -t;
            Rhino.Geometry.Line movedline = line;
            if (line.FromX == line.ToX)
                move = Transform.Translation(t, 0, 0);
            else if (line.FromY == line.ToY)
                move = Transform.Translation(0, t, 0);

            movedline.Transform(move);
            return movedline;
        }

        public static Point3d MidPoint(this Rhino.Geometry.Line line)
        {
            return line.PointAt(0.5);
        }

        public static Point2d MidPointUV(this Rhino.Geometry.Line line)
        {
            return line.PointAtUV(0.5);
        }

        public static double Slope(this Rhino.Geometry.Line line)
        {
            double m;
            //if (line.ToX - line.FromX != 0)
            m = (line.ToY - line.FromY) / (line.ToX - line.FromX);

            m = Math.Atan(m);
            //else
            //{
            //    if (line.ToY - line.FromY < 0)
            //        m = double.MinValue;
            //    else
            //        m = double.MaxValue;
            //}
            return m;
        }

        public static Point2d PointAtUV(this Rhino.Geometry.Line line, double t)
        {
            Point2d pt;
            if (line.FromX == line.ToX)
                pt = new Point2d(line.From.X, line.PointAt(t).Y);
            else if (line.FromY == line.ToY)
                pt = new Point2d(line.PointAt(t).X, line.FromY);
            else
                pt = new Point2d(line.PointAt(t).X, line.PointAt(t).Y);

            return pt;
        }

        public static Rhino.Geometry.Line Extend(this Rhino.Geometry.Line line)
        {
            Line ln;
            ln = line;
            if (ln.FromX == ln.ToX)
            {
                if (ln.FromY > ln.ToY)
                {
                    ln.FromY = 1;
                    ln.ToY = 0;
                }
                else
                {
                    ln.FromY = 0;
                    ln.ToY = 1;
                }
            }
            else if (ln.FromY == ln.ToY)
            {
                if (ln.FromX > ln.ToX)
                {
                    ln.FromX = 1;
                    ln.ToX = 0;
                }
                else
                {
                    ln.FromX = 0;
                    ln.ToX = 1;
                }
            }


            return ln;
        }

    }

    public static class EnumExtensions
    {
        public static T SecondLast<T>(this IEnumerable<T> list)
        {
            var lst = list.ToList();
            var secondLast = lst[lst.Count() - 2];
            return secondLast;
        }

        //public static T SecondLast<T>(this List<T> list)
        //{
        //    var secondLast = list[list.Count() - 2];
        //    return secondLast;
        //}

        public static void AddToFront<T>(this List<T> list, T item)
        {
            // omits validation, etc.
            list.Insert(0, item);
        }

        public static void AddCurveEnds(this List<int> order, bool First)
        {
            if (order.Count < 1)
                return;
            if (First)
            {
                //first curve
                if (order.First<int>() % 2 == 0 && !order.Contains(order.First<int>() + 1))
                    order.AddToFront(order.First<int>() + 1);
                else if (order.First<int>() % 2 == 1 && !order.Contains(order.First<int>() - 1))
                    order.AddToFront(order.First<int>() - 1);
            }
            else
            {
                order.RemoveAt(0);
            }

            //LastCurve
            if (order.Last<int>() % 2 == 0 && !order.Contains(order.Last<int>() + 1))
                order.Add(order.Last<int>() + 1);
            else if (order.Last<int>() % 2 == 1 && !order.Contains(order.Last<int>() - 1))
                order.Add(order.Last<int>() - 1);

        }

    }

    public static class FileExtensions
    {
        /// <summary>
        /// Line Count for loops
        /// </summary>
        /// <param name="path"></param>
        /// <returns> it returns count -1 </returns>
        public static int FileLineCount(this string path)
        {
            if (File.Exists(path))
                return File.ReadLines(path).Count() - 1;
            else
                return 0;
        }
        public static string FileLastLine(this string path)
        {
            if (File.Exists(path))
                return File.ReadLines(path).Last();
            else
                return null;
        }

        /// <summary>
        /// Gets last line of path file
        /// </summary>
        /// <param name="path"></param>
        /// <param name="spr"></param>
        /// <returns> splited last line</returns>
        public static string[] FileLastLineSep(this string path, string spr = ";")
        {
            if (File.Exists(path))
                return File.ReadLines(path).Last().Split(spr.ToCharArray());
            else
                return null;
        }

        public static bool ChangeLine(this string filepath, string newline, int line_to_edit)
        {
            if (line_to_edit < 0)
            {

                string[] allLine = File.ReadAllLines(filepath);
                allLine[line_to_edit - 1] = newline;
                File.WriteAllLines(filepath, allLine);
                return true;
            }
            else
                return false;
        }

        public static void ChangeLastLine(this string filepath, string newline)
        {
            var line_to_edit = File.ReadLines(filepath).Count();
            string[] allLine = File.ReadAllLines(filepath);
            allLine[line_to_edit - 1] = newline;
            File.WriteAllLines(filepath, allLine);
        }

        //public static void RewriteData(this string filepath, IEnumerable<string> newlines)
        //{

        //    File.WriteAllLines(filepath, newlines);
        //}
    }

    public static class KnotExtensions
    {
        public static List<double> ToList(this Rhino.Geometry.Collections.NurbsSurfaceKnotList knots)
        {
            var knotList = new List<double>();
            foreach (var item in knots)
            {
                knotList.Add(item);
            }
            return knotList;
        }

        public static List<double> DistinctKnots(this Rhino.Geometry.Collections.NurbsSurfaceKnotList knots, Rhino.Geometry.Collections.NurbsSurfaceKnotList knots2)
        {

            var list1 = knots.ToList();
            var list2 = knots2.ToList();
            return list1.Except(list2).ToList();
        }


    }

    public static class PointExtensions
    {
        public struct Point3i
        {
            public Point3i(int x, int y, int z)
            {
                this.x = x;
                this.y = y;
                this.z = z;
            }
            public int x;
            public int y;
            public int z;
        }

        public struct Point2i
        {
            public Point2i(int x, int y)
            {
                this.X = x;
                this.Y = y;
            }
            public int X { get; set; }
            public int Y { get; set; }

        }

        public static Point3d PtRound(this Point3d pt3D)
        {
            pt3D = new Point3d((Math.Ceiling(pt3D.X) + Math.Floor(pt3D.X)) / 2, (Math.Ceiling(pt3D.Y) + Math.Floor(pt3D.Y)) / 2, (Math.Ceiling(pt3D.Z) + Math.Floor(pt3D.Z)) / 2);
            return pt3D;
        }

        public static Point2d ToPoint2d(this Point3d point)
        {
            Point2d newpoint = new Point2d(point.X, point.Y);
            return newpoint;
        }

        public static Point3d ToPoint3d(this Point2d point)
        {
            Point3d newpoint = new Point3d(point.X, point.Y, 0);
            return newpoint;
        }

        public static List<Point2d> ToPoint2d(this IEnumerable<Point3d> point)
        {
            List<Point2d> newpoint = new List<Point2d>();
            foreach (var item in point)
            {
                newpoint.Add(new Point2d(item.X, item.Y));
            }
            return newpoint;
        }

        public static List<Point3d> ToPoint3d(this IEnumerable<Point2d> point)
        {
            List<Point3d> newpoint = new List<Point3d>();
            foreach (var item in point)
            {
                newpoint.Add(new Point3d(item.X, item.Y, 0));
            }
            return newpoint;
        }

        public static Point3d MoveX(this Point3d point, double translation_x)
        {
            var pt = new Point3d(point.X + translation_x, point.Y, point.Z);
            return pt;
        }

        public static Point3d MoveY(this Point3d point, double translation_y)
        {
            var pt = new Point3d(point.X, point.Y + translation_y, point.Z);
            return pt;
        }

        public static Point3d MoveZ(this Point3d point, double translation_z)
        {
            var pt = new Point3d(point.X, point.Y, point.Z + translation_z);
            return pt;
        }

    }

    public static class ListExtensions
    {
        public static void Mirror<T>(this T[,] array, int i, int j)
        {
            array[j, i] = array[i, j];
        }
        public static void Mirror<T>(this T[,] array)
        {
            for (int i = 0; i < array.Rows(); i++)
            {
                for (int j = i; j < array.Columns(); j++)
                {
                    array[j, i] = array[i, j];
                }
            }
        }
        public static T DeepClone<T>(this T obj)
        {
            using (var ms = new MemoryStream())
            {
                var formatter = new BinaryFormatter();
                formatter.Serialize(ms, obj);
                ms.Position = 0;
                return (T)formatter.Deserialize(ms);
            }
        }


        public static DataTree<T> ToTree<T>(this List<List<T>> list)
        {
            var tree = new DataTree<T>();

            for (int i = 0; i < list.Count; i++)
            {
                var pth = new GH_Path(i);
                foreach (var result in list[i])
                {
                    tree.Add(result, pth);
                }
            }
            return tree;
        }

        public static void RemoveAt(this List<int> lst, IEnumerable<int> extractIdxs)
        {
            //var ordered = extractIdxs.OrderByDescending(i => i);
            foreach (var item in extractIdxs)
            {
                if (item % 2 == 0 && !extractIdxs.Contains(item + 1))
                    continue;
                else if (item % 2 == 1 && !extractIdxs.Contains(item - 1))
                    continue;
                else
                    lst.Remove(item);
            }
        }
    }

    public static class DoubleExtensions
    {
        public static string ToS(this double num, int dec = 3)
        {
            string deci = "0";
            for (int i = 0; i < dec; i++)
            {
                if (i == 0)
                    deci += ".";
                deci += "0";
            }

            return num.ToString(deci);
        }
    }
    public static class IntExtensions
    {
        public static int n(this int[,] matrix)
        {
            return matrix.GetLength(0);
        }
        public static int m(this int[,] matrix)
        {
            return matrix.GetLength(1);
        }

        public static string ToS(this int num, int dec = 3)
        {
            string deci = "0";
            for (int i = 0; i < dec; i++)
            {
                if (i == 0)
                    deci += ".";
                deci += "0";
            }

            return num.ToString(deci);
        }
    }

    public static class ArrayExtensions
    {
        /// <summary>
        /// Sets edge values of the matrix to the given value. 
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param matrix to be changed="matrix"></param>
        /// <param value to be setted="value"></param>
        /// <param edge thickness of the matrix to be changed ="thickness"></param>
        /// <returns> void</returns>
        public static void SetEdges<T>(this T[,] matrix, T value, int thickness = 1)
        {
            if (2 * thickness > matrix.GetLength(0) || 2 * thickness > matrix.GetLength(1))
                return;
            else
            {
                var rowVector = Vector.Create(matrix.GetLength(1), value);
                var colVector = Vector.Create(matrix.GetLength(0), value);

                for (int i = 0; i < thickness; i++)
                {
                    matrix = matrix.SetRow(i, rowVector);
                    matrix = matrix.SetRow(matrix.GetLength(0) - i - 1, rowVector);
                    matrix = matrix.SetColumn(i, colVector);
                    matrix = matrix.SetColumn(matrix.GetLength(1) - i - 1, colVector);
                }
            }

            return;
        }
        public static T[,] GetSubMatrix<T>(this T[,] matrix, int start_row, int end_row, int start_column, int end_column)
        {
            return matrix.Get(start_row, end_row + 1, start_column, end_column + 1);
        }

        /// <summary>
        /// Gets neighbours of a given matrix coor
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="Matrix"></param>
        /// <param name="idx">index of the point</param>
        /// <param name="r">r-ring neighbours of a </param>
        /// <returns>returns neighbours</returns>
        public static T[,] GetNeighbours<T>(this T[,] Matrix, Point3d idx, int r = 1)
        {
            int i1 = (int)idx.X - r;
            int i2 = (int)idx.X + r;
            int j1 = (int)idx.Y - r;
            int j2 = (int)idx.Y + r;
            var str = "";
            if (i1 < 0)
            { i1 = 0; str = "error"; }
            else if (j1 < 0)
            { j1 = 0; str = "error"; }
            else if (i2 >= Matrix.GetLength(0))
            { i2 = Matrix.GetLength(0) - 1; str = "error"; }
            else if (j2 >= Matrix.GetLength(1))
            { j2 = Matrix.GetLength(1) - 1; str = "error"; }

            return Matrix.GetSubMatrix(i1, i2, j1, j2);
        }

        public static T[,] MatToLoc2D<T>(this T[,] SensMatrix, int x, int y, int Border)
        {
            T[,] PTArr = new T[x + Border * 2, y + Border * 2];

            for (int i = 0; i < x; i++)
            {
                for (int j = 0; j < y; j++)
                {
                    PTArr[i + Border, j + Border] = SensMatrix[y - 1 - j, i];
                }
            }
            return PTArr;
        }

        public static T[,] LocToMat2D<T>(this T[,] SensMatrix, int x, int y)
        {
            T[,] PTArr = new T[y, x];

            for (int i = 0; i < y; i++)
            {
                for (int j = 0; j < x; j++)
                {
                    PTArr[i, j] = SensMatrix[x - j - 1, i];

                }
            }
            return PTArr;
        }


        //private double[,] MatToLoc2D(double[,] SensMatrix)
        //{
        //    AllPts = new List<Point3d>();

        //    double[,] PTArr = new double[x + 2, y + 2];

        //    for (int i = 0; i < x; i++)
        //    {
        //        for (int j = 0; j < y; j++)
        //        {
        //            PTArr[i + 1, j + 1] = SensMatrix[y - 1 - j, i];
        //            Point3d MatLoc = new Point3d(y - 1 - j, i, 0);
        //            //LocPt = MatLoc + new Point3d(0.5, 0.5, 0);
        //            AllPts.Add(MatLoc);
        //        }
        //    }
        //    return PTArr;
        //}
        //private double[,] LocToMat2D(double[,] SensMatrix)
        //{
        //    AllPts = new List<Point3d>();

        //    var Sen = SensMatrix.GetSubMatrix(1, x, 1, y);

        //    double[,] PTArr = new double[y, x];

        //    for (int i = 0; i < y; i++)
        //    {
        //        for (int j = 0; j < x; j++)
        //        {
        //            PTArr[i, j] = SensMatrix[x - j - 1, i];
        //            Point3d MatLoc = new Point3d(y - j - 1, i, 0);
        //            //LocPt = MatLoc + new Point3d(0.5, 0.5, 0);
        //            AllPts.Add(MatLoc);
        //        }
        //    }
        //    return PTArr;
        //}

    }

}


//namespace Rhino.Geometry
//{

//    public struct Point3i : ISerializable, IEquatable<Point3i>, IComparable<Point3i>, IComparable, IEpsilonComparable<Point3i>, ICloneable, IValidable, IFormattable
//    {
//        internal int m_x;

//        internal int m_y;

//        internal int m_z;

//        //
//        // Summary:
//        //     Gets the value of a point at location 0,0,0.
//        public static Point3i Origin => new Point3i(0, 0, 0);

//        //
//        // Summary:
//        //     Gets the value of a point at location RhinoMath.UnsetValue,RhinoMath.UnsetValue,RhinoMath.UnsetValue.
//        //public static Point3i Unset => new Point3i(-1.23432101234321E+308, -1.23432101234321E+308, -1.23432101234321E+308);

//        //
//        // Summary:
//        //     Gets or sets the X (first) coordinate of this point.
//        public int X
//        {
//            get
//            {
//                return m_x;
//            }
//            set
//            {
//                m_x = value;
//            }
//        }

//        //
//        // Summary:
//        //     Gets or sets the Y (second) coordinate of this point.
//        public int Y
//        {
//            get
//            {
//                return m_y;
//            }
//            set
//            {
//                m_y = value;
//            }
//        }

//        //
//        // Summary:
//        //     Gets or sets the Z (third) coordinate of this point.
//        public int Z
//        {
//            get
//            {
//                return m_z;
//            }
//            set
//            {
//                m_z = value;
//            }
//        }

//        //
//        // Summary:
//        //     Gets or sets an indexed coordinate of this point.
//        //
//        // Parameters:
//        //   index:
//        //     The coordinate index. Valid values are:
//        //     0 = X coordinate
//        //     1 = Y coordinate
//        //     2 = Z coordinate
//        //     .
//        public int this[int index]
//        {
//            get
//            {
//                if (index == 0)
//                {
//                    return m_x;
//                }

//                if (1 == index)
//                {
//                    return m_y;
//                }

//                if (2 == index)
//                {
//                    return m_z;
//                }

//                throw new IndexOutOfRangeException();
//            }
//            set
//            {
//                if (index == 0)
//                {
//                    m_x = value;
//                    return;
//                }

//                if (1 == index)
//                {
//                    m_y = value;
//                    return;
//                }

//                if (2 == index)
//                {
//                    m_z = value;
//                    return;
//                }

//                throw new IndexOutOfRangeException();
//            }
//        }

//        //
//        // Summary:
//        //     Each coordinate of the point must pass the Rhino.RhinoMath.IsValidDouble(System.Double)
//        //     test.
//        public bool IsValid
//        {
//            get
//            {
//                if (RhinoMath.IsValidDouble(m_x) && RhinoMath.IsValidDouble(m_y))
//                {
//                    return RhinoMath.IsValidDouble(m_z);
//                }

//                return false;
//            }
//        }

//        //
//        // Summary:
//        //     Initializes a new point by defining the X, Y and Z coordinates.
//        //
//        // Parameters:
//        //   x:
//        //     The value of the X (first) coordinate.
//        //
//        //   y:
//        //     The value of the Y (second) coordinate.
//        //
//        //   z:
//        //     The value of the Z (third) coordinate.
//        public Point3i(int x, int y, int z)
//        {
//            m_x = x;
//            m_y = y;
//            m_z = z;
//        }

//        //
//        // Summary:
//        //     Converts a vector in a point, needing casting.
//        //
//        // Parameters:
//        //   vector:
//        //     A vector.
//        //
//        // Returns:
//        //     The resulting point.
//        public static explicit operator Point3d(Vector3d vector)
//        {
//            return new Point3d(vector);
//        }

//        //
//        // Summary:
//        //     Converts a single-precision point in a double-precision point, without needing
//        //     casting.
//        //
//        // Parameters:
//        //   point:
//        //     A point.
//        //
//        // Returns:
//        //     The resulting point.
//        public static implicit operator Point3d(Point3f point)
//        {
//            return new Point3d(point);
//        }

//        //
//        // Summary:
//        //     Converts a single-precision point in a double-precision point.
//        //
//        // Parameters:
//        //   point:
//        //     A point.
//        //
//        // Returns:
//        //     The resulting point.
//        public static Point3d FromPoint3f(Point3f point)
//        {
//            return new Point3d(point);
//        }

//        //
//        // Summary:
//        //     Determines whether the first specified point comes before (has inferior sorting
//        //     value than) the second point.
//        //     Coordinates evaluation priority is first X, then Y, then Z.
//        //
//        // Parameters:
//        //   a:
//        //     The first point.
//        //
//        //   b:
//        //     The second point.
//        //
//        // Returns:
//        //     true if a.X is smaller than b.X, or a.X == b.X and a.Y is smaller than b.Y, or
//        //     a.X == b.X and a.Y == b.Y and a.Z is smaller than b.Z; otherwise, false.
//        public static bool operator <(Point3d a, Point3d b)
//        {
//            if (a.X < b.X)
//            {
//                return true;
//            }

//            if (a.X == b.X)
//            {
//                if (a.Y < b.Y)
//                {
//                    return true;
//                }

//                if (a.Y == b.Y && a.Z < b.Z)
//                {
//                    return true;
//                }
//            }

//            return false;
//        }

//        //
//        // Summary:
//        //     Determines whether the first specified point comes before (has inferior sorting
//        //     value than) the second point, or it is equal to it.
//        //     Coordinates evaluation priority is first X, then Y, then Z.
//        //
//        // Parameters:
//        //   a:
//        //     The first point.
//        //
//        //   b:
//        //     The second point.
//        //
//        // Returns:
//        //     true if a.X is smaller than b.X, or a.X == b.X and a.Y is smaller than b.Y, or
//        //     a.X == b.X and a.Y == b.Y and a.Z <= b.Z; otherwise, false.
//        public static bool operator <=(Point3d a, Point3d b)
//        {
//            return a.CompareTo(b) <= 0;
//        }

//        //
//        // Summary:
//        //     Determines whether the first specified point comes after (has superior sorting
//        //     value than) the second point.
//        //     Coordinates evaluation priority is first X, then Y, then Z.
//        //
//        // Parameters:
//        //   a:
//        //     The first point.
//        //
//        //   b:
//        //     The second point.
//        //
//        // Returns:
//        //     true if a.X is larger than b.X, or a.X == b.X and a.Y is larger than b.Y, or
//        //     a.X == b.X and a.Y == b.Y and a.Z is larger than b.Z; otherwise, false.
//        public static bool operator >(Point3d a, Point3d b)
//        {
//            if (a.X > b.X)
//            {
//                return true;
//            }

//            if (a.X == b.X)
//            {
//                if (a.Y > b.Y)
//                {
//                    return true;
//                }

//                if (a.Y == b.Y && a.Z > b.Z)
//                {
//                    return true;
//                }
//            }

//            return false;
//        }

//        //
//        // Summary:
//        //     Determines whether the first specified point comes after (has superior sorting
//        //     value than) the second point, or it is equal to it.
//        //     Coordinates evaluation priority is first X, then Y, then Z.
//        //
//        // Parameters:
//        //   a:
//        //     The first point.
//        //
//        //   b:
//        //     The second point.
//        //
//        // Returns:
//        //     true if a.X is larger than b.X, or a.X == b.X and a.Y is larger than b.Y, or
//        //     a.X == b.X and a.Y == b.Y and a.Z >= b.Z; otherwise, false.
//        public static bool operator >=(Point3d a, Point3d b)
//        {
//            return a.CompareTo(b) >= 0;
//        }

//        //
//        // Summary:
//        //     Determines whether the specified System.Object is a Rhino.Geometry.Point3d and
//        //     has the same values as the present point.
//        //
//        // Parameters:
//        //   obj:
//        //     The specified object.
//        //
//        // Returns:
//        //     true if obj is a Point3d and has the same coordinates as this; otherwise false.
//        [ConstOperation]
//        public override bool Equals(object obj)
//        {
//            if (obj is Point3d)
//            {
//                return this == (Point3d)obj;
//            }

//            return false;
//        }

//        //
//        // Summary:
//        //     Check that all values in other are within epsilon of the values in this
//        //
//        // Parameters:
//        //   other:
//        //
//        //   epsilon:
//        [ConstOperation]
//        public bool EpsilonEquals(Point3d other, double epsilon)
//        {
//            if (RhinoMath.EpsilonEquals(m_x, other.m_x, epsilon) && RhinoMath.EpsilonEquals(m_y, other.m_y, epsilon))
//            {
//                return RhinoMath.EpsilonEquals(m_z, other.m_z, epsilon);
//            }

//            return false;
//        }

//        //
//        // Summary:
//        //     Compares this Rhino.Geometry.Point3d with another Rhino.Geometry.Point3d.
//        //     Component evaluation priority is first X, then Y, then Z.
//        //
//        // Parameters:
//        //   other:
//        //     The other Rhino.Geometry.Point3d to use in comparison.
//        //
//        // Returns:
//        //     0: if this is identical to other
//        //     -1: if this.X < other.X
//        //     -1: if this.X == other.X and this.Y < other.Y
//        //     -1: if this.X == other.X and this.Y == other.Y and this.Z < other.Z
//        //     +1: otherwise.
//        [ConstOperation]
//        public int CompareTo(Point3d other)
//        {
//            if (m_x < other.m_x)
//            {
//                return -1;
//            }

//            if (m_x > other.m_x)
//            {
//                return 1;
//            }

//            if (m_y < other.m_y)
//            {
//                return -1;
//            }

//            if (m_y > other.m_y)
//            {
//                return 1;
//            }

//            if (m_z < other.m_z)
//            {
//                return -1;
//            }

//            if (m_z > other.m_z)
//            {
//                return 1;
//            }

//            return 0;
//        }

//        [ConstOperation]
//        int IComparable.CompareTo(object obj)
//        {
//            if (obj is Point3d)
//            {
//                return CompareTo((Point3d)obj);
//            }

//            throw new ArgumentException("Input must be of type Point3d", "obj");
//        }

//        //
//        // Summary:
//        //     Determines whether the specified Rhino.Geometry.Point3d has the same values as
//        //     the present point.
//        //
//        // Parameters:
//        //   point:
//        //     The specified point.
//        //
//        // Returns:
//        //     true if point has the same coordinates as this; otherwise false.
//        [ConstOperation]
//        public bool Equals(Point3d point)
//        {
//            return this == point;
//        }

//        //
//        // Summary:
//        //     Computes a hash code for the present point.
//        //
//        // Returns:
//        //     A non-unique integer that represents this point.
//        [ConstOperation]
//        public override int GetHashCode()
//        {
//            return m_x.GetHashCode() ^ m_y.GetHashCode() ^ m_z.GetHashCode();
//        }

//        //
//        // Summary:
//        //     Interpolate between two points.
//        //
//        // Parameters:
//        //   pA:
//        //     First point.
//        //
//        //   pB:
//        //     Second point.
//        //
//        //   t:
//        //     Interpolation parameter. If t=0 then this point is set to pA. If t=1 then this
//        //     point is set to pB. Values of t in between 0.0 and 1.0 result in points between
//        //     pA and pB.
//        public void Interpolate(Point3d pA, Point3d pB, double t)
//        {
//            m_x = pA.m_x + t * (pB.m_x - pA.m_x);
//            m_y = pA.m_y + t * (pB.m_y - pA.m_y);
//            m_z = pA.m_z + t * (pB.m_z - pA.m_z);
//        }

//        //
//        // Summary:
//        //     Constructs the string representation for the current point.
//        //
//        // Returns:
//        //     The point representation in the form X,Y,Z.
//        [ConstOperation]
//        public override string ToString()
//        {
//            CultureInfo ýnvariantCulture = CultureInfo.InvariantCulture;
//            return $"{m_x.ToString(ýnvariantCulture)},{m_y.ToString(ýnvariantCulture)},{m_z.ToString(ýnvariantCulture)}";
//        }

//        [ConstOperation]
//        public string ToString(string format, IFormatProvider formatProvider)
//        {
//            return FormatCoordinates(format, formatProvider, m_x, m_y, m_z);
//        }

//        //
//        // Summary:
//        //     Utility method for formatting coordinate groups.
//        internal static string FormatCoordinates(string format, IFormatProvider provider, params double[] coordinates)
//        {
//            string[] array = new string[coordinates.Length];
//            for (int i = 0; i < coordinates.Length; i++)
//            {
//                array[i] = coordinates[i].ToString(format, provider);
//            }

//            bool flag = true;
//            CultureInfo cultureInfo = provider as CultureInfo;
//            if (cultureInfo != null)
//            {
//                if (IsCommaLikeText(cultureInfo.NumberFormat.NumberDecimalSeparator))
//                {
//                    flag = false;
//                }
//            }
//            else if (IsCommaLikeText((-12345.6789).ToString(format, provider)))
//            {
//                flag = false;
//            }

//            if (flag)
//            {
//                return string.Join(",", array);
//            }

//            return string.Join(";", array);
//        }

//        //
//        // Summary:
//        //     Test whether a string contains any char which looks like a comma.
//        private static bool IsCommaLikeText(string text)
//        {
//            for (int i = 0; i < text.Length; i++)
//            {
//                if (IsCommaLikeChar(text[i]))
//                {
//                    return true;
//                }
//            }

//            return false;
//        }

//        //
//        // Summary:
//        //     Test whether a char looks like a comma.
//        private static bool IsCommaLikeChar(char character)
//        {
//            if (character == ',')
//            {
//                return true;
//            }

//            if (character < '\u0600')
//            {
//                return false;
//            }

//            return character switch
//            {
//                '?' => true,
//                '?' => true,
//                '?' => true,
//                '?' => true,
//                '?' => true,
//                _ => false,
//            };
//        }

//        //
//        // Summary:
//        //     Computes the distance between two points.
//        //
//        // Parameters:
//        //   other:
//        //     Other point for distance measurement.
//        //
//        // Returns:
//        //     The length of the line between this and the other point; or 0 if any of the points
//        //     is not valid.
//        [ConstOperation]
//        public double DistanceTo(Point3d other)
//        {
//            if (IsValid && other.IsValid)
//            {
//                double dx = other.m_x - m_x;
//                double dy = other.m_y - m_y;
//                double dz = other.m_z - m_z;
//                return Vector3d.GetLengthHelper(dx, dy, dz);
//            }

//            return 0.0;
//        }

//        //
//        // Summary:
//        //     Computes the square of the distance between two points.
//        //     This method is usually largely faster than DistanceTo().
//        //
//        // Parameters:
//        //   other:
//        //     Other point for squared distance measurement.
//        //
//        // Returns:
//        //     The squared length of the line between this and the other point; or 0 if any
//        //     of the points is not valid.
//        [ConstOperation]
//        public double DistanceToSquared(Point3d other)
//        {
//            if (IsValid && other.IsValid)
//            {
//                return (this - other).SquareLength;
//            }

//            return 0.0;
//        }

//        //
//        // Summary:
//        //     Transforms the present point in place. The transformation matrix acts on the
//        //     left of the point. i.e.,
//        //     result = transformation*point
//        //
//        // Parameters:
//        //   xform:
//        //     Transformation to apply.
//        public void Transform(Transform xform)
//        {
//            double num = xform.m_30 * m_x + xform.m_31 * m_y + xform.m_32 * m_z + xform.m_33;
//            if (num != 0.0)
//            {
//                num = 1.0 / num;
//            }

//            double x = num * (xform.m_00 * m_x + xform.m_01 * m_y + xform.m_02 * m_z + xform.m_03);
//            double y = num * (xform.m_10 * m_x + xform.m_11 * m_y + xform.m_12 * m_z + xform.m_13);
//            double z = num * (xform.m_20 * m_x + xform.m_21 * m_y + xform.m_22 * m_z + xform.m_23);
//            m_x = x;
//            m_y = y;
//            m_z = z;
//        }

//        //
//        // Summary:
//        //     Removes duplicates in the supplied set of points.
//        //
//        // Parameters:
//        //   points:
//        //     A list, an array or any enumerable of Rhino.Geometry.Point3d.
//        //
//        //   tolerance:
//        //     The minimum distance between points.
//        //     Points that fall within this tolerance will be discarded.
//        //     .
//        //
//        // Returns:
//        //     An array of points without duplicates; or null on error.
//        public static Point3d[] CullDuplicates(IEnumerable<Point3d> points, double tolerance)
//        {
//            if (points == null)
//            {
//                return null;
//            }

//            Point3dList point3dList = new Point3dList(points);
//            int count = point3dList.Count;
//            if (count == 0)
//            {
//                return null;
//            }

//            bool[] array = new bool[count];
//            Point3dList point3dList2 = new Point3dList(count);
//            for (int i = 0; i < count; i++)
//            {
//                if (array[i])
//                {
//                    continue;
//                }

//                point3dList2.Add(point3dList[i]);
//                for (int j = i + 1; j < count; j++)
//                {
//                    if (point3dList[i].DistanceTo(point3dList[j]) <= tolerance)
//                    {
//                        array[j] = true;
//                    }
//                }
//            }

//            return point3dList2.ToArray();
//        }

//        object ICloneable.Clone()
//        {
//            return this;
//        }

//        //
//        // Summary:
//        //     Determines whether a set of points is coplanar within a given tolerance.
//        //
//        // Parameters:
//        //   points:
//        //     A list, an array or any enumerable of Rhino.Geometry.Point3d.
//        //
//        //   tolerance:
//        //     A tolerance value. A default might be RhinoMath.ZeroTolerance.
//        //
//        // Returns:
//        //     true if points are on the same plane; false otherwise.
//        public static bool ArePointsCoplanar(IEnumerable<Point3i> points, double tolerance)
//        {
//            int count;
//            Point3i[] constArray = RhinoListHelpers.GetConstArray(points, out count);
//            if (count < 1 || constArray == null)
//            {
//                throw new ArgumentException("points must contain at least 1 point");
//            }

//            return UnsafeNativeMethods.RHC_RhinoArePointsCoplanar(count, constArray, tolerance);
//        }

//        //
//        // Summary:
//        //     Converts the string representation of a point to the equivalent Point3d structure.
//        //
//        // Parameters:
//        //   input:
//        //     The point to convert.
//        //
//        //   result:
//        //     The structure that will contain the parsed value.
//        //
//        // Returns:
//        //     true if successful, false otherwise.
//        public static bool TryParse(string input, out Point3i result)
//        {
//            result = Unset;
//            return UnsafeNativeMethods.RHC_RhinoParsePoint(input, ref result);
//        }

//        //
//        // Summary:
//        //     Orders a set of points so they will be connected in a "reasonable polyline" order.
//        //     Also, removes points from the list if their common distance exceeds a specified
//        //     threshold.
//        //
//        // Parameters:
//        //   points:
//        //     A list, an array or any enumerable of Rhino.Geometry.Point3d.
//        //
//        //   minimumDistance:
//        //     Minimum allowed distance among a pair of points. If points are closer than this,
//        //     only one of them will be kept.
//        //
//        // Returns:
//        //     The new array of sorted and culled points.
//        public static Point3i[] SortAndCullPointList(IEnumerable<Point3i> points, double minimumDistance)
//        {
//            int count;
//            Point3i[] array = RhinoListHelpers.GetConstArray(points, out count);
//            if (count < 1 || array == null)
//            {
//                return null;
//            }

//            if (!UnsafeNativeMethods.TLC_SortPointList(array, ref count, minimumDistance))
//            {
//                return null;
//            }

//            if (count < array.Length)
//            {
//                Point3i[] array2 = new Point3i[count];
//                Array.Copy(array, array2, count);
//                array = array2;
//            }

//            return array;
//        }
//    }

//}


