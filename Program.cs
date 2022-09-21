using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Accord.Math;
using Accord.IO;
using System.IO;

namespace Reader
{
    internal class Program
    {
        static void Main(string[] args)
        {

            var path = Directory.GetCurrentDirectory();
            // Let's assume they all currently reside in a "localPath" 
            // folder. So let's start by trying to load a 32-bit matrix:
            string pathInt32 = Path.Combine(path, "sensitivity.mat");

            // Create a .MAT reader for the file:
            var reader = new MatReader(pathInt32);

            // Let's check what is the name of the variable we need to load:
            string[] names = reader.FieldNames; // should be { "a" }

            // Ok, so we have to load the matrix called "a".

            // However, what if we didn't know the matrix type in advance?
            // In this case, we could use the non-generic version of Read:
            object unknown = reader.Read("dc");

            // And we could check it's type through C#:
            Type t = unknown.GetType(); // should be typeof(int[,])

            // Now we could either cast it to the correct type or
            // use the generic version of Read<> to read it again:
            double[,] matrix = reader.Read<double[,]>("dc");
            matrix = matrix.Reshape(MatrixOrder.FortranColumnMajor).Reshape(100, 300, MatrixOrder.FortranColumnMajor);
            // The a matrix should be equal to { 1, 2, 3, 4 }
            matrix.print();
            Console.ReadLine();

        }
    }
    public static class MatrixExtension
    {
        public static void print<T>(this T[,] matrix)
        {
            int rowLength = matrix.GetLength(0);
            int colLength = matrix.GetLength(1);

            if (matrix.GetType() == typeof(int[,]))
                for (int i = 0; i < rowLength; i++)
                {
                    for (int j = 0; j < colLength; j++)
                    {
                        Console.Write(string.Format("{0,4:0}", matrix[i, j]));
                    }
                    Console.Write(Environment.NewLine);
                }

            else
                for (int i = 0; i < rowLength; i++)
                {
                    for (int j = 0; j < colLength; j++)
                    {

                        Console.Write(string.Format("{0,10:0.0###}", matrix[i, j]));
                    }
                    Console.Write(Environment.NewLine);
                }

            Console.Write(Environment.NewLine);
        }
        public static void print<T>(this T[] vector, bool column = false)
        {
            int rowLength = vector.GetLength(0);


            if (column)
                for (int i = 0; i < rowLength; i++)
                {
                    if (vector.GetType() == typeof(int[]))
                        Console.Write(string.Format("{0,4}", vector[i]));
                    else
                        Console.Write(string.Format("{0,10:0.0###}", vector[i]));

                    //Console.Write(Environment.NewLine);
                }

            else
                for (int i = 0; i < rowLength; i++)
                {
                    if (vector.GetType() == typeof(int[]))
                        Console.Write(string.Format("{0,6}", vector[i]));
                    else
                        Console.Write(string.Format("{0,10:0.0###}", vector[i]));

                    Console.Write(Environment.NewLine);
                }

            Console.Write(Environment.NewLine);
        }



    }

}
