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
using Accord.Math;

// In order to load the result of this wizard, you will also need to
// add the output bin/ folder of this project to the list of loaded
// folder in Grasshopper.
// You can use the _GrasshopperDeveloperSettings Rhino command for that.

namespace TOtracing
{
    public class TOtracingCode : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public TOtracingCode()
          : base("TOtracing", "TOTR",
              "Tracing algorithm for TO",
              "TO", "Trace")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            //pManager.AddTextParameter("SensMatDir", "SensMatDir", "Senstivity Matrix Directory", GH_ParamAccess.item);
            pManager.AddNumberParameter("INT","Int","INt", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("PToutput", "PToutput", "Particle tracing output", GH_ParamAccess.list);
            //pManager.AddNumberParameter("Senvval", "Senvval", "Senvval", GH_ParamAccess.list);
            //pManager.AddPointParameter("Pts", "Pts", "Pts", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            //string path = "SenDire";
            List<NurbsCurve> CorCVS = new List<NurbsCurve>();
            double sss = 0;
            var MatrixS = MatlabRunner.ReadMatrix(print: false);
            PtFunctions.PrintMatrix2(MatrixS);
            DA.GetData(0, ref sss);

            List<Tuple<Tuple<double, double>, Point3d>> MatInfo = PtFunctions.MatToLoc2D(MatrixS, out List<Point3d> AllPts);

            Point3d Spt = new Point3d(0.5, 99.5, 0);
            int[,] TraceMatrix = new int[MatrixS.GetLength(0), MatrixS.GetLength(1)];
            double V = 0;
            int counter = 0;
            //List<NurbsCurve> CCuve = PtFunctions.ParTrace(MatrixS, MatInfo, Spt, TraceMatrix, 1, Math.Sqrt(2));
            while (true)
            {
                //if (counter > 2)
                //    break;
                //if (counter != 0)
                //{
                //    Spt = new Point3d(1, 0, 0);
                //    //Spt = PtFunctions.ExplorTracPt(MatInfo, MatrixS, TraceMatrix, Math.Sqrt(2));
                //}

                //if (counter == 1)
                //{
                //    Spt = new Point3d(0, 0, 0);
                //    //Spt = PtFunctions.ExplorTracPt(MatInfo, MatrixS, TraceMatrix, Math.Sqrt(2));
                //}
                if (counter != 0)
                {
                    //Spt = new Point3d(299, 1, 0);
                    Spt = PtFunctions.ExplorTracPt(MatInfo, MatrixS, TraceMatrix, Math.Sqrt(2));
                }
                List<NurbsCurve> CCuve = PtFunctions.ParTrace(MatrixS, MatInfo, Spt, TraceMatrix, 1, Math.Sqrt(2), AllPts);
                V = PtFunctions.ComputeVol2d(TraceMatrix);

                CorCVS.AddRange(CCuve);
                if (V >= 3000)
                    break;
                counter++;
            }


            DA.SetDataList(0, CorCVS);


        }

        /// <summary>
        /// Provides an Icon for every component that will be visible in the User Interface.
        /// Icons need to be 24x24 pixels.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                // You can add image files to your project resources and access them like this:
                //return Resources.IconForThisComponent;
                return null;
            }
        }

        /// <summary>
        /// Each component must have a unique Guid to identify it. 
        /// It is vital this Guid doesn't change otherwise old ghx files 
        /// that use the old ID will partially fail during loading.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("02fa0bc7-5c2a-48c6-8513-1a9cb0ee7f44"); }
        }
    }

  
}
