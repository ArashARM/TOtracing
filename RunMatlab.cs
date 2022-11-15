using System;

using Grasshopper.Kernel;
using MathWorks.MATLAB.Engine;
using MathWorks.MATLAB.Exceptions;
using MathWorks.MATLAB.Types;


namespace TOtracing
{
    public class RunMatlab : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the RunMatlab class.
        /// </summary>
        public RunMatlab()
          : base("RunMatlab", "RunMatlab",
              "Description",
             "TO", "Matlab")
        {
        }

        //private MATLABEngine matlab = MATLABEngine.StartMATLAB();


        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("", "", "", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool startMatlab = false;
            DA.GetData(0, ref startMatlab);

            //if (startMatlab)
            //    matlab = MATLABEngine.StartMATLAB();
            //else if (!startMatlab)
            //    matlab.Dispose();
            //MATLABEngine.
            //MATLABEngine.TerminateEngineClient();
            //MATLABEngine.ConnectMATLAB("TopOpt");
            Console.Write("Starting MATLAB... ");
            string[] names = MATLABEngine.FindMATLAB();
            if (names.Length == 0)
                Console.Error.WriteLine("No shared MATLAB sessions found.");
            string myMATLAB = names[0]; // Pick the first
            try
            {
                using (dynamic matlab = MATLABEngine.ConnectMATLAB(myMATLAB))
                {
                    matlab.disp(new RunOptions(nargout: 0), "Hello, shared MATLAB.");
                }
            }
            catch (MATLABNotAvailableException)
            {
                Console.Error.WriteLine("Could not connect to MATLAB engine.");
            }

            // Call when you no longer need MATLAB Engine in your application.
            //MATLABEngine.TerminateEngineClient();

        }

        /// <summary>
        /// Provides an Icon for the component.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                //You can add image files to your project resources and access them like this:
                // return Resources.IconForThisComponent;
                return null;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("4D0F5803-7B5E-4504-888A-90F73E75B978"); }
        }
    }
}