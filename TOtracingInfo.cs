using Grasshopper.Kernel;
using System;
using System.Drawing;

namespace TOtracing
{
    public class TOtracingInfo : GH_AssemblyInfo
    {
        public override string Name
        {
            get
            {
                return "TOtracing";
            }
        }
        public override Bitmap Icon
        {
            get
            {
                //Return a 24x24 pixel bitmap to represent this GHA library.
                return null;
            }
        }
        public override string Description
        {
            get
            {
                //Return a short string describing the purpose of this GHA library.
                return "";
            }
        }
        public override Guid Id
        {
            get
            {
                return new Guid("4c4ab6f5-d2ea-4f52-bed8-96c78985172f");
            }
        }

        public override string AuthorName
        {
            get
            {
                //Return a string identifying you or your company.
                return "";
            }
        }
        public override string AuthorContact
        {
            get
            {
                //Return a string representing your preferred contact details.
                return "";
            }
        }
    }
}
