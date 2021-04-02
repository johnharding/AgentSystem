using System;
using System.Drawing;
using Grasshopper.Kernel;
using AgentSystem.Properties;
using System.Collections.Generic;
using Rhino;
using Grasshopper;
using Rhino.Geometry;

namespace AgentSystem
{
    public class Repulsion : GH_Component
    {
        int counter;
        bool isSurface;
        Surface mySurface;
        Mesh myMesh;
        object myManifold;
        List<Point3d> myPoints = new List<Point3d>();
        List<Point3d> localPoints = new List<Point3d>();
        List<Vector3d> Velocity = new List<Vector3d>();
        List<double> cutoffs = new List<double>();
        double k;
        double d;

        public Repulsion()
            : base("Repulsion (timer)", "Node Repulsion", "Repels nodes on a surface or mesh with damping", "Extra", "Rosebud")
        {
            k = 0;
            d = 0;
            counter = 0;
            mySurface = null;
            myMesh = null;
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Manifold", "M", "The reference manifold (mesh/surface)", GH_ParamAccess.item);
            pManager.AddPointParameter("Points", "P", "The input 3d points", GH_ParamAccess.list);
            pManager.AddNumberParameter("Cutoff", "C", "A list of minimum threshold radii. Default is zero", GH_ParamAccess.list, 0.0);
            pManager.AddNumberParameter("Initial force", "K", "The initial force between particles", GH_ParamAccess.item, 0.1);
            pManager.AddNumberParameter("Damping", "D", "Damping constant", GH_ParamAccess.item, 0.99);
            pManager.AddBooleanParameter("Reset", "R", "Reset the system", GH_ParamAccess.item, false);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.Register_PointParam("Repelled Points", "R", "The repelled points", GH_ParamAccess.list);
        }

        //SolveInstance is a method in the GH_Component class
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool reset = false;
            DA.GetData("Reset", ref reset);
            if (reset)
            {
                myPoints.Clear();
                localPoints.Clear();
                Velocity.Clear();
                cutoffs.Clear();
                counter = 0;
                return;
            }

            // Get the stuff from the component inputs if counter = 0
            if (counter == 0)
            {
                DA.GetData("Manifold", ref myManifold);
                DA.GetDataList<Rhino.Geometry.Point3d>("Points", myPoints);
                localPoints.AddRange(myPoints);

                DA.GetDataList<double>("Cutoff", cutoffs);

                // Make the cutoff list equal the point list somehow.
                if (cutoffs.Count != myPoints.Count)
                {
                    cutoffs.Clear();
                    if (cutoffs.Count == 1)
                    {
                        for (int i = 0; i < myPoints.Count; i++)
                        {
                            cutoffs.Add(cutoffs[0]);
                        }
                    }
                    else
                    {
                        for (int i = 0; i < myPoints.Count; i++)
                        {
                            cutoffs.Add(0.0);
                        }
                    }
                }

                DA.GetData("Initial force", ref k);
                DA.GetData("Damping", ref d);

                if (myManifold.GetType().ToString().Equals("Grasshopper.Kernel.Types.GH_Surface"))
                {
                    Grasshopper.Kernel.GH_Convert.ToSurface(myManifold, ref mySurface, GH_Conversion.Primary);
                    isSurface = true;
                }
                else if (myManifold.GetType().ToString().Equals("Grasshopper.Kernel.Types.GH_Mesh"))
                {
                    Grasshopper.Kernel.GH_Convert.ToMesh(myManifold, ref myMesh, GH_Conversion.Primary);
                    isSurface = false;
                }

                // Initialise the velocities to zero
                // TODO: Parallel arrays are BAD. Replace with node class.
                for (int i = 0; i < localPoints.Count; i++)
                    Velocity.Add(new Rhino.Geometry.Vector3d(0.0, 0.0, 0.0));
            }

            // Return if nothing has been set
            if (mySurface == null && myMesh == null)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "No Surface or Mesh detected");
                return;
            }
            if (myPoints == null || myPoints.Count < 2)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "You must use 2 or more points");
                return;
            }

            // Get closest point
            for (int i = 0; i < localPoints.Count; i++)
            {
                if (isSurface)
                {
                    mySurface.ClosestPoint(localPoints[i], out double u, out double v);
                    localPoints[i] = mySurface.PointAt(u, v);
                }
                else
                {
                    localPoints[i] = myMesh.ClosestPoint(localPoints[i]);
                }
            }

            //Calculate

            //for (int i = 0; i < localPoints.Count; i++)
            //{


            //    // Set the velocities according to a margin
            //    Rhino.Geometry.Vector3d newVel;
            //    if (localPoints[i].DistanceTo(localPoints[ID]) < (cutoffs[i]+cutoffs[ID]))
            //    {
            //        newVel = new Vector3d(localPoints[i].X - localPoints[ID].X, localPoints[i].Y - localPoints[ID].Y, localPoints[i].Z - localPoints[ID].Z);
            //        newVel.Unitize();
            //        // Now damp (decrease time step)
            //        newVel *= k;
            //    }
            //    else
            //    {
            //        newVel = new Vector3d(0,0,0);
            //    }

            //    // Replace existing (could accumulate?...)
            //    Velocity[i] = new Vector3d(newVel);

            //}


            for (int i = 0; i < localPoints.Count; i++)
            {
                //Clear the velocity vector
                Velocity[i] = new Vector3d(0, 0, 0);

                for (int j = 0; j < localPoints.Count; j++)
                {
                    if (i != j)
                    {
                        if (localPoints[i].DistanceTo(localPoints[j]) < (cutoffs[i] + cutoffs[j]))
                        {
                            Vector3d newVel = new Vector3d(localPoints[i].X - localPoints[j].X, localPoints[i].Y - localPoints[j].Y, localPoints[i].Z - localPoints[j].Z);
                            newVel.Unitize();
                            Velocity[i] += newVel;
                        }
                    }
                }



                //NEAREST NEIGH BIT
                double minDis = 1000000000;
                int ID = 0;

                for (int j = 0; j < localPoints.Count; j++)
                {
                    double myDist = localPoints[i].DistanceTo(localPoints[j]);
                    if (myDist < minDis && i != j)
                    {
                        minDis = myDist;
                        ID = j;
                    }
                }

                Vector3d newVel2 = new Vector3d(localPoints[i].X - localPoints[ID].X, localPoints[i].Y - localPoints[ID].Y, localPoints[i].Z - localPoints[ID].Z);
                newVel2.Unitize();
                // This should be the minor part of the repulsion
                newVel2 *= 0.3333;
                Velocity[i] += newVel2;


                // Unitise the sum
                Velocity[i].Unitize();

                //multiply by the damped stiffness constant
                Velocity[i] *= k;
            }



            //Update node positions
            for (int i = 0; i < localPoints.Count; i++)
            {
                Point3d newPoint = new Point3d(localPoints[i].X + Velocity[i].X, localPoints[i].Y + Velocity[i].Y, localPoints[i].Z + Velocity[i].Z);
                if (isSurface)
                {
                    mySurface.ClosestPoint(newPoint, out double u, out double v);
                    localPoints[i] = mySurface.PointAt(u, v);
                }
                else
                {
                    localPoints[i] = myMesh.ClosestPoint(newPoint);
                }
            }

            // Damping of stiffness constant
            k *= d;
            counter++;

            // Output points
            DA.SetDataList(0, localPoints);

        }

        public override Guid ComponentGuid
        {
            get { return new Guid("c341af05-8df0-4e1e-92b3-472dd6ac1807"); }
        }


        protected override Bitmap Icon
        {
            get
            {
                return AgentSystem.Properties.Resources.Repulsion03;
            }
        }

    }
}
