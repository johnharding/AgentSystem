using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Rhino.Geometry;

namespace AgentSystem.Utilities
{
    public class Node
    {
        public Point3d Pt;
        public double Zone;
        private Vector3d Velocity;
        public double Cutoff;

        public Node(Point3d myPoint, double myZone, double cutOff)
        {
            Pt = new Point3d(myPoint);
            Zone = myZone;
            Cutoff = cutOff;
        }

        public void Damp(double d)
        {
            Velocity.X *= d;
            Velocity.Y *= d;
            Velocity.Z *= d;
        }

        public void UpdateVelocity(Vector3d newVel)
        {
            Velocity.X += newVel.X;
            Velocity.Y += newVel.Y;
            Velocity.Z += newVel.Z;
        }

        public void UpdatePosition(bool isSurface, Surface mySurface, Mesh myMesh)
        {
            Point3d newPoint = new Point3d(Pt.X + Velocity.X, Pt.Y + Velocity.Y, Pt.Z + Velocity.Z);
            if (isSurface)
            {
                double u, v;
                mySurface.ClosestPoint(newPoint, out u, out v);
                Pt = mySurface.PointAt(u, v);
            }
            else
            {
                Pt = myMesh.ClosestPoint(newPoint);
            }
        }
    }
}
