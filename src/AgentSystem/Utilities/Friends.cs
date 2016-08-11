using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Utilities;

namespace AgentSystem.Utilities
{
    public class Friends
    {
        public static void SortClosestNodes(Node myNode, Node[] otherNodes)
        {
            for (int write = 0; write < otherNodes.Length; write++)
            {
                for (int sort = 0; sort < otherNodes.Length - 1; sort++)
                {
                    if ((otherNodes[sort].Pt.DistanceTo(myNode.Pt) / otherNodes[sort].Charge) > (otherNodes[sort + 1].Pt.DistanceTo(myNode.Pt) / otherNodes[sort + 1].Charge))
                    {
                        Node temp = otherNodes[sort + 1];
                        otherNodes[sort + 1] = otherNodes[sort];
                        otherNodes[sort] = temp;
                    }
                }
            }
        }
    }
}
