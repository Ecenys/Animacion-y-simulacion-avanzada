using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

    public class SpringComparer : IEqualityComparer<Spring>
    {

        /// <summary>
        /// Compares two edges by their main vertices indexes
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        public bool Equals(Spring a, Spring b)
        {
            if ((a.nodeA.Pos == b.nodeA.Pos) && (a.nodeB.Pos == b.nodeB.Pos))
            {
                return true;
            }

            return false;
        }

        public int GetHashCode(Spring spring)
        {
            int hCode = 4756 * spring.nodeA.index + 47357 * spring.nodeB.index;
            return hCode.GetHashCode();
        }
    }
