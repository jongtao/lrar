using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace triangulation_test
{
    class Program
    {
        static void CircleApprox(double x0, double y0, double r0,
            double x1, double y1, double r1,
            double x2, double y2, double r2,
            ref double sol_x, ref double sol_y)
        {
            // Get ratios for circle A
            double dx_a = x1 - x0;
            double dy_a = y1 - y0;
            double ratio_a = r1 / r0;

            dx_a = dx_a / ratio_a;
            dy_a = dy_a / ratio_a;

            // Compute new approximate circle A
            double x_a = x0 + dx_a;
            double y_a = y0 + dy_a;
            double r_a = (r1 + r2) / 2;

            // Get ratios for circle B
            double dx_b = x2 - x_a;
            double dy_b = y2 - y_a;
            double ratio_b = r2 / r_a;

            dx_b = dx_b / ratio_b;
            dy_b = dy_b / ratio_b;


            // Get approximate point
            double x_b = x_a + dx_b;
            double y_b = y_a + dy_b;

            sol_x = x_b;
            sol_y = y_b;
        } // CircleApprox()


        static bool CircleIntersect(double x0, double y0, double r0,
            double x1, double y1, double r1,
            double x2, double y2, double r2,
            ref double sol_x, ref double sol_y, double tolerance)
        {
            double a, dx, dy, d, h, rx, ry;
            double point2_x, point2_y;

            /* dx and dy are the vertical and horizontal distances between
            * the circle centers.
            */
            dx = x1 - x0;
            dy = y1 - y0;

            /* Determine the straight-line distance between the centers. */
            d = Math.Sqrt((dy * dy) + (dx * dx));

            /* Check for solvability. */
            if (d > (r0 + r1))
            {
                /* no solution. circles do not intersect. */
                return false;
            }
            if (d < Math.Abs(r0 - r1))
            {
                /* no solution. one circle is contained in the other */
                return false;
            }

            /* 'point 2' is the point where the line through the circle
            * intersection points crosses the line between the circle
            * centers.
            */

            /* Determine the distance from point 0 to point 2. */
            a = ((r0 * r0) - (r1 * r1) + (d * d)) / (2.0 * d);

            /* Determine the coordinates of point 2. */
            point2_x = x0 + (dx * a / d);
            point2_y = y0 + (dy * a / d);

            /* Determine the distance from point 2 to either of the
            * intersection points.
            */
            h = Math.Sqrt((r0 * r0) - (a * a));

            /* Now determine the offsets of the intersection points from
            * point 2.
            */
            rx = -dy * (h / d);
            ry = dx * (h / d);

            /* Determine the absolute intersection points. */
            double intersectionPoint1_x = point2_x + rx;
            double intersectionPoint2_x = point2_x - rx;
            double intersectionPoint1_y = point2_y + ry;
            double intersectionPoint2_y = point2_y - ry;

            Console.WriteLine("INTERSECTION Circle1 AND Circle2:" +
                "(" + intersectionPoint1_x + "," + intersectionPoint1_y + ")" +
                " AND (" + intersectionPoint2_x + "," + intersectionPoint2_y + ")");

            /* Lets determine if circle 3 intersects at either of the above intersection points. */
            dx = intersectionPoint1_x - x2;
            dy = intersectionPoint1_y - y2;
            double d1 = Math.Sqrt((dy * dy) + (dx * dx));

            dx = intersectionPoint2_x - x2;
            dy = intersectionPoint2_y - y2;
            double d2 = Math.Sqrt((dy * dy) + (dx * dx));

            if (Math.Abs(d1 - r2) < tolerance)
            {
                sol_x = intersectionPoint1_x;
                sol_y = intersectionPoint1_y;

                Console.WriteLine("INTERSECTION Circle1 AND Circle2 AND Circle3:" + 
                    "(" + intersectionPoint1_x + "," + intersectionPoint1_y + ")");
            }
            else if (Math.Abs(d2 - r2) < tolerance)
            {
                sol_x = intersectionPoint1_x;
                sol_y = intersectionPoint1_y;

                Console.WriteLine("INTERSECTION Circle1 AND Circle2 AND Circle3:" + 
                    "(" + intersectionPoint2_x + "," + intersectionPoint2_y + ")");
            }
            else
            {
                Console.WriteLine("INTERSECTION Circle1 AND Circle2 AND Circle3: NONE");
                return false;
            }

            return true;
        }


        static void get_circle_sol(double x1, double y1, double r1,
            double x2, double y2, double r2,
            double x3, double y3, double r3,
            ref double sol_x, ref double sol_y, double tolerance)
        {

            bool converges = false;
            converges = CircleIntersect(
                x1, y1, r1,
                x2, y2, r2,
                x3, y3, r3,
                ref sol_x, ref sol_y, tolerance);

            if (!converges)
            {
                Console.WriteLine("Second Try;");
                // Second Try
                converges = CircleIntersect(
                    x1, y1, r1,
                    x3, y3, r3,
                    x2, y2, r2,
                    ref sol_x, ref sol_y, tolerance);

                if (!converges)
                {
                    Console.WriteLine("Third Try;");
                    // Third Try
                    converges = CircleIntersect(
                        x2, y2, r2,
                        x3, y3, r3,
                        x1, y1, r1,
                        ref sol_x, ref sol_y, tolerance);

                } // third try
            } // second try

            if (!converges)
            {
                // Give up with approximation
                CircleApprox(
                    x1, y1, r1,
                    x2, y2, r2,
                    x3, y3, r3,
                    ref sol_x, ref sol_y);
                Console.WriteLine("Failed To converge: (" + sol_x + "," + sol_y + ")");
            } // give up
            else
                Console.WriteLine("Converge: (" + sol_x + "," + sol_y + ")");
        } // get_circle_sol()



        static void Main(string[] args)
        {
            double x1 = 0.0;
            double y1 = 1.0;
            double r1 = 0.9; //1

            double x2 = 0.0;
            double y2 = -1.0;
            double r2 = 1; //1

            double x3 = 2.0;
            double y3 = 0.0;
            double r3 = 2; //2

            double tolerance = 0.1;
            double sol_x = 0.0;
            double sol_y = 0.0;


            get_circle_sol(
                x1, y1, r1,
                x2, y2, r2,
                x3, y3, r3,
                ref sol_x, ref sol_y, tolerance);

            Console.ReadKey();
        }
    }
}
