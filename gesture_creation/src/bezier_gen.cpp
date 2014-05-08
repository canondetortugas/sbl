#include <gesture_creation/bezier_gen.h>

double const TOLERANCE = 0.00001;
double const INV_TOLERANCE = 1/TOLERANCE;

double truncate(double conver)
{
  conver = conver * INV_TOLERANCE;
  conver = (conver > (floor(conver)+0.5f)) ? ceil(conver) : floor(conver);
  conver = conver / INV_TOLERANCE;
  return conver;
}

double tolerance(double eqn_root)
{
  return TOLERANCE < fabs(truncate(eqn_root));
}

using namespace std;

// ---------------------------------------------------------------------------------
/// @Synopsis:  Given bezier_coordinate, which contains parameters for a sequence of bezier curves
/// creates a sequence of tf frames by discretizing the curves with timestep 'step'.
///		
/// @Param bezier_coordinate: A list containing all the bezier cooridinates
/// @Param gesture_coordinate: A pre-initilaized vector of gesture coordinates 
/// @Param step: The step size, this is required to calculate the bezier curve
// ----------------------------------------------------------------------------
void Bezier6Dof::create_gestures(list<Bezier6Dof> &bezier_coordinate, vector<SixDim> &gesture_coordinate, int step)
{
    gesture_coordinate.clear();

    list<Bezier6Dof>::iterator itr;

    for(itr = bezier_coordinate.begin() ; itr !=  bezier_coordinate.end(); itr++)
   {
      
      double t = (itr->getEnd().time - itr->getStart().time) / 3;
      double t1 = itr->getStart().time;
      double t2 = t1 + t;
      itr->setCtrl1Time(t2);
      double t3 = t2 + t;
      itr->setCtrl2Time(t3);
      double t4 = itr->getEnd().time;

      Bezier1D b1 = itr->getBx(t1, t2, t3, t4);
      Bezier1D b2 = itr->getBy(t1, t2, t3, t4);
      Bezier1D b3 = itr->getBz(t1, t2, t3, t4);
      Bezier1D b4 = itr->getBroll(t1, t2, t3, t4);
      Bezier1D b5 = itr->getBpitch(t1, t2, t3, t4);
      Bezier1D b6 = itr->getByaw(t1, t2, t3, t4);

      //Stepsize
      double stsize = (1/(double)step);
      //t parameter of the bezier curve
      double bezier_t = 0;
 
      for(int i = 0; i<=step; i++){
        bezier_t = stsize*i;
	/// Evaluate each bezier curve at time (bezier_t)
        gesture_coordinate.push_back(itr->get6Dof(b1, b2, b3, b4, b5, b6, bezier_t));
      }
    }
}




// --------------------------------------------------------------------------------------
/// @Synopsis: Computes a SixDimensional co-ordinate on the 6 dimension bezier curve. 
///		It takes in the x-axis, i.e.., time, on each 
///		dimension and gets the corressponding bezier curve for that dimension. This only
///		works for a single curve, not a sequence.
///		
///
/// @Param tVal: This is the value computed by the function findTvalue
/// @Param t1: The x-coordinate of the start waypoint in each dimension
/// @Param t2: The x-coordinate of the first controlpoint in each dimension
/// @Param t3: The x-coordinate of the second controlpoint in each dimension
/// @Param t4: The x-coordinate of the end waypoint in each dimension
/// @Param coord: This parameter contains the coordinate on the 6 dimensional bezier Curve
// ---------------------------------------------------------------------------------------
void Bezier6Dof::find6Dof(double tVal, double t1, double t2, double t3, double t4, SixDim *coord)
{ 


  Bezier1D b1 = this->getBx(t1, t2, t3, t4);
      
  Bezier1D b2 = this->getBy(t1, t2, t3, t4);
      
  Bezier1D b3 = this->getBz(t1, t2, t3, t4);
      
  Bezier1D b4 = this->getBroll(t1, t2, t3, t4);
      
  Bezier1D b5 = this->getBpitch(t1, t2, t3, t4);
      
  Bezier1D b6 = this->getByaw(t1, t2, t3, t4);

  *coord  = this->get6Dof(b1, b2, b3, b4, b5, b6, tVal);

}

// --------------------------------------------------------------------------
/// @Synopsis: Inverts the bezier curve equation. Given parameters (x0, x1, x2, x3) and an
/// amplitude xVal, find t such that B(t) = xVal.
///
/// @Param x0: Value of the first point, waypoint P1
/// @Param x1: Value of the second point, control point P2
/// @Param x2: Value of the third point, control point P3
/// @Param x3: Value of the fourth point, waypoint P4
/// @Param xVal: The value for which the position on the curve needs to be determined
///
/// @Returns: Returns the "t" parameter of the bezier equation
// ----------------------------------------------------------------------------
double Bezier6Dof::findTvalue(double x0, double x1, double x2, double x3, double xVal)
{
  //bothered only about the real values
  double a, b, c, d;
  double root1, root2, root3;

  a =  x3 - 3*(x2) + 3*(x1) - x0;
  b = 3*(x2) - 6*(x1) + 3*(x0);
  c = 3*(x1 - x0);
  d = x0-xVal;
  double rootfloor = 0;
  double rootceil = 1;

  static double midpoint;
  double x;
  double roots[3] = {0, 0, 0};
  int j = 0, k = 0;
  midpoint = (rootceil + rootfloor) / 2.0;
  if( tolerance(a * pow(x, 3) + b * pow(x, 2) + c * x + d) < TOLERANCE )
  roots[0] = x;
  x = rootfloor;
  do {
    if( tolerance(a * pow(x, 3) + b * pow(x, 2) + c * x + d) < TOLERANCE ) {
      roots[j] = x;
      j++;
  }
    x += TOLERANCE;
  }while(x <= midpoint);
  x = rootceil;
  do {
    if( tolerance(a * pow(x, 3) + b * pow(x, 2) + c * x + d) < TOLERANCE ) {
      roots[j] = x;
      j++;
    }
    x -= TOLERANCE;
  }while(midpoint <= x);
  if(roots[0] == 0 && roots[1] == 0 && roots[2] == 0) {
    cout<<"Roots not found between "<<rootfloor<<" and "<<rootceil<<". Please try again"<<endl;
  }
  else {
    //for(k = 0; k < j; k++)
    //cout<<roots[k]<<endl;
  }
  return roots[0];
}
  

// --------------------------------------------------------------------------
/// @Synopsis: Evaluates a sequence of six-dimensional bezier curves at a given time
///
/// @Param bezier_coordinate: List of bezier coordinates
/// @Param time: The time for which the coordinates are requied
///
/// @Returns: TF frame for the computed co-ordinate
// ----------------------------------------------------------------------------
// tf::Transform static Bezier6Dof::findBezierCoordinates(std::list<Bezier6Dof> &bezier_coordinate, double time)
// {
//   // ROS_INFO("Reconfigure Request: %f", time);

//   if(flag){
//     SixDim *b = new SixDim();

//     list<Bezier6Dof>::iterator itr;
//     list<Bezier6Dof>::iterator prev;


//     for(itr = prev = bezier_coordinate.begin(); itr != bezier_coordinate.end(); itr++){
  
//       if(time >= itr->getStart().time  && time <= itr->getEnd().time){

//         itr->find6Dof(findTValue(itr->getStart().time, itr->getCtrl1().time, 
//               itr->getCtrl2().time, itr->getEnd().time, config.double_param), 
//             itr->getStart().time, itr->getCtrl1().time, itr->getCtrl2().time, itr->getEnd().time, b);
    
//         makeMarker(tf::Vector3(b->x, b->y, b->z), tf::createQuaternionFromRPY(b->roll, b->pitch, b->yaw), name);

//     tf::Transform t;

//     ros::Time time = ros::Time::now();
//     t.setOrigin(tf::Vector3(b->x, b->y, b->z));
//     t.setRotation(tf::Quaternion(b->roll, b->pitch, b->yaw));
//     break;
//     }
//     prev = itr;
//     }
//   }
//   return t;
// }

  



