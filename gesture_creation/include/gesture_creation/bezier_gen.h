#ifndef _BEZIER_GEN_H
#define _BEZIER_GEN_H

#include<math.h>
#include<fstream>
#include<list>
#include<vector>

#include <tf/transform_datatypes.h>

extern double const TOLERANCE;
extern double const INV_TOLERANCE;

// --------------------------------------------------------------------------
/// @Synopsis: Data Structure to hold the x and y cordinates  
// ----------------------------------------------------------------------------
struct Pt{
  double x, y;

  Pt(double x, double y):
    x(x), y(y)
  {}
};

// --------------------------------------------------------------------------
/// @Synopsis: A Class to define the Bezier Curve parameter in each Dimension
// ----------------------------------------------------------------------------
class Bezier1D{
  Pt start, ctrl1, ctrl2, end;

  public:
    Bezier1D(const Pt &start, const Pt &ctrl1, const Pt &ctrl2, const Pt &end):
      start(start), ctrl1(ctrl1), ctrl2(ctrl2), end(end)
    {}
    
    double getY(double bezier_t){
      double y;
      y = pow((1-bezier_t), 3) * start.y + 3 * pow((1-bezier_t), 2) * bezier_t * ctrl1.y
          + 3 * pow(bezier_t, 2) * (1-bezier_t) * ctrl2.y + pow(bezier_t, 3) * end.y;
      return y;
    }

};

// --------------------------------------------------------------------------
/// @Synopsis: A structure to store the Six Dimensional Parameters
// ----------------------------------------------------------------------------
struct SixDim{
    double x, y, z, roll, pitch, yaw, time;
    std::string name;

    SixDim(double x, double y, double z, double roll, double pitch, double yaw, double time, std::string name):
    x(x), y(y), z(z), roll(roll), pitch(pitch), yaw(yaw), time(time), name(name)
    {}
    SixDim():
      x(0), y(0), z(0), roll(0), pitch(0), yaw(0), time(0), name("name")
    {}
    void setName(std::string nm)
    {
      name = nm;
    }
};


// ---------------------------------------------------------------------------------
/// @Synopsis: Structure to hold the Bezier Coordinates of a Six Dimensional Bezier
// ---------------------------------------------------------------------------------
class Bezier6Dof{
  
  SixDim start, ctrl1, ctrl2, end;

  double findTvalue(double x0, double x1, double x2, double x3, double xVal); 

  public:
    Bezier6Dof(const SixDim &start, const SixDim &ctrl1, const SixDim &ctrl2, const SixDim &end):
      start(start), ctrl1(ctrl1), ctrl2(ctrl2), end(end)
      {}
    Bezier6Dof(){}
    
    void static create_gestures(std::list<Bezier6Dof> &bezier_coordinate, std::vector<SixDim> &gesture_coordinate, 
    int step);

    tf::Transform static findBezierCoordinates(std::list<Bezier6Dof> &bezier_coordinate, double time);

    void find6Dof(double tVal, double t1, double t2, double t3, double t4, SixDim *coord);

    SixDim getStart(){
      return start;
    }

    SixDim getCtrl1(){
      return ctrl1;
    }
    SixDim getCtrl2(){
      return ctrl2;
    }
    SixDim getEnd(){
      return end;
    }

    void setStart(double x, double y, double z, double roll, double pitch, double yaw){
      start.x = x;
      start.y = y;
      start.z = z;
      start.roll = roll;
      start.pitch = pitch;
      start.yaw = yaw; 
    }

    void setCtrl1(double x, double y, double z, double roll, double pitch, double yaw){
      ctrl1.x = x;
      ctrl1.y = y;
      ctrl1.z = z;
      ctrl1.roll = roll;
      ctrl1.pitch = pitch;
      ctrl1.yaw = yaw; 
    }

    void setCtrl2(double x, double y, double z, double roll, double pitch, double yaw){
      ctrl2.x = x;
      ctrl2.y = y;
      ctrl2.z = z;
      ctrl2.roll = roll;
      ctrl2.pitch = pitch;
      ctrl2.yaw = yaw; 
    }
    void setEnd(double x, double y, double z, double roll, double pitch, double yaw){
      end.x = x;
      end.y = y;
      end.z = z;
      end.roll = roll;
      end.pitch = pitch;
      end.yaw = yaw; 
    }


    void setStartName(std::string name)
    {
      start.name = name;
    }
    
    void setCtrl1Name(std::string name)
    {
      ctrl1.name = name;
    }

    void setCtrl2Name(std::string name)
    {
      ctrl2.name = name;
    }
    
    void setEndName(std::string name)
    {
      end.name = name;
    }

    void setStartTime(double t)
    {
      start.time = t;
    }

    void setCtrl1Time(double t)
    {
      ctrl1.time = t;
    }

    void setCtrl2Time(double t)
    {
      ctrl2.time = t;
    }

    void setEndTime(double t)
    {
      end.time = t ;
    }


    Bezier1D getBx(const double &t1, const double &t2, const double &t3, const double &t4)
    {
      return Bezier1D(Pt(t1, start.x), Pt(t2, ctrl1.x), Pt(t3, ctrl2.x), Pt(t4, end.x));
    }

    Bezier1D getBy(const double &t1, const double &t2, const double &t3, const double &t4)
    {
      return Bezier1D(Pt(t1, start.y), Pt(t2, ctrl1.y), Pt(t3, ctrl2.y), Pt(t4, end.y));
    }

    Bezier1D getBz(const double &t1, const double &t2, const double &t3, const double &t4)
    {
      return Bezier1D(Pt(t1, start.z), Pt(t2, ctrl1.z), Pt(t3, ctrl2.z), Pt(t4, end.z));
    }

    Bezier1D getBroll(const double &t1, const double &t2, const double &t3, const double &t4)
    {
      return Bezier1D(Pt(t1, start.roll), Pt(t2, ctrl1.roll), Pt(t3, ctrl2.roll), Pt(t4, end.roll));
    }

    Bezier1D getBpitch(const double &t1, const double &t2, const double &t3, const double &t4)
    {
      return Bezier1D(Pt(t1, start.pitch), Pt(t2, ctrl1.pitch), Pt(t3, ctrl2.pitch), Pt(t4, end.pitch));
    }

    Bezier1D getByaw(const double &t1, const double &t2, const double &t3, const double &t4)
    {
      return Bezier1D(Pt(t1, start.yaw), Pt(t2, ctrl1.yaw), Pt(t3, ctrl2.yaw), Pt(t4, end.yaw));
    }

    SixDim get6Dof(Bezier1D b1, Bezier1D b2, Bezier1D b3, Bezier1D b4, Bezier1D b5, Bezier1D b6, double bezier_t)
    {
      return SixDim(b1.getY(bezier_t), b2.getY(bezier_t), b3.getY(bezier_t), b4.getY(bezier_t), b5.getY(bezier_t), b6.getY(bezier_t), 0, "name");
    }
};

#endif
