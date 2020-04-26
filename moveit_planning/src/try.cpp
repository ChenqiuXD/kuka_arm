#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

int main(int argc, char** argv)
{
    tf::Quaternion q1, q2, q3;
    double result, PI = 3.1416;
    q1.setEuler(0, 0, 0);
    q2.setEuler(-PI/2, PI/2, 0);
    tf::Vector3 ex = q1.getAxis();
    double angle = q1.getAngle();
    cout << "q1 axis is: " << ex.x() << " "<< ex.y() << " "<< ex.z() <<" "<<"and the angle is: "<<angle <<endl;
    ex = q2.getAxis();
    cout << "q2's value is: " << ex.x() << " "<< ex.y() << " "<< ex.z() <<" "<<q2.getW()<<endl;

    Eigen::Quaterniond t3;
    tf::quaternionTFToEigen(q2,t3);
    cout << "t3's value is:" << t3.x()<<" "<<t3.y()<<" "<<t3.z()<<" "<<t3.w()  << endl;
    // cout << "t3's matrix is: " << t3.Vector3 << endl;
    Eigen::Matrix3d a = t3.toRotationMatrix();
    cout << "First a is:\n" <<a << endl;

    Eigen::Matrix3d xRot;
    xRot(0,0)=1;xRot(0,1)=0;xRot(0,2)=0;
    xRot(1,0)=0;xRot(1,1)=0;xRot(1,2)=-1;
    xRot(2,0)=0;xRot(2,1)=1;xRot(2,2)=0;

    a = xRot*a;
    cout << "Now a is:\n" << a << endl;
}