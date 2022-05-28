#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;

int main(int argc, char **argv)
{
    Matrix3d rotation_matrix = Matrix3d::Identity();

    AngleAxisd rotation_vector(M_PI / 4, Vector3d(0, 0, 1));

    cout.precision(3);

    cout << "rotation matrix \n " << rotation_vector.matrix() << endl;

    rotation_matrix = rotation_vector.toRotationMatrix();

    Vector3d v(1, 0, 0);

    Vector3d v_rotated = rotation_vector * v;

    cout << v_rotated.transpose() << endl;

    v_rotated = rotation_matrix * v;

    cout << v_rotated.transpose() << endl;

    Vector3d eulaer_angles = rotation_matrix.eulerAngles(2, 1, 0);

    cout << "euler angles: " << eulaer_angles.transpose() << endl;

    Isometry3d T = Isometry3d::Identity();

    T.rotate(rotation_vector);

    T.pretranslate(Vector3d(1, 1, 1));

    cout << "T: " << endl
         << T.matrix() << endl;

    Vector3d v_transformed = T * v;

    cout << "v_transformed: " << v_transformed.transpose() << endl;

    Quaterniond q = Quaterniond(rotation_vector);

    cout << "q: " << q.coeffs().transpose() << endl;

    q = Quaterniond(rotation_matrix);

    cout << "q: " << q.coeffs().transpose() << endl;

    v_rotated = q * v;

    cout << "v_rotated: " << v_rotated.transpose() << endl;

    return 0;
}