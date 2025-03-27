#ifndef MATRIX3X3_H
#define MATRIX3X3_H

#include "Vector3.h"

class Matrix3x3 {
public:
    Matrix3x3() {
        *this = identity();
    }

    // Construct from 3 column vectors (TBN)
    Matrix3x3(const Vector3& col0, const Vector3& col1, const Vector3& col2) {
        mat[0][0] = col0.x; mat[1][0] = col0.y; mat[2][0] = col0.z;
        mat[0][1] = col1.x; mat[1][1] = col1.y; mat[2][1] = col1.z;
        mat[0][2] = col2.x; mat[1][2] = col2.y; mat[2][2] = col2.z;
    }

    static Matrix3x3 identity() {
        Matrix3x3 result;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                result.mat[i][j] = (i == j) ? 1.0f : 0.0f;
            }
        }
        return result;
    }

    // Multiply matrix by Vector3
    Vector3 operator*(const Vector3& v) const {
        return Vector3(
            mat[0][0] * v.x + mat[0][1] * v.y + mat[0][2] * v.z,
            mat[1][0] * v.x + mat[1][1] * v.y + mat[1][2] * v.z,
            mat[2][0] * v.x + mat[2][1] * v.y + mat[2][2] * v.z
        );
    }


private:
    float mat[3][3];
};

#endif //MATRIX3X3_H