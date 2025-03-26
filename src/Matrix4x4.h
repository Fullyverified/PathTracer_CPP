#ifndef MATRIX4X4_H
#define MATRIX4X4_H

#include "Vector3.h"

class Matrix4x4 {
public:
    Matrix4x4() {
        // identity matrix
        *this = identity();
    }

    static Matrix4x4 identity() {
        Matrix4x4 result;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                result.mat[i][j] = (i == j) ? 1.0f : 0.0f;
            }
        }
        return result;
    }

    Matrix4x4 operator*(const Matrix4x4& other) const {
        Matrix4x4 result;
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                result.mat[row][col] = 0.0f;
                for (int k = 0; k < 4; ++k) {
                    result.mat[row][col] += mat[row][k] * other.mat[k][col];
                }
            }
        }
        return result;
    }


    static Matrix4x4 translate(const Vector3& t) {
        Matrix4x4 result = identity();
        result.mat[0][3] = t.x;
        result.mat[1][3] = t.y;
        result.mat[2][3] = t.z;
        return result;
    }

    static Matrix4x4 scale(const Vector3& s) {
        Matrix4x4 result = identity();
        result.mat[0][0] = s.x;
        result.mat[1][1] = s.y;
        result.mat[2][2] = s.z;
        return result;
    }

    static Matrix4x4 rotateX(float angleRadians) {
        Matrix4x4 result = identity();
        float c = cos(angleRadians);
        float s = sin(angleRadians);

        result.mat[1][1] = c;
        result.mat[1][2] = -s;
        result.mat[2][1] = s;
        result.mat[2][2] = c;

        return result;
    }

    static Matrix4x4 rotateY(float angleRadians) {
        Matrix4x4 result = identity();
        float c = cos(angleRadians);
        float s = sin(angleRadians);
        result.mat[0][0] = c;
        result.mat[0][2] = s;
        result.mat[2][0] = -s;
        result.mat[2][2] = c;
        return result;
    }

    static Matrix4x4 rotateZ(float angleRadians) {
        Matrix4x4 result = identity();
        float c = cos(angleRadians);
        float s = sin(angleRadians);
        result.mat[0][0] = c;
        result.mat[0][1] = -s;
        result.mat[1][0] = s;
        result.mat[1][1] = c;
        return result;
    }

    Vector3 MultiplyPoint(const Vector3& v) const {
        float x = v.x * mat[0][0] + v.y * mat[0][1] + v.z * mat[0][2] + mat[0][3];
        float y = v.x * mat[1][0] + v.y * mat[1][1] + v.z * mat[1][2] + mat[1][3];
        float z = v.x * mat[2][0] + v.y * mat[2][1] + v.z * mat[2][2] + mat[2][3];
        float w = v.x * mat[3][0] + v.y * mat[3][1] + v.z * mat[3][2] + mat[3][3];

        if (w != 0.0f) {
            x /= w;
            y /= w;
            z /= w;
        }

        return Vector3(x, y, z);
    }

    Vector3 MultiplyVector(const Vector3& v) const {
        float x = v.x * mat[0][0] + v.y * mat[0][1] + v.z * mat[0][2];
        float y = v.x * mat[1][0] + v.y * mat[1][1] + v.z * mat[1][2];
        float z = v.x * mat[2][0] + v.y * mat[2][1] + v.z * mat[2][2];
        return Vector3(x, y, z);
    }

    Matrix4x4 inverse() const {
        Matrix4x4 inv;

        float cof00 = mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1];
        float cof01 = mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0];
        float cof02 = mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0];

        float cof10 = mat[0][1] * mat[2][2] - mat[0][2] * mat[2][1];
        float cof11 = mat[0][0] * mat[2][2] - mat[0][2] * mat[2][0];
        float cof12 = mat[0][0] * mat[2][1] - mat[0][1] * mat[2][0];

        float cof20 = mat[0][1] * mat[1][2] - mat[0][2] * mat[1][1];
        float cof21 = mat[0][0] * mat[1][2] - mat[0][2] * mat[1][0];
        float cof22 = mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0];

        float det = mat[0][0] * cof00 - mat[0][1] * cof01 + mat[0][2] * cof02;
        if (fabs(det) < 1e-8f) {
            std::cerr << "Matrix inverse failed: determinant too small.\n";
            return Matrix4x4::identity();
        }

        float invDet = 1.0f / det;

        // Inverse 3x3 rotation-scale part
        inv.mat[0][0] =  (mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1]) * invDet;
        inv.mat[0][1] = -(mat[0][1] * mat[2][2] - mat[0][2] * mat[2][1]) * invDet;
        inv.mat[0][2] =  (mat[0][1] * mat[1][2] - mat[0][2] * mat[1][1]) * invDet;

        inv.mat[1][0] = -(mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0]) * invDet;
        inv.mat[1][1] =  (mat[0][0] * mat[2][2] - mat[0][2] * mat[2][0]) * invDet;
        inv.mat[1][2] = -(mat[0][0] * mat[1][2] - mat[0][2] * mat[1][0]) * invDet;

        inv.mat[2][0] =  (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]) * invDet;
        inv.mat[2][1] = -(mat[0][0] * mat[2][1] - mat[0][1] * mat[2][0]) * invDet;
        inv.mat[2][2] =  (mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0]) * invDet;

        // Inverse translation
        inv.mat[0][3] = -(inv.mat[0][0] * mat[0][3] + inv.mat[0][1] * mat[1][3] + inv.mat[0][2] * mat[2][3]);
        inv.mat[1][3] = -(inv.mat[1][0] * mat[0][3] + inv.mat[1][1] * mat[1][3] + inv.mat[1][2] * mat[2][3]);
        inv.mat[2][3] = -(inv.mat[2][0] * mat[0][3] + inv.mat[2][1] * mat[1][3] + inv.mat[2][2] * mat[2][3]);

        // Bottom row
        inv.mat[3][0] = 0.0f;
        inv.mat[3][1] = 0.0f;
        inv.mat[3][2] = 0.0f;
        inv.mat[3][3] = 1.0f;

        return inv;
    }

    void Print() const {
        for (int i = 0; i < 4; ++i) {
            std::cout << "[ ";
            for (int j = 0; j < 4; ++j)
                std::cout << mat[i][j] << " ";
            std::cout << "]\n";
        }
    }

    float mat[4][4];

private:
};


#endif //MATRIX4X4_H
