#pragma once
#include "point.h"
#include "array.h"
#include <vector>
#include <algorithm>

struct Triangle {
    Point vertex1;
    Point vertex2;
    Point vertex3;
    float red;
    float green;
    float blue;
    unsigned short CTVal;
    bool virtualTri = false;
    int partIdx = -1;
    int secondPartIdx = -1;

    Triangle() {}
    //vertex1のindexが一番小さくなるように設定
    Triangle(Point s, Point t, Point u) {
        std::vector<Point> tmp = { s,t,u };
        std::sort(tmp.begin(), tmp.end());
        this->vertex1 = tmp[0];
        this->vertex2 = tmp[1];
        this->vertex3 = tmp[2];

        //デバッグ用のエラーコード
        if (s == t || t == u || u == s) {
            //printf("error: 3点が異なるという条件を満たしていない(%d %d %d)\n",s.index, t.index, u.index);
        }
    }

    //比較演算子をオーバーロード
    bool operator==(const Triangle rhs) {
        return vertex1 == rhs.vertex1 && vertex2 == rhs.vertex2 && vertex3 == rhs.vertex3;
    }
    const bool operator==(const Triangle rhs) const {
        return vertex1 == rhs.vertex1 && vertex2 == rhs.vertex2 && vertex3 == rhs.vertex3;
    }
    bool operator!=(const Triangle rhs) {
        return vertex1 != rhs.vertex1 || vertex2 != rhs.vertex2 || vertex3 != rhs.vertex3;
    }
    const bool operator!=(const Triangle rhs) const {
        return vertex1 != rhs.vertex1 || vertex2 != rhs.vertex2 || vertex3 != rhs.vertex3;
    }
    bool operator<(const Triangle rhs) {
        return vertex1 < rhs.vertex1 || (vertex1 == rhs.vertex1 && vertex2 < rhs.vertex2) || (vertex1 == rhs.vertex1 && vertex2 == rhs.vertex2 && vertex3 < rhs.vertex3);
    }
    const bool operator<(const Triangle rhs) const {
        return vertex1 < rhs.vertex1 || (vertex1 == rhs.vertex1 && vertex2 < rhs.vertex2) || (vertex1 == rhs.vertex1 && vertex2 == rhs.vertex2 && vertex3 < rhs.vertex3);
    }
    bool operator<=(const Triangle rhs) {
        return vertex1 <= rhs.vertex1 || (vertex1 == rhs.vertex1 && vertex2 <= rhs.vertex2) || (vertex1 == rhs.vertex1 && vertex2 == rhs.vertex2 && vertex3 <= rhs.vertex3);
    }
    const bool operator<=(const Triangle rhs) const {
        return vertex1 <= rhs.vertex1 || (vertex1 == rhs.vertex1 && vertex2 <= rhs.vertex2) || (vertex1 == rhs.vertex1 && vertex2 == rhs.vertex2 && vertex3 <= rhs.vertex3);
    }
    bool operator>(const Triangle rhs) {
        return vertex1 > rhs.vertex1 || (vertex1 == rhs.vertex1 && vertex2 > rhs.vertex2) || (vertex1 == rhs.vertex1 && vertex2 == rhs.vertex2 && vertex3 > rhs.vertex3);
    }
    const bool operator>(const Triangle rhs) const {
        return vertex1 > rhs.vertex1 || (vertex1 == rhs.vertex1 && vertex2 > rhs.vertex2) || (vertex1 == rhs.vertex1 && vertex2 == rhs.vertex2 && vertex3 > rhs.vertex3);
    }
    bool operator>=(const Triangle rhs) {
        return vertex1 >= rhs.vertex1 || (vertex1 == rhs.vertex1 && vertex2 >= rhs.vertex2) || (vertex1 == rhs.vertex1 && vertex2 == rhs.vertex2 && vertex3 >= rhs.vertex3);
    }
    const bool operator>=(const Triangle rhs) const {
        return vertex1 >= rhs.vertex1 || (vertex1 == rhs.vertex1 && vertex2 >= rhs.vertex2) || (vertex1 == rhs.vertex1 && vertex2 == rhs.vertex2 && vertex3 >= rhs.vertex3);
    }

    //頂点から面積を求める関数
    double calcArea() {
        const Array arr1 = vertex1.differentialVector(vertex2);
        const Array arr2 = vertex1.differentialVector(vertex3);
        const Array crossArray = arr1.cross(arr2);
        return crossArray.norm()/(double)2.0;
    }

    //3頂点から面の法線を求める関数
    Array calcNormal() {
        const Array arr1 = vertex1.differentialVector(vertex2);
        const Array arr2 = vertex1.differentialVector(vertex3);
        Array normal = arr1.cross(arr2);
        normal.normalize();
        return normal;
    }
    //3頂点を通る平面の係数を返す関数
    std::vector<double> calcCofficients() {
        std::vector<double> ret(4);
        Array normal = this->calcNormal();
        ret[0] = normal.x; ret[1] = normal.y; ret[2] = normal.z;
        ret[3] = -(ret[0] * this->vertex1.x + ret[1] * this->vertex1.y + ret[2] * this->vertex1.z);
        return ret;
    }

    //三角形の1点を変換する関数
    void convertPoint(Point from, Point to) {
        if (vertex1 == from) this->vertex1 = to;
        else if (vertex2 == from) this->vertex2 = to;
        else if (vertex3 == from) this->vertex3 = to;
        sortVertices();
    }

    //指定した二点を変換する関数
    bool convertPoints(Point from1, Point from2, Point to){
        if (vertex1 == from1) {
            if (vertex2 == from2 || vertex3 == from2) return false;
            else {
                this->vertex1 = to;
                sortVertices();
                return true;
            }
        }
        else if (vertex2 == from1) {
            if (vertex3 == from2 || vertex1 == from2) return false;
            else {
                this->vertex2 = to;
                sortVertices();
                return true;
            }
        }
        else if (vertex3 == from1) {
            if (vertex1 == from2 || vertex2 == from2) return false;
            else {
                this->vertex3 = to;
                sortVertices();
                return true;
            }
        }

        if (vertex1 == from2) {
            if (vertex2 == from1 || vertex3 == from1) return false;
            else {
                this->vertex1 = to;
                sortVertices();
                return true;
            }
        }
        else if (vertex2 == from2) {
            if (vertex3 == from1 || vertex1 == from1) return false;
            else {
                this->vertex2 = to;
                sortVertices();
                return true;
            }
        }
        else if (vertex3 == from2) {
            if (vertex1 == from1 || vertex2 == from1) return false;
            else {
                this->vertex3 = to;
                sortVertices();
                return true;
            }
        }
        return true;
    }

    //3頂点の色の平均を三角形の色として設定する関数。減色を行う(objファイル用)
    void setColor(float divide) {
        const unsigned char r = (this->vertex1.r + this->vertex2.r + this->vertex3.r) / 3;
        const unsigned char g = (this->vertex1.g + this->vertex2.g + this->vertex3.g) / 3;
        const unsigned char b = (this->vertex1.b + this->vertex2.b + this->vertex3.b) / 3;

        this->red = std::round(divide * (float)r / 255.0) / divide;
        this->green = std::round(divide * (float)g / 255.0) / divide;
        this->blue = std::round(divide * (float)b / 255.0) / divide;
    }

    void setCT() {
        this->CTVal = (vertex1.CTval + vertex2.CTval + vertex3.CTval) / 3;
    }

    //頂点が全て同じ色かどうかを判定する
    const bool checkSameColor() const {
        if (vertex1.r == vertex2.r && vertex2.r == vertex3.r && vertex1.g == vertex2.g && vertex2.g == vertex3.g && vertex1.b == vertex2.b && vertex2.b == vertex3.b) return true;
        else return false;
    }

private:
    void sortVertices() {
        std::vector<Point> tmp = { vertex1,vertex2,vertex3 };
        std::sort(tmp.begin(), tmp.end());
        this->vertex1 = tmp[0];
        this->vertex2 = tmp[1];
        this->vertex3 = tmp[2];
    }
};