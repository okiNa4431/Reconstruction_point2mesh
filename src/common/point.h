#pragma once

#include <stdexcept>
#include "array.h"

struct Point {
public:
	Point(){
        x = 0; y = 0; z = 0;
        index = -1;
    }
    Point(double x, double y, double z) {
        this->x = x;
        this->y = y;
        this->z = z;
    }
	//座標情報
	double x=0,y=0,z=0;
    //色情報
    unsigned char r = 255, g = 255, b = 255;
    //CT値
    unsigned short CTval = 0;
	//凸包で隣り合う三角形を見るときの頂点番号
	int index=-1;
	//点の持つ法線情報
	Array normal;

	//添字演算子をオーバーロード
	double& operator[](size_t pos) {
		if (pos < 0 || pos >= 3) {
			throw std::runtime_error("Vector index out of bounds!");
		}
		switch (pos) {
			case 0:
				return x;
			case 1:
				return y;
			case 2:
				return z;

		}
	}
	const double& operator[](size_t pos) const{
		if (pos < 0 || pos >= 3) {
			throw std::runtime_error("Vector index out of bounds!");
		}
		switch (pos) {
		case 0:
			return x;
		case 1:
			return y;
		case 2:
			return z;

		}
	}
	inline Point operator+(const Point &p) {
		Point res;
		res.x = this->x + p.x;
		res.y = this->y + p.y;
		res.z = this->z + p.z;
		return res;
	}

	inline const Point operator+(const Point& p) const{
		Point res;
		res.x = this->x + p.x;
		res.y = this->y + p.y;
		res.z = this->z + p.z;
		return res;
	}
    inline Point operator-(const Point& p) {
        Point res;
        res.x = this->x - p.x;
        res.y = this->y - p.y;
        res.z = this->z - p.z;
        return res;
    }

    inline const Point operator-(const Point& p) const {
        Point res;
        res.x = this->x - p.x;
        res.y = this->y - p.y;
        res.z = this->z - p.z;
        return res;
    }

	inline Point operator/(int num) {
		Point res;
		res.x = this->x / (double)num;
		res.y = this->y / (double)num;
		res.z = this->z / (double)num;
		return res;
	}

	inline const Point operator/(int num) const{
		Point res;
		res.x = this->x / (double)num;
		res.y = this->y / (double)num;
		res.z = this->z / (double)num;
		return res;
	}

    inline Point operator=(Point p) {
        this->x = p.x; this->y = p.y; this->z = p.z;
        this->normal.x = p.normal.x; this->normal.y = p.normal.y; this->normal.z = p.normal.z;
        this->CTval = p.CTval; this->r = p.r; this->g = p.g; this->b = p.b;
        this->index = p.index;
        return *this;
    }

    //比較演算子をオーバーロード
    bool operator==(const Point rhs) {
        return index == rhs.index;
    }
    const bool operator==(const Point rhs) const {
        return index == rhs.index;
    }

    bool operator!=(const Point rhs) {
        return index != rhs.index;
    }
    const bool operator!=(const Point rhs) const {
        return index != rhs.index;
    }

    bool operator<(const Point rhs) {
        return index < rhs.index;
    }
    const bool operator<(const Point rhs) const {
        return index < rhs.index;
    }

    bool operator<=(const Point rhs) {
        return index <= rhs.index;
    }
    const bool operator<=(const Point rhs) const {
        return index <= rhs.index;
    }

    bool operator>(const Point rhs) {
        return index > rhs.index;
    }
    const bool operator>(const Point rhs) const {
        return index > rhs.index;
    }

    bool operator>=(const Point rhs) {
        return index >= rhs.index;
    }
    const bool operator>=(const Point rhs) const {
        return index >= rhs.index;
    }

	//引数のPointとの距離を求める
	const double distance(const Point p) const{
		return sqrt((x - p.x) * (x - p.x) + (y - p.y) * (y - p.y) + (z - p.z) * (z - p.z));
	}

	//引数のPointとの距離の二乗を求める
	const double squareDistance(const Point p) const {
		return (x - p.x) * (x - p.x) + (y - p.y) * (y - p.y) + (z - p.z) * (z - p.z);
	}
    //引数のポイントとの差をベクトルとして求める
    Array differentialVector(const Point p) {
        Array res;
        res.x = x - p.x;
        res.y = y - p.y;
        res.z = z - p.z;
        return res;
    }
	const Array differentialVector(const Point p) const {
		Array res;
		res.x = x - p.x;
		res.y = y - p.y;
		res.z = z - p.z;
		return res;
	}
    //デバッグ用の点の座標と法線の表示
    inline void displayD() {
        printf("座標: %lf, %lf, %lf  法線: %lf, %lf, %lf インデックス: %d\n", x, y, z, normal.x, normal.y, normal.z, index);
    }
    //デバッグ用の色変換(赤)
    void convertRed() {
        r = 255; g = 0; b = 0;
    }
    //デバッグ用の色変換(緑)
    void convertGreen() {
        r = 0; g = 255; b = 0;
    }
    //デバッグ用の色変換(青)
    void convertBlue() {
        r = 0; g = 0; b = 255;
    }
    //デバッグ用の色変換(白)
    void convertWhite() {
        r = 255; g = 255; b = 255;
    }
};