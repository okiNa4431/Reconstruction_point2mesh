#pragma once

#include <vector>
#include <algorithm>
#include <queue>
#include "point.h"

struct Node {
public:
	Node() {}
	bool isLeaf() {
		return axis == -1;
	}
	Point point;
	Node* leftchild = nullptr;
	Node* rightchild = nullptr;
	int axis = -1;
};

class kdtree {
public:
	kdtree() {}
	virtual ~kdtree() {
		clear();
	}
	void clear() {
		for (Node* node : nodes) {
			delete node;
		}
		nodes.clear();
	}
	void constract(const std::vector<Point> &points) {
		//treeの初期値の設定
		std::vector<Point> points_copy(points.begin(), points.end());
		int size = points_copy.size();
		int left = 0, right=size;
		root = subConstract(points_copy, left, right);
		printf("kdtreeの構築完了\n");
	}

    std::vector<Point> nearestsbyBall(const Point& point, const int pointsNum, double& radius) {
        //このキューに引数の点からの距離と点のインデックスを入れる
        std::priority_queue<std::pair<double, Node*>> pQueue;
        
        //キューの数がpointsNumよりも少ない時は半径を大きくして再度探索
        while (pQueue.size() < pointsNum) {
            std::priority_queue<std::pair<double, Node*>> pQueue_reset;
            pQueue = pQueue_reset;
            subNearestsbyBall(point, pQueue, root, radius);
            if(pQueue.size() < pointsNum) radius *= 1.2;
            else if(pQueue.size() > pointsNum*1.5) radius /= 1.2;
        }

        //queueの中からpointsNum-1番目までの要素を抜き出して配列に入れる
        std::vector<Point> ans(pointsNum);
        for (int i = 0; i < pointsNum; i++) {
            const std::pair<double, Node*> tmp = pQueue.top();
            pQueue.pop();
            ans[i] = tmp.second->point;
        }

        return ans;
    }

private:
	Node* subConstract(std::vector<Point>& points, int left,int right) {
		//pointsに値が入っていなければnullptrを返す
		if (left >= right) {
			return nullptr;
		}

		if (right - left == 1) {
			Node* node = new Node();
			node->point = points[left];
		}

		//各軸方向の分散を計算
		const int count = right - left;
		double aveX = 0.0, aveY = 0.0, aveZ = 0.0;
		double varX = 0.0, varY = 0.0, varZ = 0.0;
		//平均を計算
		for (int i = left; i < right; i++) {
			aveX += points[i].x / count;
			aveY += points[i].y / count;
			aveZ += points[i].z / count;
		}
		//分散を計算
		for (int i = left; i < right; i++) {
			varX += (points[i].x - aveX) * (points[i].x - aveX) / count;
			varY += (points[i].y - aveY) * (points[i].y - aveY) / count;
			varZ += (points[i].z - aveZ) * (points[i].z - aveZ) / count;
		}

		//分散が最も大きくなる軸方向を求める
		int maxAxis;
		double maxVar = std::max(std::max(varX, varY), varZ);
		if (maxVar == varX) maxAxis = 0;
		else if (maxVar == varY) maxAxis = 1;
		else maxAxis = 2;

		//求めた軸方向の座標値を昇順で並べる
		std::sort(points.begin() + left, points.begin() + right, [&](Point const& lhs, Point const& rhs) {return lhs[maxAxis] < rhs[maxAxis]; });

		//半分に分割するための要素を取り出す
		int midthres = (right+left) / 2;
		Point medialPoint = points[midthres];

		//ここで探索した情報をノードに入れて再帰的に構築していく
		auto rootNode = new Node();
		rootNode->point = medialPoint;
		rootNode->leftchild = subConstract(points, left, midthres);
		rootNode->rightchild = subConstract(points, midthres+1, right);
		rootNode->axis = maxAxis;
		nodes.push_back(rootNode);
		return rootNode;
	}

    //球に入っているかどうかを計算する方法によるk近傍探索
    void subNearestsbyBall(const Point& query, std::priority_queue<std::pair<double, Node*>>& pQueue, Node* node, const double& radius) {
        if (node == nullptr) {
            return;
        }
        //球に含まれていたらcandidateもキューに入れる
        const Point candidate = node->point;
        const double distance = query.distance(candidate);
        if (distance < radius) {
            pQueue.push(std::make_pair(-distance, node));
        }

        const int axis = node->axis;
        const double diff = fabs(query[axis] - candidate[axis]);

        //半径よりも軸方向の距離が大きい時は近い方のノードのみ探索する
        if (diff > radius) {
            if (query[axis] < candidate[axis]) {
                subNearestsbyBall(query, pQueue, node->leftchild, radius);
            }
            else {
                subNearestsbyBall(query, pQueue, node->rightchild, radius);
            }
        }
        //半径よりも軸方向の距離が大きい時はどちらのノードも探索する
        else {
            subNearestsbyBall(query, pQueue, node->leftchild, radius);
            subNearestsbyBall(query, pQueue, node->rightchild, radius);
        }
    }

	Node* root = nullptr;
	std::vector<Node*> nodes;
	double maxDistance=-1.0;
};