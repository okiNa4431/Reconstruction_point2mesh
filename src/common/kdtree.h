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
		//tree�̏����l�̐ݒ�
		std::vector<Point> points_copy(points.begin(), points.end());
		int size = points_copy.size();
		int left = 0, right=size;
		root = subConstract(points_copy, left, right);
		printf("kdtree�̍\�z����\n");
	}

    std::vector<Point> nearestsbyBall(const Point& point, const int pointsNum, double& radius) {
        //���̃L���[�Ɉ����̓_����̋����Ɠ_�̃C���f�b�N�X������
        std::priority_queue<std::pair<double, Node*>> pQueue;
        
        //�L���[�̐���pointsNum�������Ȃ����͔��a��傫�����čēx�T��
        while (pQueue.size() < pointsNum) {
            std::priority_queue<std::pair<double, Node*>> pQueue_reset;
            pQueue = pQueue_reset;
            subNearestsbyBall(point, pQueue, root, radius);
            if(pQueue.size() < pointsNum) radius *= 1.2;
            else if(pQueue.size() > pointsNum*1.5) radius /= 1.2;
        }

        //queue�̒�����pointsNum-1�Ԗڂ܂ł̗v�f�𔲂��o���Ĕz��ɓ����
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
		//points�ɒl�������Ă��Ȃ����nullptr��Ԃ�
		if (left >= right) {
			return nullptr;
		}

		if (right - left == 1) {
			Node* node = new Node();
			node->point = points[left];
		}

		//�e�������̕��U���v�Z
		const int count = right - left;
		double aveX = 0.0, aveY = 0.0, aveZ = 0.0;
		double varX = 0.0, varY = 0.0, varZ = 0.0;
		//���ς��v�Z
		for (int i = left; i < right; i++) {
			aveX += points[i].x / count;
			aveY += points[i].y / count;
			aveZ += points[i].z / count;
		}
		//���U���v�Z
		for (int i = left; i < right; i++) {
			varX += (points[i].x - aveX) * (points[i].x - aveX) / count;
			varY += (points[i].y - aveY) * (points[i].y - aveY) / count;
			varZ += (points[i].z - aveZ) * (points[i].z - aveZ) / count;
		}

		//���U���ł��傫���Ȃ鎲���������߂�
		int maxAxis;
		double maxVar = std::max(std::max(varX, varY), varZ);
		if (maxVar == varX) maxAxis = 0;
		else if (maxVar == varY) maxAxis = 1;
		else maxAxis = 2;

		//���߂��������̍��W�l�������ŕ��ׂ�
		std::sort(points.begin() + left, points.begin() + right, [&](Point const& lhs, Point const& rhs) {return lhs[maxAxis] < rhs[maxAxis]; });

		//�����ɕ������邽�߂̗v�f�����o��
		int midthres = (right+left) / 2;
		Point medialPoint = points[midthres];

		//�����ŒT�����������m�[�h�ɓ���čċA�I�ɍ\�z���Ă���
		auto rootNode = new Node();
		rootNode->point = medialPoint;
		rootNode->leftchild = subConstract(points, left, midthres);
		rootNode->rightchild = subConstract(points, midthres+1, right);
		rootNode->axis = maxAxis;
		nodes.push_back(rootNode);
		return rootNode;
	}

    //���ɓ����Ă��邩�ǂ������v�Z������@�ɂ��k�ߖT�T��
    void subNearestsbyBall(const Point& query, std::priority_queue<std::pair<double, Node*>>& pQueue, Node* node, const double& radius) {
        if (node == nullptr) {
            return;
        }
        //���Ɋ܂܂�Ă�����candidate���L���[�ɓ����
        const Point candidate = node->point;
        const double distance = query.distance(candidate);
        if (distance < radius) {
            pQueue.push(std::make_pair(-distance, node));
        }

        const int axis = node->axis;
        const double diff = fabs(query[axis] - candidate[axis]);

        //���a�����������̋������傫�����͋߂����̃m�[�h�̂ݒT������
        if (diff > radius) {
            if (query[axis] < candidate[axis]) {
                subNearestsbyBall(query, pQueue, node->leftchild, radius);
            }
            else {
                subNearestsbyBall(query, pQueue, node->rightchild, radius);
            }
        }
        //���a�����������̋������傫�����͂ǂ���̃m�[�h���T������
        else {
            subNearestsbyBall(query, pQueue, node->leftchild, radius);
            subNearestsbyBall(query, pQueue, node->rightchild, radius);
        }
    }

	Node* root = nullptr;
	std::vector<Node*> nodes;
	double maxDistance=-1.0;
};