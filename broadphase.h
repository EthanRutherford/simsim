#ifndef BROADPHASE_H
#define BROADPHASE_H

#include "body.h"
#include <vector>
#include <unordered_set>

struct BPPoint{
	mShape* shape;
	bool begin;
	char axis;
	BPPoint(mShape* b, bool B, char a) : shape(b), begin(B), axis(a) {}
	double get();
};

struct BPPair{
	mShape* A;
	mShape* B;
	BPPair(mShape* a, mShape* b);
	bool operator==(const BPPair BP) const;
};

namespace std{template<>
struct hash<BPPair>{
	size_t operator()(const BPPair &a ) const
	{return  hash<int>()((a.A->id << 16)^ a.B->id);}
};}

class BroadPhase{
	public:
		std::unordered_set<BPPair> pair;
		void addAABB(AABB* aabb);
		void Remove(Body* b);
		void sort();
		void Flush();
	private:
		void sortaxis(bool x);
		std::vector<BPPoint*> axis_x;
		std::vector<BPPoint*> axis_y;
};

#endif