#include "broadphase.h"

double BPPoint::get()
{
	if (begin)
	{
		if (axis == 'x') return shape->getAABB()->min.x;
		else return shape->getAABB()->min.y;
	}
	else
	{
		if (axis == 'x') return shape->getAABB()->max.x;
		else return shape->getAABB()->max.y;
	}
}

BPPair::BPPair(mShape* a, mShape* b)
{
	if (a->id < b->id)
	{
		A = a;
		B = b;
	}
	else
	{
		A = b;
		B = a;
	}
}

bool BPPair::operator==(const BPPair BP) const
{
	return (A == BP.A and B == BP.B);
}

void BroadPhase::addAABB(AABB* aabb)
{
	axis_x.push_back(new BPPoint(aabb->shape, true, 'x'));
	axis_x.push_back(new BPPoint(aabb->shape, false, 'x'));
	axis_y.push_back(new BPPoint(aabb->shape, true, 'y'));
	axis_y.push_back(new BPPoint(aabb->shape, false, 'y'));
}

void BroadPhase::Remove(Body* b)
{
	for (auto it = axis_x.begin(); it != axis_x.end(); it++)
	{
		if (b == (*it)->shape->body)
		{
			delete *it;
			axis_x.erase(it);
			it--;
		}
	}
	for (auto it = axis_y.begin(); it != axis_y.end(); it++)
	{
		if (b == (*it)->shape->body)
		{
			delete *it;
			axis_y.erase(it);
			it--;
		}
	}
	std::unordered_set<BPPair> tmppair(pair);
	pair.clear();
	for (auto it = tmppair.begin(); it != tmppair.end(); it++)
		if (!(b == it->A->body or b == it->B->body))
			pair.emplace(*it);
}

void BroadPhase::sort()
{	
	sortaxis(true);
	sortaxis(false);
}

void BroadPhase::Flush()
{
	pair.clear();
	for (int i = 0; i < axis_x.size(); i++)
		delete axis_x[i];
	axis_x.clear();
	for (int i = 0; i < axis_y.size(); i++)
		delete axis_y[i];
	axis_y.clear();
}

void BroadPhase::sortaxis(bool x)
{
	std::vector<BPPoint*>* axis;
	if (x) axis = &axis_x;
	else axis = &axis_y;
	for (int j = 1; j < (*axis).size(); j++)
	{
		BPPoint* keyelem = (*axis)[j];
		double key = keyelem->get();
		int i = j - 1;
		while (i >= 0 && (*axis)[i]->get() > key)
		{
			BPPoint* swap = (*axis)[i];
			if (keyelem->begin && !swap->begin)
				if (AABBtest(swap->shape->getAABB(), keyelem->shape->getAABB()))
					pair.emplace(swap->shape, keyelem->shape);
			if (!keyelem->begin && swap->begin)
				pair.erase(BPPair(swap->shape, keyelem->shape));
			(*axis)[i + 1] = swap;
			i--;
		}
		(*axis)[i + 1] = keyelem;
	}
}