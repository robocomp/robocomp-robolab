#ifndef BIASEDSELECTORALGORITHM_H
#define BIASEDSELECTORALGORITHM_H

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include <algorithm>
#include <vector>
#include <string>

#define BIASED_MULT 10000.

struct BiasedCandidate
{
public:
	uint32_t id;
	double weight;
	double accum;
	bool operator()(BiasedCandidate const& a, BiasedCandidate const& b) const
	{
		return a.weight > b.weight;
	}
};

class BiasedSelector
{
public:
	// Constructor. The parameter 'size' stands for the maximum number of candidates.
	BiasedSelector(uint32_t size=1)
	{
		resize(size, true);
		totalWeight = 0.;
	}
	// Constructor. The parameter 'size' stands for the maximum number of candidates.
	BiasedSelector(const BiasedSelector &other)
	{
		copyFrom(other);
	}
	BiasedSelector &operator=(const BiasedSelector &other)
	{
		copyFrom(other);
		return *this;
	}

	void copyFrom(const BiasedSelector &other)
	{
		candidates = other.candidates;
		totalWeight = other.totalWeight;
	}

	void resize(size_t size, bool initialize=false)
	{
		size_t prevSize = candidates.size();
		candidates.resize(size);
		if (prevSize<size or initialize==true)
		{
			for (size_t i=initialize?0:prevSize; i<candidates.size(); ++i)
			{
				candidates[i].id = i;
				candidates[i].weight = 0.;
			}
		}
	}
	uint32_t get()
	{
		const uint64_t randomValue = ((uint64_t)(rand() * rand()));
		const uint64_t module = (uint64_t)(floor(totalWeight));
		const uint64_t result = randomValue % module;
		for (size_t i=0; i<candidates.size(); i++)
		{
			if (candidates[i].accum >= result)
				return candidates[i].id;
		}
		//printf("BiasedSelector::WTF!? 1\n");
		throw std::string("BiasedSelector::Error: No possible choice.");
	}
	double getWeight(uint32_t id)
	{
// 		printf("BiasedSelector:getw %lu", candidates.size());
		for (size_t i=0; i<candidates.size(); ++i)
		{
			if (candidates[i].id == id)
			{
				return candidates[i].weight / BIASED_MULT;
			}
		}
		//printf("BiasedSelector::WTF!? 2\n");
		throw "BiasedSelector::WTF!? 2";
	}
	bool setWeight(uint32_t id, double p, bool perform_sort=true)
	{
// 		printf("BiasedSelector:setw %lu", candidates.size());
		if (p<0.)
			return false;
		for (size_t i=0; i<candidates.size(); ++i)
		{
			if (candidates[i].id == id)
			{
				totalWeight -= candidates[i].weight;
				candidates[i].weight = BIASED_MULT * p;
				totalWeight += candidates[i].weight;
				if (perform_sort)
					sort();
			}
		}
		return false;
	}
	void sort()
	{
// 		printf("BiasedSelector:sort %lu", candidates.size());
		std::sort(candidates.begin(), candidates.end(), BiasedCandidate());
		double accum = 0;
		for (size_t i=0; i<candidates.size(); ++i)
		{
			accum += candidates[i].weight;
			candidates[i].accum = accum;
		}
	}
	void print()
	{
// 		printf("BiasedSelector:print %lu", candidates.size());
		printf("Biased selector weights (%g)\n", totalWeight);
		for (size_t i=0; i<candidates.size(); i++)
		{
// 			printf("dd\n");
			printf("ID: %d\tweight:%10.5g\taccum:%10.5g\n", candidates[i].id, candidates[i].weight, candidates[i].accum);
		}
	}

// private:
	std::vector<BiasedCandidate> candidates;
// 	uint32_t size;
	double totalWeight;
};

#endif
