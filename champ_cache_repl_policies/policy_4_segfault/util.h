/*
 * util.h
 *
 *  Created on: Sep 28, 2016
 *      Author: armin
 */

#ifndef UTIL_H_
#define UTIL_H_

#include <iostream>
#include <cstdint>
#include <cstdlib>
#include<vector>
typedef uint8_t byte;

#define INF 1000*1000*1000
#define EPS 1e-5

#define HISTORY_SIZE 2

#define EXIT_FAILURE 1

#define PRINT(X) std::cerr << #X << " : " << X << "\n";

#define maxRRPV 7
#define maxHITCOUNT 7
#define PROB_BYPASS_NON_LOAD ((double)7/8)
#define PROB_BYPASS_HIT_0 ((double)31/32)

inline int logarithm2(uint64_t x) {
	int cnt = 0;
	while(x > 1) {
		cnt++;
		x >>= 1;
	}
	return cnt;
}

class RepData {
public:
	RepData() {
		size_of_tag_array = 0;
		rrip_bits = 0;
	}

	int size_of_tag_array;
	int rrip_bits;
};

#endif /* UTIL_H_ */
