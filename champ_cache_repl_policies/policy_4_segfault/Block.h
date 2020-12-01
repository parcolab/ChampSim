/*
 * Block.h
 *
 *  Created on: Sep 28, 2016
 *      Author: armin
 */

#ifndef BLOCK_H_
#define BLOCK_H_

#include "util.h"

class Block {
public:
	int hit_count;
	int rrpv;
	Block();
	virtual ~Block();

};

#endif /* BLOCK_H_ */
