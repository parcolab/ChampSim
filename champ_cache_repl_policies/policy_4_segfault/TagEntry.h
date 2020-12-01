#ifndef TAGENTRY_H_
#define TAGENTRY_H_

#include "Block.h"
#include "util.h"

class TagEntry {
public:
	TagEntry();

	void add(int hit_count);
	int get_hit_count();
	void clear();

	uint8_t valid;
	uint64_t tag;
	int32_t hit_count[HISTORY_SIZE];
	uint8_t ptr;
};

#endif
