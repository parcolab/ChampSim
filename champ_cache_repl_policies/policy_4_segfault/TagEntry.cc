#include "TagEntry.h"

TagEntry::TagEntry() {
	tag = 0;
	valid = true;
	for(int i = 0; i < HISTORY_SIZE; i++)
		hit_count[i] = 1;
	ptr = 0;
}

void TagEntry::clear() {
	tag = 0;
	valid = true;
	for (int i = 0; i < HISTORY_SIZE; i++)
		hit_count[i] = 1;
	ptr = 0;
}

void TagEntry::add(int hit_count) {
	this->hit_count[ptr] = hit_count;
	ptr++;
	ptr %= HISTORY_SIZE;
}

int TagEntry::get_hit_count() {
	int sum = 0;
	for(int i = 0; i < HISTORY_SIZE; i++)
		sum += hit_count[i];

	return sum ;
}
