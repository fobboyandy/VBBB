#ifndef cell_h
#define cell_h

struct cell
{
	enum cell_t
	{
		blank,
		wall,
		path
	};
	cell() {};
	cell(size_t xC, size_t yC, cell_t cT) :x(xC), y(yC), type(cT) {};
	size_t x, y;
	cell_t type;
};

#endif
