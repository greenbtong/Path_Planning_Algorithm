#pragma once
#include <utility>

using namespace std;

class maps {
public:
	maps() {
		int emptyMap[15][30] = { {0}
		};

		pair<int, int> obstacle[] = {
			make_pair(2, 2), make_pair(3, 2), make_pair(3, 3), make_pair(4,2), make_pair(5, 2), make_pair(9,2), make_pair(10, 2), make_pair(11,2), make_pair(12, 2), make_pair(13, 2), 
			make_pair(2, 12), make_pair(2,13), make_pair(2, 14), make_pair(2, 15), make_pair(2, 16), make_pair(3, 16), make_pair(3, 17),
			make_pair(5, 12), make_pair(5, 13), make_pair(5, 14), make_pair(5, 15), make_pair(6, 15), make_pair(7, 15), make_pair(8, 15), make_pair(9, 15),
			make_pair(3, 7), make_pair(4, 7), make_pair(5, 7), make_pair(6,7), make_pair(7, 7),
			make_pair(6, 19), make_pair(7, 19), make_pair(8, 19), make_pair(9, 19), make_pair(10, 19), make_pair(10, 20),
			make_pair(12, 11), make_pair(12, 12), make_pair(12, 13), make_pair(12, 14),
			make_pair(2, 22), make_pair(2, 23), make_pair(2, 24), make_pair(2,25), make_pair(2, 26), make_pair(2,27), make_pair(2, 28), make_pair(2,29),
			make_pair(5,26), make_pair(6, 26), make_pair(7,26), make_pair(8, 26), make_pair(9,26), make_pair(10, 26), make_pair(11,26), make_pair(12, 26), make_pair(13,26), make_pair(14, 26),  
		};

		for (const auto& p: obstacle) {
			emptyMap[p.first][p.second] = 1;
		}

		width = 15; height = 30;
		for (int r = 0; r < width; r++)
			for (int s = 0; s < height; s++)
				m[s][r] = emptyMap[r][s];
	}
	int operator() (int x, int y) { return m[x][y]; }
	int m[30][15], width, height;
	
};