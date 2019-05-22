/* Astar path finder - 30 x 15 map - 
Start point is random on the Y-axis & 0 on the X-axis -
End point is at (14, 29) - 
Cost, computational time, graph, and path are outputted
*/

#include "pathPlanning.h"
#include <time.h>
#include <stdlib.h>
#include <ctime>

int main() {
	srand((time(NULL)));
	clock_t clockStart = clock();
	int randS = rand() % 14;
	maps m;
	point startP(0,randS), endP(29,14);
	aStar as;

	if (as.check(startP, endP, m)) {
		list<point> path;
		int c = as.path(path);
		for (int y = 0; y < m.width; y++) {
			for (int x = 0; x < m.height; x++) {
				if (m(x, y) == 1)
					cout << "@";
				else {
					if (point(x, y) == startP)
						cout << "S";
					else if (point(x, y) == endP)
						cout << "E";
					else if (find(path.begin(), path.end(), point(x, y)) != path.end())
						cout << "x";
					else cout << ".";
				}
			}
			cout << "\n";
		}

		clock_t clockEnd = clock();
		float timeCal = ((float)clockEnd - (float)clockStart);
		cout << "\nTime to Calculate the route: " << timeCal << "ms" << endl;
		cout << "\nStart: " << "(" << startP.x << ", " << startP.y << ") " << "\nEnd: " << "(" << endP.x << ", " << endP.y << ") ";
		cout << "\nPath cost " << c << ": ";
		for (list<point>::iterator i = path.begin(); i != path.end(); i++) {
			cout << "(" << (*i).x << ", " << (*i).y << ") ";
		}
		
	}
	cout << endl;
	return 0;
}
