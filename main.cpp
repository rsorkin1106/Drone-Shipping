// Project Identifier: 1761414855B69983BD8035097EFBD312EB0527F0
#include "drone.cpp"
#include <iomanip>


int main(int argc, char** argv)
{
	ios_base::sync_with_stdio(false);
	cout << setprecision(2);
	cout << fixed;

	Drone d;
	d.getOptions(argc, argv);
	d.getCoords();


	return 0;
}