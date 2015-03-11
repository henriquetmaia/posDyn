#include <iostream>
#include "Viewer.h"

using namespace std;
using namespace DDG;

int main( int argc, char** argv )
{
	if( argc < 2 )
	{
		cerr << "usage: " << argv[0] << " first.obj < second.obj third.obj ... > " << endl;
		return 1;
	}

	Viewer viewer;
	for( int arg = 1; arg < argc; ++arg ){
		viewer.sim.addMesh( argv[arg] );
	}
	viewer.init();

	return 0;
}

